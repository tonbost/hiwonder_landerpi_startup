#!/usr/bin/env python3
"""
Autonomous exploration for LanderPi robot.
Runs directly ON the robot - not via SSH from host.

Usage (on robot):
    python3 ~/robot_explorer.py explore --duration 5
    python3 ~/robot_explorer.py explore --duration 30
    python3 ~/robot_explorer.py stop
    python3 ~/robot_explorer.py status
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional, List, Dict, Any

# Configuration
LOG_DIR = Path.home() / "landerpi" / "exploration_logs"
DOCKER_EXEC = "docker exec landerpi-ros2 bash -c"
ROS_SOURCE = "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash 2>/dev/null;"


@dataclass
class SectorData:
    """Data for a lidar sector."""
    index: int
    name: str
    center_angle: float  # degrees from front (0 = front, positive = left)
    min_distance: float = float('inf')
    max_distance: float = 0.0
    point_count: int = 0


@dataclass
class ExplorationConfig:
    """Exploration configuration."""
    # Motion
    forward_speed: float = 0.35  # m/s (fast when clear)
    turn_speed: float = 1.5  # rad/s

    # Safety - closer approach to obstacles
    obstacle_stop_dist: float = 0.15  # m - emergency stop (can get 15cm from walls)
    obstacle_slow_dist: float = 0.30  # m - slow down
    sector_blocked_dist: float = 0.40  # m - sector considered blocked

    # Timing
    loop_rate_hz: float = 8.0  # Control loop rate (faster)
    stuck_timeout: float = 8.0  # seconds without progress = stuck
    escape_duration: float = 1.5  # seconds to back up

    # Turn commitment - prevents oscillation
    min_turn_duration: float = 0.6  # seconds to commit to a turn direction
    turn_complete_threshold: float = 25.0  # degrees - consider turn complete when within this

    # Battery
    battery_cutoff: float = 6.6  # V
    battery_warning: float = 7.0  # V


@dataclass
class ExplorationState:
    """Current exploration state."""
    running: bool = False
    action: str = "idle"
    start_time: float = 0.0
    max_duration: float = 0.0
    last_movement_time: float = 0.0
    battery_voltage: float = 0.0
    lidar_samples: int = 0
    events: List[Dict] = field(default_factory=list)
    # Turn commitment tracking
    turn_start_time: float = 0.0
    turn_direction: int = 0  # -1 = right, 0 = none, 1 = left
    turn_target_sector: str = ""


class DataLogger:
    """Logs exploration data directly to filesystem."""

    def __init__(self, base_dir: Path = LOG_DIR):
        self.base_dir = base_dir
        self.session_dir: Optional[Path] = None
        self.lidar_file: Optional[Path] = None
        self.events_file: Optional[Path] = None
        self.odom_file: Optional[Path] = None
        self.start_time: float = 0
        self.lidar_count = 0
        self.event_count = 0
        self.last_lidar_log: float = 0

    def start_session(self) -> Path:
        """Start new logging session."""
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.session_dir = self.base_dir / f"session_{timestamp}"
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self.lidar_file = self.session_dir / "lidar_scans.jsonl"
        self.events_file = self.session_dir / "events.jsonl"
        self.odom_file = self.session_dir / "odometry.jsonl"

        self.start_time = time.time()
        self.lidar_count = 0
        self.event_count = 0

        # Write initial metadata
        metadata = {
            "session_start": timestamp,
            "start_timestamp": self.start_time,
        }
        (self.session_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))

        print(f"Logging to: {self.session_dir}")
        return self.session_dir

    def log_lidar(self, sectors: List[SectorData], throttle_hz: float = 1.0) -> None:
        """Log lidar sector data (throttled)."""
        if not self.lidar_file:
            return

        now = time.time()
        if now - self.last_lidar_log < (1.0 / throttle_hz):
            return
        self.last_lidar_log = now

        entry = {
            "ts": now,
            "sectors": [
                {"name": s.name, "min": round(s.min_distance, 3) if s.min_distance < 100 else None}
                for s in sectors
            ]
        }
        with open(self.lidar_file, "a") as f:
            f.write(json.dumps(entry) + "\n")
        self.lidar_count += 1

    def log_event(self, event_type: str, details: Optional[Dict] = None) -> None:
        """Log exploration event."""
        if not self.events_file:
            return
        entry = {"ts": time.time(), "type": event_type, "details": details or {}}
        with open(self.events_file, "a") as f:
            f.write(json.dumps(entry) + "\n")
        self.event_count += 1

    def log_motion(self, vx: float, vy: float, wz: float) -> None:
        """Log motion command."""
        if not self.odom_file:
            return
        entry = {"ts": time.time(), "vx": round(vx, 3), "vy": round(vy, 3), "wz": round(wz, 3)}
        with open(self.odom_file, "a") as f:
            f.write(json.dumps(entry) + "\n")

    def end_session(self, reason: str) -> Dict:
        """End session and write final metadata."""
        if not self.session_dir:
            return {}

        end_time = time.time()
        duration = end_time - self.start_time

        metadata = {
            "session_start": datetime.fromtimestamp(self.start_time).strftime("%Y-%m-%d_%H-%M-%S"),
            "session_end": datetime.now().strftime("%Y-%m-%d_%H-%M-%S"),
            "duration_seconds": round(duration, 1),
            "stats": {
                "lidar_samples": self.lidar_count,
                "events": self.event_count,
            },
            "stop_reason": reason,
        }
        (self.session_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))

        print(f"Session ended: {self.session_dir}")
        print(f"  Duration: {duration/60:.1f} min")
        print(f"  Lidar samples: {self.lidar_count}")
        print(f"  Events: {self.event_count}")
        return metadata


class RobotController:
    """Controls robot via ROS2 commands."""

    def __init__(self):
        self.sdk_path = Path.home() / "ros_robot_controller"

    def _run_docker(self, cmd: str, timeout: float = 5.0) -> tuple:
        """Run command in Docker container."""
        full_cmd = f'{DOCKER_EXEC} "{ROS_SOURCE} {cmd}"'
        try:
            result = subprocess.run(
                full_cmd, shell=True, capture_output=True, text=True, timeout=timeout
            )
            return result.returncode == 0, result.stdout.strip()
        except subprocess.TimeoutExpired:
            return False, "timeout"
        except Exception as e:
            return False, str(e)

    def _run_sdk(self, script: str, timeout: float = 5.0) -> tuple:
        """Run SDK script directly."""
        full_script = f"""
import sys
sys.path.insert(0, '{self.sdk_path}')
{script}
"""
        try:
            result = subprocess.run(
                ["python3", "-c", full_script],
                capture_output=True, text=True, timeout=timeout
            )
            return result.returncode == 0, result.stdout.strip()
        except Exception as e:
            return False, str(e)

    def move(self, vx: float, vy: float, wz: float) -> bool:
        """Send velocity command."""
        twist = f"'{{linear: {{x: {vx}, y: {vy}, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {wz}}}}}'"
        cmd = f"ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist {twist}"
        ok, _ = self._run_docker(cmd)
        return ok

    def stop(self) -> None:
        """Stop all motors."""
        self.move(0, 0, 0)

    def get_battery(self) -> Optional[float]:
        """Get battery voltage in V."""
        script = """
from ros_robot_controller_sdk import Board
import time
board = Board()
board.enable_reception()
time.sleep(0.1)
for _ in range(5):
    v = board.get_battery()
    if v is not None:
        print(v / 1000.0 if v > 100 else v)
        break
    time.sleep(0.05)
"""
        ok, output = self._run_sdk(script)
        if ok and output:
            try:
                return float(output)
            except:
                pass
        return None

    def read_lidar(self) -> tuple:
        """Read lidar scan. Returns (ranges, angle_min, angle_increment)."""
        # Use ros2 topic echo directly - much faster than Python script
        try:
            result = subprocess.run(
                'docker exec landerpi-ros2 bash -c "'
                'source /opt/ros/humble/setup.bash && '
                'source /ros2_ws/install/setup.bash 2>/dev/null; '
                'timeout 1 ros2 topic echo --once /scan --no-arr 2>/dev/null | head -20"',
                shell=True, capture_output=True, text=True, timeout=3
            )
            if result.returncode == 0 and result.stdout.strip():
                # Parse YAML output for angle_min and angle_increment
                angle_min = 0.0
                angle_inc = 0.0
                for line in result.stdout.split('\n'):
                    if 'angle_min:' in line:
                        angle_min = float(line.split(':')[1].strip())
                    elif 'angle_increment:' in line:
                        angle_inc = float(line.split(':')[1].strip())

                # Get ranges separately (they're large)
                result2 = subprocess.run(
                    'docker exec landerpi-ros2 bash -c "'
                    'source /opt/ros/humble/setup.bash && '
                    'source /ros2_ws/install/setup.bash 2>/dev/null; '
                    'timeout 1 ros2 topic echo --once /scan --field ranges 2>/dev/null"',
                    shell=True, capture_output=True, text=True, timeout=3
                )
                if result2.returncode == 0 and result2.stdout.strip():
                    # Parse ranges array
                    ranges = []
                    for line in result2.stdout.split('\n'):
                        line = line.strip()
                        if line.startswith('- '):
                            try:
                                val = float(line[2:])
                                ranges.append(val)
                            except ValueError:
                                pass
                    if ranges:
                        return ranges, angle_min, angle_inc
        except Exception:
            pass
        return [], 0.0, 0.0


class FrontierPlanner:
    """Simple frontier-based navigation."""

    SECTOR_NAMES = ["Front", "Front-Left", "Left", "Back-Left",
                    "Back", "Back-Right", "Right", "Front-Right"]
    SECTOR_ANGLES = [0, 45, 90, 135, 180, -135, -90, -45]  # degrees

    def __init__(self):
        self.sectors: List[SectorData] = []
        self._init_sectors()

    def _init_sectors(self):
        """Initialize 8 directional sectors."""
        self.sectors = [
            SectorData(i, name, angle)
            for i, (name, angle) in enumerate(zip(self.SECTOR_NAMES, self.SECTOR_ANGLES))
        ]

    def update(self, ranges: List[float], angle_min: float, angle_inc: float) -> None:
        """Update sectors from lidar scan."""
        # Reset sectors
        for s in self.sectors:
            s.min_distance = float('inf')
            s.max_distance = 0.0
            s.point_count = 0

        if not ranges:
            return

        for i, r in enumerate(ranges):
            if r <= 0.05 or r > 12.0 or math.isinf(r) or math.isnan(r):
                continue

            # Calculate angle in degrees (-180 to 180)
            angle_rad = angle_min + i * angle_inc
            angle_deg = math.degrees(angle_rad)

            # Normalize to -180 to 180
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360

            # Find sector (each is 45 degrees wide)
            sector_idx = self._angle_to_sector(angle_deg)
            s = self.sectors[sector_idx]
            s.min_distance = min(s.min_distance, r)
            s.max_distance = max(s.max_distance, r)
            s.point_count += 1

    def _angle_to_sector(self, angle_deg: float) -> int:
        """Convert angle to sector index."""
        # Sectors are centered at: 0, 45, 90, 135, 180, -135, -90, -45
        # Each sector spans 45 degrees
        normalized = (angle_deg + 22.5) % 360  # Shift by half sector width
        return int(normalized / 45) % 8

    def get_best_direction(self, min_dist: float = 0.40) -> tuple:
        """Get best direction to move. Returns (sector, reason)."""
        # Priority: Front > Front-Left/Right > Left/Right > Back
        priority_order = [0, 7, 1, 6, 2, 5, 3, 4]  # Indices

        for idx in priority_order:
            s = self.sectors[idx]
            if s.min_distance > min_dist:
                return s, f"clear at {s.min_distance:.2f}m"

        # All blocked - find most open
        best = max(self.sectors, key=lambda s: s.min_distance)
        if best.min_distance > 0.20:  # Even 20cm clearance is usable
            return best, f"best option at {best.min_distance:.2f}m"

        return None, "all blocked"

    def get_front_distance(self) -> float:
        """Get minimum distance in front sector."""
        return self.sectors[0].min_distance


class Explorer:
    """Main exploration controller."""

    def __init__(self, config: ExplorationConfig):
        self.config = config
        self.robot = RobotController()
        self.planner = FrontierPlanner()
        self.logger = DataLogger()
        self.state = ExplorationState()

        # Signal handling
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, sig, frame):
        """Handle shutdown signals."""
        print("\nShutdown signal received")
        self.stop("Signal received")
        sys.exit(0)

    def start(self, duration_minutes: float) -> None:
        """Start exploration."""
        self.state.running = True
        self.state.start_time = time.time()
        self.state.max_duration = duration_minutes * 60
        self.state.last_movement_time = time.time()
        self.state.action = "starting"

        # Check battery
        voltage = self.robot.get_battery()
        if voltage:
            self.state.battery_voltage = voltage
            print(f"Battery: {voltage:.2f}V")
            if voltage < self.config.battery_cutoff:
                print(f"Battery too low! ({voltage:.2f}V < {self.config.battery_cutoff}V)")
                self.state.running = False
                return

        self.logger.start_session()
        self.logger.log_event("start", {"duration_min": duration_minutes, "battery": voltage})

        print(f"Starting exploration for {duration_minutes} min")
        self._run_loop()

    def stop(self, reason: str = "Manual stop") -> None:
        """Stop exploration."""
        self.state.running = False
        self.state.action = "stopped"
        self.robot.stop()
        self.logger.log_event("stop", {"reason": reason})
        self.logger.end_session(reason)
        print(f"Stopped: {reason}")

    def _run_loop(self) -> None:
        """Main control loop."""
        loop_period = 1.0 / self.config.loop_rate_hz
        last_status = time.time()

        while self.state.running:
            loop_start = time.time()

            # Check runtime limit
            elapsed = time.time() - self.state.start_time
            if elapsed >= self.state.max_duration:
                self.stop(f"Runtime limit ({self.state.max_duration/60:.1f} min)")
                break

            # Check battery periodically
            if int(elapsed) % 60 == 0 and elapsed > 1:
                voltage = self.robot.get_battery()
                if voltage:
                    self.state.battery_voltage = voltage
                    if voltage < self.config.battery_cutoff:
                        self.stop(f"Low battery ({voltage:.2f}V)")
                        break
                    elif voltage < self.config.battery_warning:
                        print(f"Battery warning: {voltage:.2f}V")

            # Read lidar
            ranges, angle_min, angle_inc = self.robot.read_lidar()
            if ranges:
                self.planner.update(ranges, angle_min, angle_inc)
                self.logger.log_lidar(self.planner.sectors)
                self.state.lidar_samples += 1

            # Navigation decision
            self._navigate()

            # Check for stuck
            if self._is_stuck():
                self._escape_stuck()

            # Status update every 30s
            if time.time() - last_status >= 30:
                last_status = time.time()
                remaining = (self.state.max_duration - elapsed) / 60
                print(f"Status: {remaining:.1f} min remaining, action: {self.state.action}")

            # Rate limiting
            loop_elapsed = time.time() - loop_start
            if loop_elapsed < loop_period:
                time.sleep(loop_period - loop_elapsed)

    def _navigate(self) -> None:
        """Make navigation decision with turn commitment to prevent oscillation."""
        front_dist = self.planner.get_front_distance()

        # Emergency stop - always takes priority
        if front_dist < self.config.obstacle_stop_dist:
            self.robot.stop()
            self.state.action = "obstacle_stop"
            self.state.turn_direction = 0  # Cancel any turn
            self.logger.log_event("obstacle_stop", {"distance": front_dist})
            return

        # Check if we're committed to a turn
        if self.state.turn_direction != 0:
            turn_elapsed = time.time() - self.state.turn_start_time

            # Check if minimum turn duration has passed
            if turn_elapsed < self.config.min_turn_duration:
                # Keep turning in committed direction
                wz = self.config.turn_speed * self.state.turn_direction
                self.robot.move(0, 0, wz)
                return

            # Minimum time passed - check if turn is complete
            best_sector, _ = self.planner.get_best_direction(self.config.obstacle_slow_dist)
            if best_sector and abs(best_sector.center_angle) < self.config.turn_complete_threshold:
                # Turn complete - can move forward now
                self.state.turn_direction = 0
                self.state.turn_target_sector = ""
                print(f"Turn complete, front clear at {front_dist:.2f}m")
            else:
                # Keep turning - not yet facing clear direction
                wz = self.config.turn_speed * self.state.turn_direction
                self.robot.move(0, 0, wz)
                self.state.action = f"turning to {self.state.turn_target_sector}"
                return

        # Not turning - evaluate direction
        best_sector, reason = self.planner.get_best_direction(self.config.sector_blocked_dist)

        if best_sector is None:
            # All blocked - stop
            self.robot.stop()
            self.state.action = "blocked"
            return

        target_angle = best_sector.center_angle
        speed = self.config.forward_speed

        # Slow down if obstacle nearby
        if front_dist < self.config.obstacle_slow_dist:
            speed *= 0.5

        # Need to turn?
        if abs(target_angle) > self.config.turn_complete_threshold:
            # Start committed turn
            self.state.turn_direction = 1 if target_angle > 0 else -1
            self.state.turn_start_time = time.time()
            self.state.turn_target_sector = best_sector.name

            wz = self.config.turn_speed * self.state.turn_direction
            self.robot.move(0, 0, wz)
            self.state.action = f"turning to {best_sector.name}"
            self.logger.log_motion(0, 0, wz)
            print(f"Starting turn to {best_sector.name} ({target_angle:.0f}°)")
        else:
            # Move forward
            self.robot.move(speed, 0, 0)
            self.state.action = "forward"
            self.state.last_movement_time = time.time()
            self.logger.log_motion(speed, 0, 0)

    def _is_stuck(self) -> bool:
        """Check if robot is stuck."""
        return (time.time() - self.state.last_movement_time) > self.config.stuck_timeout

    def _escape_stuck(self) -> None:
        """Escape from stuck situation with committed maneuver."""
        print("Stuck detected - escaping")
        self.logger.log_event("stuck_escape")

        # Back up for escape duration
        print("  Backing up...")
        self.robot.move(-self.config.forward_speed * 0.5, 0, 0)
        time.sleep(self.config.escape_duration)
        self.robot.stop()

        # Re-read lidar after backing up
        ranges, angle_min, angle_inc = self.robot.read_lidar()
        if ranges:
            self.planner.update(ranges, angle_min, angle_inc)

        # Find most open direction and commit to full turn
        best = max(self.planner.sectors, key=lambda s: s.min_distance)
        if best.min_distance > 0.3 and abs(best.center_angle) > 15:
            # Calculate turn time needed (angle / angular_velocity)
            turn_angle_rad = abs(math.radians(best.center_angle))
            turn_time = turn_angle_rad / self.config.turn_speed
            turn_time = max(turn_time, 0.5)  # At least 0.5s
            turn_time = min(turn_time, 3.0)  # At most 3s

            direction = 1 if best.center_angle > 0 else -1
            wz = self.config.turn_speed * direction

            print(f"  Turning {best.center_angle:.0f}° to {best.name} ({turn_time:.1f}s)")
            self.robot.move(0, 0, wz)
            time.sleep(turn_time)
            self.robot.stop()
        else:
            # No clear direction - try random turn
            print("  No clear direction, turning 90°")
            self.robot.move(0, 0, self.config.turn_speed)
            time.sleep(1.0)  # ~90 degrees at 1.5 rad/s
            self.robot.stop()

        self.state.last_movement_time = time.time()


def main():
    parser = argparse.ArgumentParser(description="LanderPi Autonomous Exploration")
    subparsers = parser.add_subparsers(dest="command", help="Command")

    # Explore command
    explore_parser = subparsers.add_parser("explore", help="Start exploration")
    explore_parser.add_argument("--duration", type=float, default=5.0, help="Duration in minutes")
    explore_parser.add_argument("--speed", type=float, default=0.35, help="Forward speed (m/s)")

    # Stop command
    subparsers.add_parser("stop", help="Stop motors")

    # Status command
    subparsers.add_parser("status", help="Check robot status")

    args = parser.parse_args()

    if args.command == "explore":
        config = ExplorationConfig(forward_speed=args.speed)
        explorer = Explorer(config)
        explorer.start(args.duration)

    elif args.command == "stop":
        robot = RobotController()
        robot.stop()
        print("Motors stopped")

    elif args.command == "status":
        robot = RobotController()
        voltage = robot.get_battery()
        if voltage:
            print(f"Battery: {voltage:.2f}V")
        else:
            print("Battery: unknown")

        # Check ROS2 stack
        result = subprocess.run(
            "docker ps --filter name=landerpi-ros2 -q",
            shell=True, capture_output=True, text=True
        )
        if result.stdout.strip():
            print("ROS2 stack: running")
        else:
            print("ROS2 stack: not running")

    else:
        parser.print_help()


if __name__ == "__main__":
    main()
