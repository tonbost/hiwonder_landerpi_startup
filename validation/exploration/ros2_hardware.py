"""ROS2 hardware interface layer.

Provides all hardware access via ROS2 topics through docker exec.
This layer abstracts the docker exec calls so exploration code
doesn't need to know about Docker.
"""

import json
import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

# Docker exec prefix for ROS2 commands
DOCKER_EXEC = "docker exec landerpi-ros2 bash -c"
ROS_SOURCE = "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash 2>/dev/null;"


@dataclass
class ROS2Config:
    """ROS2 hardware configuration."""
    # Topics
    cmd_vel_topic: str = "/cmd_vel"
    scan_topic: str = "/scan"
    depth_stats_topic: str = "/depth_stats"  # Processed depth stats (JSON)
    arm_cmd_topic: str = "/arm/cmd"
    arm_state_topic: str = "/arm/state"
    battery_topic: str = "/battery"
    hazards_topic: str = "/hazards"

    # Timeouts (seconds)
    cmd_timeout: float = 5.0
    lidar_timeout: float = 3.0
    depth_timeout: float = 3.0
    battery_timeout: float = 2.0

    # SDK path (for battery fallback)
    sdk_path: str = "~/ros_robot_controller"

    # Sensor bridge configuration (low-latency file-based reading)
    use_file_bridge: bool = True  # Use JSON files from sensor_bridge node
    data_dir: str = "~/landerpi_data"  # Directory with JSON files
    file_stale_threshold: float = 2.0  # Max age of data before fallback (seconds)


class ROS2Hardware:
    """Hardware interface via ROS2 topics.

    All methods use docker exec to run ROS2 commands inside
    the landerpi-ros2 container.
    """

    def __init__(self, config: Optional[ROS2Config] = None):
        self.config = config or ROS2Config()
        self._sdk_path = Path(self.config.sdk_path).expanduser()

        # Sensor bridge file paths (low-latency reading)
        self._data_dir = Path(self.config.data_dir).expanduser()
        self._lidar_file = self._data_dir / "lidar.json"
        self._hazards_file = self._data_dir / "hazards.json"
        self._depth_stats_file = self._data_dir / "depth_stats.json"
        self._battery_file = self._data_dir / "battery.json"

        # Sensor reading cache to avoid slowing down control loop
        # (still used for subprocess fallback)
        self._last_depth_time: float = 0.0
        self._cached_depth: Optional[dict] = None
        self._depth_read_interval: float = 1.0  # Only read depth every 1 second
        # Lidar cache
        self._last_lidar_time: float = 0.0
        self._cached_lidar: Tuple[List[float], float, float] = ([], 0.0, 0.0)
        self._lidar_read_interval: float = 0.5  # Read lidar every 0.5 seconds
        # Background motion process (for continuous movement)
        self._motion_process: Optional[subprocess.Popen] = None
        # Hazard cache (reading is slow, so cache results)
        self._last_hazard_time: float = 0.0
        self._cached_hazards: List[dict] = []
        self._hazard_read_interval: float = 0.5  # Read hazards every 0.5 seconds

    def _run_docker(self, cmd: str, timeout: Optional[float] = None) -> Tuple[bool, str]:
        """Run command in Docker container.

        Args:
            cmd: The ROS2 command to run (without sourcing)
            timeout: Command timeout in seconds

        Returns:
            Tuple of (success, output)
        """
        timeout = timeout or self.config.cmd_timeout
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

    def _run_sdk(self, script: str, timeout: float = 5.0) -> Tuple[bool, str]:
        """Run SDK script directly on the host.

        Args:
            script: Python script to execute
            timeout: Script timeout in seconds

        Returns:
            Tuple of (success, output)
        """
        full_script = f"""
import sys
sys.path.insert(0, '{self._sdk_path}')
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

    def _read_json_file(self, file_path: Path) -> Optional[dict]:
        """Read JSON file with freshness check.

        Uses sensor bridge files for low-latency reading (~10ms vs ~400ms).

        Args:
            file_path: Path to JSON file

        Returns:
            Dict with data and timestamp, or None if file doesn't exist,
            is stale, or can't be parsed.
        """
        if not self.config.use_file_bridge:
            return None

        if not file_path.exists():
            return None

        try:
            with open(file_path, 'r') as f:
                data = json.load(f)

            # Check freshness
            timestamp = data.get('timestamp', 0)
            age = time.time() - timestamp
            if age > self.config.file_stale_threshold:
                return None  # Data is too old, fall back to subprocess

            return data
        except (json.JSONDecodeError, IOError, KeyError):
            return None

    # --- Motion Control ---

    def move(self, vx: float, vy: float, wz: float) -> bool:
        """Send velocity command via /cmd_vel.

        Args:
            vx: Linear velocity X (m/s) - forward/backward
            vy: Linear velocity Y (m/s) - strafe left/right
            wz: Angular velocity Z (rad/s) - rotation

        Returns:
            True if command sent successfully
        """
        twist = f"'{{linear: {{x: {vx}, y: {vy}, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {wz}}}}}'"
        cmd = f"ros2 topic pub --once {self.config.cmd_vel_topic} geometry_msgs/msg/Twist {twist}"
        ok, _ = self._run_docker(cmd)
        return ok

    def stop(self) -> None:
        """Stop all motors."""
        self.stop_move()  # Stop any background motion first
        self.move(0, 0, 0)

    def move_for_duration(self, vx: float, vy: float, wz: float, duration: float) -> bool:
        """Send velocity command continuously for a specified duration.

        Uses ros2 topic pub --rate to continuously publish, keeping
        the watchdog happy (0.5s timeout in cmd_vel_bridge).

        This is a BLOCKING call - it will wait for the duration to complete.

        Args:
            vx: Linear velocity X (m/s) - forward/backward
            vy: Linear velocity Y (m/s) - strafe left/right
            wz: Angular velocity Z (rad/s) - rotation
            duration: How long to move in seconds

        Returns:
            True if command executed successfully
        """
        # Stop any existing background motion
        self.stop_move()

        twist = f"'{{linear: {{x: {vx}, y: {vy}, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {wz}}}}}'"
        # Use --rate 10 (10 Hz) to publish continuously, with timeout
        # Rate of 10 Hz means publish every 0.1s, well within 0.5s watchdog
        # Add small buffer to timeout to ensure full duration
        cmd = f"timeout {duration + 0.1} ros2 topic pub --rate 10 {self.config.cmd_vel_topic} geometry_msgs/msg/Twist {twist}"
        ok, _ = self._run_docker(cmd, timeout=duration + 2.0)
        # Always send stop after to ensure motors halt
        self.move(0, 0, 0)
        return ok

    def start_move(self, vx: float, vy: float, wz: float) -> bool:
        """Start continuous velocity command (non-blocking).

        Uses ros2 topic pub --rate in background. Call stop_move() to stop.
        This is for cases where you need to move while doing other work
        (e.g., sampling sensors during a 360 scan).

        Args:
            vx: Linear velocity X (m/s) - forward/backward
            vy: Linear velocity Y (m/s) - strafe left/right
            wz: Angular velocity Z (rad/s) - rotation

        Returns:
            True if background process started successfully
        """
        # Stop any existing background motion first
        self.stop_move()

        twist = f"'{{linear: {{x: {vx}, y: {vy}, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {wz}}}}}'"
        # Use --rate 10 to keep watchdog happy, no timeout (runs until killed)
        cmd = f"ros2 topic pub --rate 10 {self.config.cmd_vel_topic} geometry_msgs/msg/Twist {twist}"
        full_cmd = f'{DOCKER_EXEC} "{ROS_SOURCE} {cmd}"'

        try:
            self._motion_process = subprocess.Popen(
                full_cmd,
                shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return True
        except Exception:
            self._motion_process = None
            return False

    def stop_move(self) -> None:
        """Stop background continuous motion started by start_move().

        Safe to call even if no motion is running.
        """
        if self._motion_process is not None:
            try:
                self._motion_process.terminate()
                self._motion_process.wait(timeout=1.0)
            except Exception:
                try:
                    self._motion_process.kill()
                except Exception:
                    pass
            self._motion_process = None

    # --- Lidar ---

    def read_lidar(self) -> Tuple[List[float], float, float]:
        """Read lidar scan via /scan topic.

        Uses sensor bridge file for low-latency reading (~10ms).
        Falls back to subprocess if file not available (~400ms).

        Returns:
            Tuple of (ranges, angle_min, angle_increment)
            Empty tuple on failure: ([], 0.0, 0.0)
        """
        # Try file-based reading first (low-latency, ~10ms)
        data = self._read_json_file(self._lidar_file)
        if data:
            ranges = data.get('ranges', [])
            angle_min = data.get('angle_min', 0.0)
            angle_inc = data.get('angle_increment', 0.0)
            if ranges:
                self._cached_lidar = (ranges, angle_min, angle_inc)
                return self._cached_lidar

        # Fallback to subprocess (slower, ~400ms with caching)
        return self._read_lidar_subprocess()

    def _read_lidar_subprocess(self) -> Tuple[List[float], float, float]:
        """Read lidar via subprocess (fallback for when sensor bridge unavailable)."""
        now = time.time()

        # Return cached result if called too frequently
        if now - self._last_lidar_time < self._lidar_read_interval:
            return self._cached_lidar

        self._last_lidar_time = now

        try:
            # Get full scan in a single call (metadata + ranges)
            result = subprocess.run(
                'docker exec landerpi-ros2 bash -c "'
                'source /opt/ros/humble/setup.bash && '
                'source /ros2_ws/install/setup.bash 2>/dev/null; '
                'timeout 2 ros2 topic echo --once /scan 2>/dev/null"',
                shell=True, capture_output=True, text=True,
                timeout=self.config.lidar_timeout
            )

            if result.returncode != 0 or not result.stdout.strip():
                return self._cached_lidar  # Return last good data

            # Parse angle values and ranges from YAML output
            angle_min = 0.0
            angle_inc = 0.0
            ranges = []
            in_ranges = False

            for line in result.stdout.split('\n'):
                if 'angle_min:' in line:
                    angle_min = float(line.split(':')[1].strip())
                elif 'angle_increment:' in line:
                    angle_inc = float(line.split(':')[1].strip())
                elif 'ranges:' in line:
                    in_ranges = True
                elif in_ranges:
                    line = line.strip()
                    if line.startswith('- '):
                        try:
                            ranges.append(float(line[2:]))
                        except ValueError:
                            pass
                    elif line and not line.startswith('-'):
                        # End of ranges section
                        in_ranges = False

            if ranges:
                self._cached_lidar = (ranges, angle_min, angle_inc)

            return self._cached_lidar

        except Exception:
            return self._cached_lidar  # Return last good data on error

    def _parse_ranges(self, output: str) -> List[float]:
        """Parse range values from ros2 topic echo output."""
        ranges = []

        # Try array('f', [...]) format first (from --field output)
        if output.startswith("array('f',"):
            match = re.search(r"array\('f',\s*\[(.*)\]\)", output, re.DOTALL)
            if match:
                values_str = match.group(1)
                for val_str in values_str.split(','):
                    val_str = val_str.strip()
                    if val_str:
                        try:
                            ranges.append(float(val_str))
                        except ValueError:
                            pass
        else:
            # Fallback: YAML list format with '- ' prefix
            for line in output.split('\n'):
                line = line.strip()
                if line.startswith('- '):
                    try:
                        ranges.append(float(line[2:]))
                    except ValueError:
                        pass

        return ranges

    # --- Depth Camera ---

    def read_depth(self) -> Optional[dict]:
        """Read depth camera stats via /depth_stats topic.

        Uses sensor bridge file for low-latency reading (~10ms).
        Falls back to subprocess if file not available (~400ms).

        Returns:
            Dict with 'min_depth', 'avg_depth', 'valid_percent' keys (depths in meters),
            or None if read failed.
        """
        # Try file-based reading first (low-latency, ~10ms)
        data = self._read_json_file(self._depth_stats_file)
        if data:
            self._cached_depth = {
                "min_depth": data.get("min_depth_m", 0.0),
                "avg_depth": data.get("avg_depth_m", 0.0),
                "valid_percent": data.get("valid_percent", 0.0),
            }
            return self._cached_depth

        # Fallback to subprocess (slower, ~400ms with caching)
        return self._read_depth_subprocess()

    def _read_depth_subprocess(self) -> Optional[dict]:
        """Read depth stats via subprocess (fallback for when sensor bridge unavailable)."""
        now = time.time()

        # Return cached result if called too frequently
        if now - self._last_depth_time < self._depth_read_interval:
            return self._cached_depth

        self._last_depth_time = now

        # Read from /depth_stats topic (JSON format from depth_stats_node)
        # Note: --field data times out, and default output truncates long strings
        # Using --full-length to get complete JSON
        cmd = f"timeout 2 ros2 topic echo --once --full-length {self.config.depth_stats_topic} 2>/dev/null"
        ok, output = self._run_docker(cmd, timeout=3.0)

        if not ok or not output:
            return self._cached_depth  # Return last good data

        try:
            # Parse output format: "data: '{JSON}'\n---"
            # Extract JSON from the data: line
            for line in output.split('\n'):
                line = line.strip()
                if line.startswith("data: "):
                    # Remove "data: " prefix and surrounding quotes
                    json_str = line[6:].strip().strip("'").strip('"')
                    stats = json.loads(json_str)

                    # Convert to the format expected by exploration code
                    self._cached_depth = {
                        "min_depth": stats.get("min_depth_m", 0.0),
                        "avg_depth": stats.get("avg_depth_m", 0.0),
                        "valid_percent": stats.get("valid_percent", 0.0),
                    }
                    break

        except (json.JSONDecodeError, KeyError, TypeError) as e:
            # Keep last good data on parse error
            pass

        return self._cached_depth

    # --- Hazards ---

    def read_hazards(self) -> List[dict]:
        """Read semantic hazards from /hazards topic.

        Uses sensor bridge file for low-latency reading (~10ms).
        Falls back to subprocess if file not available (~400ms).

        Returns:
            List of hazard dicts: [{"type": "person", "distance": 1.0, ...}, ...]
        """
        # Try file-based reading first (low-latency, ~10ms)
        data = self._read_json_file(self._hazards_file)
        if data:
            self._cached_hazards = data.get("hazards", [])
            return self._cached_hazards

        # Fallback to subprocess (slower, ~400ms with caching)
        return self._read_hazards_subprocess()

    def _read_hazards_subprocess(self) -> List[dict]:
        """Read hazards via subprocess (fallback for when sensor bridge unavailable)."""
        now = time.time()
        if now - self._last_hazard_time < self._hazard_read_interval:
            return self._cached_hazards

        self._last_hazard_time = now

        # Read from /hazards topic with timeout long enough to get data
        full_cmd = (
            f'{DOCKER_EXEC} "{ROS_SOURCE} timeout 0.3 ros2 topic echo /hazards 2>/dev/null" '
            f'2>/dev/null | grep -m1 "data:" | sed \'s/data: //\' | tr -d "\'"'
        )
        try:
            result = subprocess.run(
                full_cmd, shell=True, capture_output=True, text=True, timeout=1.0
            )
            output = result.stdout.strip()
            if output and output.startswith("{"):
                # Try to parse full JSON first
                try:
                    data = json.loads(output)
                    self._cached_hazards = data.get("hazards", [])
                    return self._cached_hazards
                except json.JSONDecodeError:
                    # Output is truncated - extract individual hazard objects
                    hazards = []
                    for match in re.finditer(
                        r'\{"type":\s*"([^"]+)",\s*"distance":\s*([\d.]+),\s*"angle":\s*([-\d.]+),\s*"score":\s*([\d.]+),\s*"source":\s*"([^"]+)"\}',
                        output
                    ):
                        hazards.append({
                            "type": match.group(1),
                            "distance": float(match.group(2)),
                            "angle": float(match.group(3)),
                            "score": float(match.group(4)),
                            "source": match.group(5),
                        })
                    self._cached_hazards = hazards
                    return hazards
        except subprocess.TimeoutExpired:
            pass

        # Keep previous cached value if read failed
        return self._cached_hazards

    # --- Arm Control ---

    def set_arm_pose(self, positions: List[List[int]], duration: float) -> bool:
        """Set multiple arm servo positions.

        Args:
            positions: List of [servo_id, position] pairs
            duration: Movement duration in seconds

        Returns:
            True if command sent successfully
        """
        # Use SDK directly for arm control (more reliable than ROS2 topic)
        script = f"""
from ros_robot_controller_sdk import Board
import time
board = Board()
board.enable_reception()
time.sleep(0.1)
board.bus_servo_set_position({duration}, {positions})
print("OK")
"""
        ok, output = self._run_sdk(script, timeout=duration + 3.0)
        return ok and "OK" in output

    def set_servo(self, servo_id: int, position: int, duration: float) -> bool:
        """Set single arm servo position.

        Args:
            servo_id: Servo ID (1-5 for arm, 10 for gripper)
            position: Target position (0-1000)
            duration: Movement duration in seconds

        Returns:
            True if command sent successfully
        """
        return self.set_arm_pose([[servo_id, position]], duration)

    def init_arm_explore(self) -> bool:
        """Initialize arm to exploring home position (folded back, out of the way).

        Arm is tucked for safe navigation. Camera extends for scanning when needed.
        """
        # Exploring home: arm folded back, out of the way
        positions = [[1, 547], [2, 818], [3, 203], [4, 58], [5, 501]]
        duration = 2.0
        print("Initializing arm to explore home position...")
        ok = self.set_arm_pose(positions, duration)
        if ok:
            time.sleep(duration + 0.5)  # Wait for arm to reach position
            print("Arm ready (tucked)")
        else:
            print("Arm init failed")
        return ok

    # Keep old name as alias for compatibility
    def init_arm_horizontal(self) -> bool:
        """Alias for init_arm_explore (legacy name)."""
        return self.init_arm_explore()

    # --- Battery ---

    def get_battery(self) -> Optional[float]:
        """Get battery voltage in volts.

        Uses sensor bridge file for low-latency reading (~10ms).
        Falls back to ROS2 topic then SDK if file not available.

        Returns:
            Battery voltage in V, or None if read failed.
        """
        # Try file-based reading first (low-latency, ~10ms)
        data = self._read_json_file(self._battery_file)
        if data:
            voltage = data.get("voltage", 0.0)
            if voltage > 0:
                return voltage

        # Fallback to subprocess/SDK
        return self._get_battery_fallback()

    def _get_battery_fallback(self) -> Optional[float]:
        """Get battery via ROS2 topic or SDK (fallback methods)."""
        # Try ROS2 topic first (if battery_monitor node is running)
        cmd = f"timeout 1 ros2 topic echo --once {self.config.battery_topic} --field data 2>/dev/null"
        ok, output = self._run_docker(cmd, timeout=self.config.battery_timeout)

        if ok and output:
            try:
                voltage = float(output.strip())
                if voltage > 0:
                    return voltage
            except ValueError:
                pass

        # Fallback to SDK
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
            except ValueError:
                pass

        return None

    # --- Utility ---

    def check_ros2_stack(self) -> bool:
        """Check if ROS2 stack container is running."""
        result = subprocess.run(
            "docker ps --filter name=landerpi-ros2 -q",
            shell=True, capture_output=True, text=True
        )
        return bool(result.stdout.strip())

    def check_camera_health(self) -> Tuple[bool, float]:
        """Check if camera is publishing images.

        Returns:
            Tuple of (is_healthy, rate_hz)
            - is_healthy: True if camera is publishing at > 1 Hz
            - rate_hz: Measured publish rate (0 if not publishing)
        """
        # Quick check - try to get one message with short timeout
        # Use grep to wait for the rate line (head -1 terminates too early)
        cmd = "timeout 3 ros2 topic hz /aurora/rgb/image_raw --window 3 2>&1 | grep -m 1 'average rate'"
        ok, output = self._run_docker(cmd, timeout=5.0)

        if not ok or "does not appear" in output or not output:
            return False, 0.0

        # Parse rate from output like "average rate: 10.274"
        import re
        match = re.search(r'average rate:\s*([\d.]+)', output)
        if match:
            rate = float(match.group(1))
            return rate > 1.0, rate

        return False, 0.0

    def restart_camera(self) -> bool:
        """Restart the camera driver when it gets stuck.

        Kills the aurora930 process and restarts the ROS2 container.

        Returns:
            True if restart succeeded and camera is publishing again
        """
        # Kill the stuck camera process
        kill_cmd = "pkill -9 -f aurora930"
        subprocess.run(
            f"{DOCKER_EXEC} '{kill_cmd}'",
            shell=True, capture_output=True, timeout=5
        )

        # Restart the docker container (cleanest way to restart all nodes)
        restart_cmd = "cd ~/landerpi/docker && docker compose restart"
        result = subprocess.run(
            restart_cmd,
            shell=True, capture_output=True, timeout=30
        )

        if result.returncode != 0:
            return False

        # Wait for camera to come back up
        time.sleep(8)

        # Check if camera recovered
        healthy, rate = self.check_camera_health()
        return healthy

    def get_status(self) -> dict:
        """Get hardware interface status."""
        ros2_running = self.check_ros2_stack()
        battery = self.get_battery() if ros2_running else None

        return {
            "ros2_stack_running": ros2_running,
            "battery_voltage": battery,
            "config": {
                "cmd_vel_topic": self.config.cmd_vel_topic,
                "scan_topic": self.config.scan_topic,
                "depth_stats_topic": self.config.depth_stats_topic,
            }
        }
