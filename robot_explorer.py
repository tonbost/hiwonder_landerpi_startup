#!/usr/bin/env python3
"""
Autonomous exploration for LanderPi robot.
Runs directly ON the robot - not via SSH from host.

This is a thin CLI that uses the exploration module from validation/exploration/.
The module provides all the logic; this script just wires it up with ROS2Hardware.

Usage (on robot):
    python3 ~/robot_explorer.py explore --duration 5
    python3 ~/robot_explorer.py explore --duration 30 --rosbag
    python3 ~/robot_explorer.py stop
    python3 ~/robot_explorer.py status
"""

import argparse
import signal
import sys
import time
from pathlib import Path

# Add exploration module to path (deployed to ~/landerpi/exploration/)
EXPLORATION_PATH = Path.home() / "landerpi" / "exploration"
if EXPLORATION_PATH.exists():
    sys.path.insert(0, str(EXPLORATION_PATH.parent))

# Import exploration module
try:
    from exploration import (
        ROS2Hardware,
        ROS2Config,
        ExplorationController,
        ExplorationConfig,
        SafetyConfig,
        SensorConfig,
        DataLogger,
        LogConfig,
        ArmScanner,
        ArmScannerConfig,
        EscapeConfig,
    )
    EXPLORATION_MODULE_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Exploration module not available: {e}")
    print("Make sure the exploration module is deployed to ~/landerpi/exploration/")
    EXPLORATION_MODULE_AVAILABLE = False


# Log directory
LOG_DIR = Path.home() / "landerpi" / "exploration_logs"


class RobotExplorer:
    """Thin wrapper that wires up exploration module with ROS2Hardware."""

    def __init__(
        self,
        duration_minutes: float = 5.0,
        forward_speed: float = 0.35,
        turn_time_multiplier: float = 2.5,
        enable_rosbag: bool = False,
    ):
        self.duration = duration_minutes
        self.enable_rosbag = enable_rosbag

        # Create ROS2 hardware interface
        self.hardware = ROS2Hardware()

        # Create configs
        exploration_config = ExplorationConfig(
            forward_speed=forward_speed,
            turn_time_multiplier=turn_time_multiplier,
        )

        safety_config = SafetyConfig(
            max_runtime_minutes=duration_minutes,
            battery_warning_voltage=7.0,
            battery_cutoff_voltage=6.6,
        )

        sensor_config = SensorConfig()

        log_config = LogConfig(
            base_dir=str(LOG_DIR),
            rosbag_enabled=enable_rosbag,
        )

        escape_config = EscapeConfig(
            turn_time_multiplier=turn_time_multiplier,
        )

        arm_scanner_config = ArmScannerConfig()

        # Create arm scanner with hardware callbacks
        self.arm_scanner = ArmScanner(
            arm_move_func=self.hardware.set_servo,
            get_depth_func=self.hardware.read_depth,
            config=arm_scanner_config,
            arm_pose_func=self.hardware.set_arm_pose,
        )

        # Create logger
        self.logger = DataLogger(log_config)

        # Create exploration controller with hardware callbacks
        self.controller = ExplorationController(
            move_func=self.hardware.move,
            stop_func=self.hardware.stop,
            get_battery_func=self.hardware.get_battery,
            exploration_config=exploration_config,
            safety_config=safety_config,
            sensor_config=sensor_config,
            logger=self.logger,
            arm_scanner=self.arm_scanner,
            escape_config=escape_config,
            on_speak=self._speak,
            # Continuous motion callbacks (watchdog-safe)
            move_for_duration_func=self.hardware.move_for_duration,
            start_move_func=self.hardware.start_move,
            stop_move_func=self.hardware.stop_move,
        )

        # Signal handling
        self._setup_signals()

    def _setup_signals(self):
        """Setup signal handlers for clean shutdown."""
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, sig, frame):
        """Handle shutdown signals."""
        print("\nShutdown signal received")
        self.stop("Signal received")
        sys.exit(0)

    def _speak(self, message: str):
        """Output a message (could be TTS in future)."""
        print(f"[TARS] {message}")

    def start(self):
        """Start exploration."""
        print(f"Starting exploration for {self.duration} minutes")
        print(f"  Forward speed: {self.controller.config.forward_speed} m/s")
        print(f"  Turn multiplier: {self.controller.config.turn_time_multiplier}")
        if self.enable_rosbag:
            print("  ROS2 bag recording: enabled")

        # Check ROS2 stack
        if not self.hardware.check_ros2_stack():
            print("ERROR: ROS2 stack not running. Deploy with:")
            print("  uv run python deploy_ros2_stack.py deploy")
            return

        # Initialize arm to horizontal position for camera
        self.hardware.init_arm_horizontal()

        # Start controller
        self.controller.start()

        # Main loop
        self._run_loop()

    def _run_loop(self):
        """Main control loop."""
        loop_rate = self.controller.config.loop_rate_hz
        loop_period = 1.0 / loop_rate
        last_status = time.time()

        while self.controller.running:
            loop_start = time.time()

            # Read sensors
            ranges, angle_min, angle_inc = self.hardware.read_lidar()
            depth_data = self.hardware.read_depth()

            # Update controller with sensor data
            if ranges:
                # Update lidar data
                self.controller.update_sensors(
                    ranges, angle_min, angle_inc,
                    None, 0, 0  # Raw depth not used
                )

                # Update depth stats separately if available
                if depth_data and "min_depth" in depth_data:
                    self.controller.sensors.update_depth_stats(depth_data["min_depth"])

            # Execute control step
            if not self.controller.step():
                break

            # Status update every 30s
            if time.time() - last_status >= 30:
                last_status = time.time()
                status = self.controller.get_status()
                remaining = status["safety"]["remaining_minutes"]
                action = status["action"]
                escape_level = status["escape"]["level"]
                print(f"Status: {remaining:.1f} min remaining | action: {action} | escape: {escape_level}")

            # Rate limiting
            elapsed = time.time() - loop_start
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)

    def stop(self, reason: str = "Manual stop"):
        """Stop exploration."""
        self.controller.stop_exploration(reason)
        print(f"Stopped: {reason}")


def cmd_explore(args):
    """Handle explore command."""
    if not EXPLORATION_MODULE_AVAILABLE:
        print("ERROR: Exploration module not available")
        sys.exit(1)

    explorer = RobotExplorer(
        duration_minutes=args.duration,
        forward_speed=args.speed,
        turn_time_multiplier=args.turn_multiplier,
        enable_rosbag=args.rosbag,
    )
    explorer.start()


def cmd_stop(args):
    """Handle stop command."""
    # Send stop command via ROS2
    if EXPLORATION_MODULE_AVAILABLE:
        hardware = ROS2Hardware()
        hardware.stop()
        print("Stop command sent")
    else:
        # Fallback: direct docker exec
        import subprocess
        subprocess.run(
            'docker exec landerpi-ros2 bash -c "'
            'source /opt/ros/humble/setup.bash && '
            'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '
            '\"{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}\""',
            shell=True
        )
        print("Stop command sent (fallback)")


def cmd_status(args):
    """Handle status command."""
    import subprocess

    # Check ROS2 stack
    result = subprocess.run(
        "docker ps --filter name=landerpi-ros2 -q",
        shell=True, capture_output=True, text=True
    )
    ros2_running = bool(result.stdout.strip())
    print(f"ROS2 stack: {'running' if ros2_running else 'not running'}")

    # Get battery
    if EXPLORATION_MODULE_AVAILABLE:
        hardware = ROS2Hardware()
        voltage = hardware.get_battery()
        if voltage:
            print(f"Battery: {voltage:.2f}V")
        else:
            print("Battery: unknown")
    else:
        print("Battery: (exploration module not available)")


def main():
    parser = argparse.ArgumentParser(description="LanderPi Autonomous Exploration")
    subparsers = parser.add_subparsers(dest="command", help="Command")

    # Explore command
    explore_parser = subparsers.add_parser("explore", help="Start exploration")
    explore_parser.add_argument("--duration", type=float, default=5.0, help="Duration in minutes")
    explore_parser.add_argument("--speed", type=float, default=0.35, help="Forward speed (m/s)")
    explore_parser.add_argument("--turn-multiplier", type=float, default=2.5,
                                help="Turn time multiplier for carpet friction")
    explore_parser.add_argument("--rosbag", action="store_true", help="Enable ROS2 bag recording")
    explore_parser.set_defaults(func=cmd_explore)

    # Stop command
    stop_parser = subparsers.add_parser("stop", help="Stop motors")
    stop_parser.set_defaults(func=cmd_stop)

    # Status command
    status_parser = subparsers.add_parser("status", help="Check robot status")
    status_parser.set_defaults(func=cmd_status)

    args = parser.parse_args()

    if hasattr(args, 'func'):
        args.func(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
