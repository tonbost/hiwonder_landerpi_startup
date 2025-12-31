#!/usr/bin/env python3
"""
ROS2 node for monitoring battery voltage.

Publishes to: /battery (std_msgs/Float32) - voltage in volts

Uses the ros_robot_controller SDK to read battery voltage from the STM32
controller. This is the only node that should access the battery via SDK
to avoid serial port conflicts.
"""

import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Add SDK path - the SDK is uploaded to ~/ros_robot_controller on the robot
# Inside Docker, we need to import from the mounted volume
SDK_PATHS = [
    "/ros2_ws/src/ros_robot_controller",  # Docker mount path
    str(Path.home() / "ros_robot_controller"),  # Direct path
]

# Try to import the SDK
Board = None
for sdk_path in SDK_PATHS:
    if Path(sdk_path).exists():
        sys.path.insert(0, sdk_path)
        try:
            from ros_robot_controller_sdk import Board as _Board
            Board = _Board
            break
        except ImportError:
            pass


class BatteryMonitor(Node):
    """ROS2 node for battery voltage monitoring."""

    def __init__(self):
        super().__init__('battery_monitor')

        # Declare parameters
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('low_voltage_warning', 7.0)  # V
        self.declare_parameter('critical_voltage', 6.6)  # V

        self.publish_rate = self.get_parameter('publish_rate').value
        self.low_voltage = self.get_parameter('low_voltage_warning').value
        self.critical_voltage = self.get_parameter('critical_voltage').value

        # Publisher
        self.battery_pub = self.create_publisher(Float32, '/battery', 10)

        # Initialize SDK board connection
        self.board = None
        self._init_board()

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_battery)

        # State tracking
        self.last_voltage: float = 0.0
        self.warning_logged: bool = False

        self.get_logger().info(f'Battery monitor started (rate: {self.publish_rate} Hz)')

    def _init_board(self):
        """Initialize the SDK board connection."""
        if Board is None:
            self.get_logger().error(
                'ros_robot_controller SDK not found. Battery monitoring unavailable.'
            )
            return

        try:
            self.board = Board()
            self.board.enable_reception()
            time.sleep(0.1)  # Give board time to initialize
            self.get_logger().info('SDK board connection established')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize board: {e}')
            self.board = None

    def publish_battery(self):
        """Read and publish battery voltage."""
        voltage = self._read_battery()

        if voltage is not None:
            self.last_voltage = voltage

            # Publish
            msg = Float32()
            msg.data = voltage
            self.battery_pub.publish(msg)

            # Log warnings
            if voltage <= self.critical_voltage:
                self.get_logger().error(f'CRITICAL: Battery at {voltage:.2f}V!')
            elif voltage <= self.low_voltage and not self.warning_logged:
                self.get_logger().warn(f'Low battery: {voltage:.2f}V')
                self.warning_logged = True
            elif voltage > self.low_voltage:
                self.warning_logged = False

    def _read_battery(self) -> float | None:
        """Read battery voltage from SDK.

        Returns:
            Voltage in volts, or None if read failed.
        """
        if self.board is None:
            return None

        try:
            # Try multiple times (SDK can be flaky)
            for _ in range(3):
                voltage = self.board.get_battery()
                if voltage is not None:
                    # SDK returns mV if > 100, otherwise V
                    if voltage > 100:
                        return voltage / 1000.0
                    return float(voltage)
                time.sleep(0.05)
        except Exception as e:
            self.get_logger().debug(f'Battery read error: {e}')

        return None


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
