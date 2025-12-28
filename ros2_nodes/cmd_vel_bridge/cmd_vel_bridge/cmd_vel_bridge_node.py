#!/usr/bin/env python3
"""
ROS2 node that bridges /cmd_vel (Twist) to motor commands.

Subscribes to: /cmd_vel (geometry_msgs/Twist)
Publishes to: /ros_robot_controller/set_motor (ros_robot_controller_msgs/MotorsState)
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import MotorsState, MotorState

from .mecanum_kinematics import twist_to_wheel_speeds, RobotGeometry, WheelSpeeds


class CmdVelBridge(Node):
    """Bridge node from cmd_vel to motor commands."""

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # Declare parameters with defaults
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.15)
        self.declare_parameter('track_width', 0.15)
        self.declare_parameter('max_wheel_rps', 3.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('motor_ids.front_left', 1)
        self.declare_parameter('motor_ids.back_left', 2)
        self.declare_parameter('motor_ids.front_right', 3)
        self.declare_parameter('motor_ids.back_right', 4)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('motor_topic', '/ros_robot_controller/set_motor')

        # Get parameters
        self.geometry = RobotGeometry(
            wheel_radius=self.get_parameter('wheel_radius').value,
            wheel_base=self.get_parameter('wheel_base').value,
            track_width=self.get_parameter('track_width').value,
        )
        self.max_rps = self.get_parameter('max_wheel_rps').value
        self.timeout = self.get_parameter('cmd_vel_timeout').value

        self.motor_ids = {
            'front_left': self.get_parameter('motor_ids.front_left').value,
            'back_left': self.get_parameter('motor_ids.back_left').value,
            'front_right': self.get_parameter('motor_ids.front_right').value,
            'back_right': self.get_parameter('motor_ids.back_right').value,
        }

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        motor_topic = self.get_parameter('motor_topic').value

        # Last command timestamp for watchdog
        self.last_cmd_time = self.get_clock().now()

        # Publisher
        self.motor_pub = self.create_publisher(MotorsState, motor_topic, 10)

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Watchdog timer (10Hz check)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info(
            f'cmd_vel_bridge started: {cmd_vel_topic} -> {motor_topic}'
        )
        self.get_logger().info(
            f'Geometry: radius={self.geometry.wheel_radius}m, '
            f'base={self.geometry.wheel_base}m, track={self.geometry.track_width}m'
        )

    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming cmd_vel message."""
        self.last_cmd_time = self.get_clock().now()

        # Calculate wheel speeds
        speeds = twist_to_wheel_speeds(
            vx=msg.linear.x,
            vy=msg.linear.y,
            wz=msg.angular.z,
            geometry=self.geometry,
            max_rps=self.max_rps
        )

        # Publish motor commands
        self.publish_motor_speeds(speeds)

    def publish_motor_speeds(self, speeds: WheelSpeeds):
        """Publish wheel speeds to motor controller."""
        msg = MotorsState()
        msg.data = [
            self._motor_state(self.motor_ids['front_left'], speeds.front_left),
            self._motor_state(self.motor_ids['back_left'], speeds.back_left),
            self._motor_state(self.motor_ids['front_right'], speeds.front_right),
            self._motor_state(self.motor_ids['back_right'], speeds.back_right),
        ]
        self.motor_pub.publish(msg)

    def _motor_state(self, motor_id: int, rps: float) -> MotorState:
        """Create a MotorState message."""
        state = MotorState()
        state.id = motor_id
        state.rps = rps
        return state

    def watchdog_callback(self):
        """Stop motors if no cmd_vel received within timeout."""
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > self.timeout:
            # Send zero velocities
            self.publish_motor_speeds(WheelSpeeds(0.0, 0.0, 0.0, 0.0))

    def stop_motors(self):
        """Emergency stop - zero all motors."""
        self.publish_motor_speeds(WheelSpeeds(0.0, 0.0, 0.0, 0.0))
        self.get_logger().info('Motors stopped')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
