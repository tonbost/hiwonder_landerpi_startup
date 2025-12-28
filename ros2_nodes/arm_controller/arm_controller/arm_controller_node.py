#!/usr/bin/env python3
"""
ROS2 node for controlling the LanderPi 5-DOF arm.

Subscribes to: /arm/cmd (std_msgs/String) - JSON commands
Publishes to: /arm/state (std_msgs/String) - JSON state

Command format:
  {"action": "set_position", "duration": 2.0, "positions": [[1, 500], [2, 500], ...]}
  {"action": "home"}
  {"action": "stop"}
  {"action": "gripper", "position": "open"|"close"|<pulse>}
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# SDK path - mounted from host
sys.path.insert(0, '/ros2_ws/src/ros_robot_controller/ros_robot_controller')


class ArmController(Node):
    """ROS2 node for arm control via SDK."""

    # Servo configuration
    ARM_SERVO_IDS = [1, 2, 3, 4, 5]
    GRIPPER_SERVO_ID = 10
    ALL_SERVO_IDS = ARM_SERVO_IDS + [GRIPPER_SERVO_ID]
    HOME_POSITION = 500
    GRIPPER_OPEN = 200
    GRIPPER_CLOSED = 700

    def __init__(self):
        super().__init__('arm_controller')

        # Try to import SDK
        try:
            from ros_robot_controller_sdk import Board
            self.board = Board()
            self.board.enable_reception()
            self.sdk_available = True
            self.get_logger().info('SDK initialized successfully')
        except Exception as e:
            self.board = None
            self.sdk_available = False
            self.get_logger().warn(f'SDK not available: {e}')

        # Publisher for arm state
        self.state_pub = self.create_publisher(String, '/arm/state', 10)

        # Subscriber for arm commands
        self.cmd_sub = self.create_subscription(
            String,
            '/arm/cmd',
            self.cmd_callback,
            10
        )

        # State timer (publish at 1Hz)
        self.state_timer = self.create_timer(1.0, self.publish_state)

        self.get_logger().info('Arm controller started')

    def cmd_callback(self, msg: String):
        """Handle incoming arm commands."""
        try:
            cmd = json.loads(msg.data)
            action = cmd.get('action', '')

            if action == 'set_position':
                self.set_position(cmd.get('duration', 2.0), cmd.get('positions', []))
            elif action == 'home':
                self.go_home(cmd.get('duration', 2.0))
            elif action == 'stop':
                self.stop()
            elif action == 'gripper':
                self.control_gripper(cmd.get('position', 'open'))
            else:
                self.get_logger().warn(f'Unknown action: {action}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Command error: {e}')

    def set_position(self, duration: float, positions: list):
        """Set servo positions."""
        if not self.sdk_available:
            self.get_logger().warn('SDK not available')
            return

        self.get_logger().info(f'Setting positions: {positions} over {duration}s')
        self.board.bus_servo_set_position(duration, positions)

    def go_home(self, duration: float = 2.0):
        """Move all servos to home position."""
        if not self.sdk_available:
            return

        positions = [[sid, self.HOME_POSITION] for sid in self.ALL_SERVO_IDS]
        self.get_logger().info(f'Moving to home position over {duration}s')
        self.board.bus_servo_set_position(duration, positions)

    def stop(self):
        """Stop all servos."""
        if not self.sdk_available:
            return

        self.get_logger().info('Stopping all servos')
        self.board.bus_servo_stop(self.ALL_SERVO_IDS)

    def control_gripper(self, position):
        """Control gripper."""
        if not self.sdk_available:
            return

        if position == 'open':
            pulse = self.GRIPPER_OPEN
        elif position == 'close':
            pulse = self.GRIPPER_CLOSED
        else:
            pulse = int(position)

        self.get_logger().info(f'Gripper to {pulse}')
        self.board.bus_servo_set_position(0.5, [[self.GRIPPER_SERVO_ID, pulse]])

    def publish_state(self):
        """Publish current arm state."""
        state = {
            'sdk_available': self.sdk_available,
            'timestamp': time.time(),
        }

        if self.sdk_available:
            try:
                positions = {}
                for sid in self.ALL_SERVO_IDS:
                    pos = self.board.bus_servo_read_position(sid)
                    positions[sid] = pos[0] if pos else None
                state['positions'] = positions
            except Exception:
                pass

        msg = String()
        msg.data = json.dumps(state)
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
