#!/usr/bin/env python3
"""
ROS2 node for controlling the LanderPi 5-DOF arm.

Uses ros_robot_controller topics instead of direct SDK access to avoid
serial port conflicts.

Subscribes to: /arm/cmd (std_msgs/String) - JSON commands
Publishes to: /arm/state (std_msgs/String) - JSON state
Publishes to: /ros_robot_controller/bus_servo/set_state - Servo commands

Command format:
  {"action": "set_position", "duration": 2.0, "positions": [[1, 500], [2, 500], ...]}
  {"action": "home"}
  {"action": "stop"}
  {"action": "gripper", "position": "open"|"close"|<pulse>}
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Import vendor messages
from ros_robot_controller_msgs.msg import SetBusServoState, BusServoState


class ArmController(Node):
    """ROS2 node for arm control via ros_robot_controller topics."""

    # Servo configuration
    ARM_SERVO_IDS = [1, 2, 3, 4, 5]
    GRIPPER_SERVO_ID = 10
    ALL_SERVO_IDS = ARM_SERVO_IDS + [GRIPPER_SERVO_ID]
    # Home positions for each servo (user-defined resting position)
    HOME_POSITIONS = {
        1: 546,   # Base
        2: 790,   # Shoulder
        3: 0,     # Elbow (was -54, clamped to 0)
        4: 324,   # Wrist
        5: 501,   # Rotate
        10: 500,  # Gripper
    }
    GRIPPER_OPEN = 200
    GRIPPER_CLOSED = 700

    def __init__(self):
        super().__init__('arm_controller')

        # Publisher for servo commands (via ros_robot_controller)
        self.servo_pub = self.create_publisher(
            SetBusServoState,
            '/ros_robot_controller/bus_servo/set_state',
            10
        )

        # Publisher for arm state (high-level)
        self.state_pub = self.create_publisher(String, '/arm/state', 10)

        # Subscriber for arm commands (high-level)
        self.cmd_sub = self.create_subscription(
            String,
            '/arm/cmd',
            self.cmd_callback,
            10
        )

        # State timer (publish at 1Hz)
        self.state_timer = self.create_timer(1.0, self.publish_state)

        # Track last commanded positions
        self.last_positions = {sid: self.HOME_POSITIONS[sid] for sid in self.ALL_SERVO_IDS}

        self.get_logger().info('Arm controller started (using ros_robot_controller topics)')

    def cmd_callback(self, msg: String):
        """Handle incoming arm commands."""
        try:
            cmd = json.loads(msg.data)
            action = cmd.get('action', '')

            self.get_logger().info(f'Received command: {action}')

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
        """Set servo positions via ros_robot_controller topic.

        The vendor node expects a specific format:
        - Each servo needs its own BusServoState
        - present_id = [1, servo_id]  (1 = flag to enable)
        - position = [1, position_value]  (1 = flag to enable)
        """
        if not positions:
            return

        self.get_logger().info(f'Setting positions: {positions} over {duration}s')

        # Build SetBusServoState message
        msg = SetBusServoState()
        msg.duration = duration

        # Create BusServoState for EACH servo (vendor API requirement)
        states = []
        for servo_id, pos in positions:
            servo_state = BusServoState()
            # present_id[0] = 1 (enable flag), present_id[1] = servo ID
            servo_state.present_id = [1, int(servo_id)]
            # position[0] = 1 (enable flag), position[1] = position value
            # Clamp to valid range [0, 65535] - ROS2 msg requires unsigned int
            pos_clamped = max(0, min(65535, int(pos)))
            servo_state.position = [1, pos_clamped]
            states.append(servo_state)

        msg.state = states

        # Publish command
        self.servo_pub.publish(msg)

        # Update tracked positions
        for servo_id, pos in positions:
            self.last_positions[servo_id] = pos

    def go_home(self, duration: float = 2.0):
        """Move all servos to home position."""
        positions = [[sid, self.HOME_POSITIONS[sid]] for sid in self.ALL_SERVO_IDS]
        self.get_logger().info(f'Moving to home position over {duration}s')
        self.set_position(duration, positions)

    def stop(self):
        """Stop all servos."""
        self.get_logger().info('Stopping all servos')

        # Send stop command via BusServoState
        # Vendor format: present_id = [1, servo_id], stop = [1]
        msg = SetBusServoState()
        msg.duration = 0.0

        states = []
        for servo_id in self.ALL_SERVO_IDS:
            servo_state = BusServoState()
            servo_state.present_id = [1, int(servo_id)]
            servo_state.stop = [1]
            states.append(servo_state)

        msg.state = states
        self.servo_pub.publish(msg)

    def control_gripper(self, position):
        """Control gripper."""
        if position == 'open':
            pulse = self.GRIPPER_OPEN
        elif position == 'close':
            pulse = self.GRIPPER_CLOSED
        else:
            pulse = int(position)

        self.get_logger().info(f'Gripper to {pulse}')
        self.set_position(0.5, [[self.GRIPPER_SERVO_ID, pulse]])

    def publish_state(self):
        """Publish current arm state."""
        state = {
            'timestamp': time.time(),
            'last_positions': self.last_positions,
        }

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
