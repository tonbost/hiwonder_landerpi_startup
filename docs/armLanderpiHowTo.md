# HiWonder LanderPi Robotic Arm Control Guide

This guide covers controlling the 5-DOF robotic arm on the HiWonder LanderPi robot using ROS2 and direct SDK methods.

## Overview

The LanderPi features a 5-DOF (5 Degrees of Freedom) robotic arm with:
- **5 bus servos** (IDs 1-5) for arm joints
- **1 gripper servo** (ID 10) for end effector
- Serial communication via STM32 controller at `/dev/rrc` (1000000 baud)

## Arm Architecture

### Joint Configuration (Modified DH Parameters)

| Joint | Range (degrees) | Servo Pulse Range | Description |
|-------|-----------------|-------------------|-------------|
| Joint 1 | -120 to 120 | 0-1000 (center: 500) | Base rotation |
| Joint 2 | -180 to 0 | 0-1000 (center: 500) | Shoulder |
| Joint 3 | -120 to 120 | 0-1000 (center: 500) | Elbow |
| Joint 4 | -200 to 20 | 0-1000 (center: 500) | Wrist pitch |
| Joint 5 | -120 to 120 | 0-1000 (center: 500) | Wrist roll |
| Gripper (ID 10) | 0-1000 | 0-1000 | Open/close gripper |

### Link Lengths

| Link | Length (m) | Description |
|------|------------|-------------|
| base_link | 0.152-0.165 | Base height (varies by chassis type) |
| link1 | 0.074 | Upper arm |
| link2 | 0.074 | Forearm |
| link3 | 0.050 | Wrist |
| tool_link | 0.089 | Gripper length |

## ROS2 Control (Recommended)

### Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/servo_controller` | `servo_controller_msgs/ServosPosition` | Set servo positions |
| `/ros_robot_controller/bus_servo/set_position` | `ros_robot_controller_msgs/ServosPosition` | Alternative servo control |

### Message Types

**servo_controller_msgs/ServosPosition:**
```
float64 duration          # Movement duration in seconds
string position_unit      # "pulse" or "degree"
ServoPosition[] position  # Array of servo positions
```

**servo_controller_msgs/ServoPosition:**
```
uint16 id        # Servo ID (1-5 for arm, 10 for gripper)
float32 position # Position value (0-1000 for pulse)
```

### ROS2 Services for Kinematics

| Service | Type | Description |
|---------|------|-------------|
| `/kinematics/set_pose_target` | `kinematics_msgs/SetRobotPose` | Inverse kinematics - position to servo pulses |
| `/kinematics/set_joint_value` | `kinematics_msgs/SetJointValue` | Forward kinematics - servo pulses to position |

**SetRobotPose Service:**
```
# Request
float64[] position    # Target [x, y, z] in meters
float64 pitch         # Target pitch angle in degrees
float64[] pitch_range # Search range if exact pitch fails [-180, 180]
float64 resolution    # Pitch search resolution (default: 1.0)
---
# Response
bool success
uint16[] pulse        # Calculated servo pulse values
uint16[] current_pulse
float64[] rpy         # Resulting roll, pitch, yaw
float64 min_variation
```

**SetJointValue Service:**
```
# Request
float32[] joint_value  # Servo pulse values [j1, j2, j3, j4, j5]
---
# Response
bool success
bool solution
geometry_msgs/Pose pose  # Resulting 3D pose
```

### Example: ROS2 Python Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from servo_controller_msgs.msg import ServoPosition, ServosPosition

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.publisher = self.create_publisher(
            ServosPosition,
            'servo_controller',
            1
        )

    def set_servo_positions(self, duration, positions):
        """
        Set multiple servo positions.

        Args:
            duration: Movement time in seconds
            positions: List of tuples [(servo_id, pulse_value), ...]
        """
        msg = ServosPosition()
        msg.duration = float(duration)
        msg.position_unit = "pulse"

        for servo_id, position in positions:
            servo = ServoPosition()
            servo.id = servo_id
            servo.position = float(position)
            msg.position.append(servo)

        self.publisher.publish(msg)

    def move_to_home(self):
        """Move arm to home position (all servos at center)."""
        self.set_servo_positions(2.0, [
            (1, 500),   # Base rotation
            (2, 500),   # Shoulder
            (3, 500),   # Elbow
            (4, 500),   # Wrist pitch
            (5, 500),   # Wrist roll
            (10, 500),  # Gripper (half open)
        ])

    def open_gripper(self):
        """Open the gripper."""
        self.set_servo_positions(0.5, [(10, 200)])

    def close_gripper(self):
        """Close the gripper."""
        self.set_servo_positions(0.5, [(10, 800)])

def main():
    rclpy.init()
    controller = ArmController()

    # Move to home position
    controller.move_to_home()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example: Using Kinematics Service

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from kinematics_msgs.srv import SetRobotPose

class KinematicsClient(Node):
    def __init__(self):
        super().__init__('kinematics_client')
        self.client = self.create_client(
            SetRobotPose,
            '/kinematics/set_pose_target'
        )
        self.client.wait_for_service()

    def get_servo_positions(self, x, y, z, pitch):
        """
        Calculate servo positions for target 3D position.

        Args:
            x, y, z: Target position in meters
            pitch: Target pitch angle in degrees

        Returns:
            List of servo pulse values if successful
        """
        request = SetRobotPose.Request()
        request.position = [float(x), float(y), float(z)]
        request.pitch = float(pitch)
        request.pitch_range = [-180.0, 180.0]
        request.resolution = 1.0

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            return list(response.pulse)
        return None

def main():
    rclpy.init()
    client = KinematicsClient()

    # Get servo positions for target (0.2m forward, 0m side, 0.15m up)
    pulses = client.get_servo_positions(0.2, 0.0, 0.15, 0)
    if pulses:
        print(f"Servo positions: {pulses}")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Action Groups

Pre-recorded arm movements are stored as `.d6a` files (SQLite databases) in `/home/ubuntu/software/arm_pc/ActionGroups/`.

### Using Action Groups

```python
from servo_controller.action_group_controller import ActionGroupController
from servo_controller_msgs.msg import ServosPosition

# Create publisher
pub = node.create_publisher(ServosPosition, 'servo_controller', 1)

# Initialize controller with action path
controller = ActionGroupController(
    pub,
    '/home/ubuntu/software/arm_pc/ActionGroups'
)

# Run a predefined action
controller.run_action('init')        # Initialize arm position
controller.run_action('voice_pick')  # Pick object action
controller.run_action('voice_give')  # Give object action

# Stop running action
controller.stop_action_group()
```

## Direct SDK Control (No ROS2)

For direct control without ROS2, use the `ros_robot_controller_sdk.py`:

```python
from ros_robot_controller_sdk import Board
import time

# Initialize board connection
board = Board(device="/dev/rrc", baudrate=1000000)
board.enable_reception()

# Set servo positions
# Args: duration (seconds), positions as [[servo_id, pulse_value], ...]
board.bus_servo_set_position(1.0, [
    [1, 500],   # Joint 1 to center
    [2, 500],   # Joint 2 to center
    [3, 500],   # Joint 3 to center
    [4, 500],   # Joint 4 to center
    [5, 500],   # Joint 5 to center
    [10, 500],  # Gripper half open
])

time.sleep(1.5)

# Open gripper
board.bus_servo_set_position(0.5, [[10, 200]])
time.sleep(0.6)

# Close gripper
board.bus_servo_set_position(0.5, [[10, 800]])
```

### SDK Bus Servo Functions

| Function | Description |
|----------|-------------|
| `bus_servo_set_position(duration, positions)` | Move servos to positions |
| `bus_servo_stop(servo_ids)` | Stop specified servos |
| `bus_servo_enable_torque(servo_id, enable)` | Enable/disable servo torque |
| `bus_servo_read_position(servo_id)` | Read current position |
| `bus_servo_read_id(servo_id=254)` | Read servo ID (254 = broadcast) |
| `bus_servo_read_vin(servo_id)` | Read servo voltage |
| `bus_servo_read_temp(servo_id)` | Read servo temperature |
| `bus_servo_set_id(old_id, new_id)` | Change servo ID |
| `bus_servo_set_offset(servo_id, offset)` | Set position offset |
| `bus_servo_save_offset(servo_id)` | Save offset to servo EEPROM |
| `bus_servo_set_angle_limit(servo_id, [min, max])` | Set angle limits |

### Reading Servo State

```python
# Read servo position
position = board.bus_servo_read_position(1)
print(f"Joint 1 position: {position}")

# Read servo voltage (mV)
voltage = board.bus_servo_read_vin(1)
print(f"Servo voltage: {voltage}mV")

# Read servo temperature
temp = board.bus_servo_read_temp(1)
print(f"Servo temperature: {temp}C")

# Check torque state
torque = board.bus_servo_read_torque_state(1)
print(f"Torque enabled: {torque}")
```

## Common Arm Positions

| Position | J1 | J2 | J3 | J4 | J5 | Gripper | Description |
|----------|----|----|----|----|----|---------| ------------|
| Home | 500 | 500 | 500 | 500 | 500 | 500 | Neutral position |
| Ready | 500 | 720 | 210 | 220 | 500 | 200 | Ready to pick |
| Extended | 500 | 300 | 700 | 500 | 500 | 500 | Arm extended forward |
| Folded | 500 | 800 | 200 | 800 | 500 | 500 | Arm folded back |

## Pulse to Angle Conversion

```python
def pulse_to_angle(pulse, joint):
    """
    Convert pulse value to angle in degrees.

    Joint mappings (pulse_min, pulse_max, pulse_mid, angle_min, angle_max, angle_mid):
    - Joint 1: [0, 1000, 500, -120, 120, 0]
    - Joint 2: [0, 1000, 500, 30, -210, -90]
    - Joint 3: [0, 1000, 500, 120, -120, 0]
    - Joint 4: [0, 1000, 500, 30, -210, -90]
    - Joint 5: [0, 1000, 500, -120, 120, 0]
    """
    mappings = {
        1: [0, 1000, 500, -120, 120, 0],
        2: [0, 1000, 500, 30, -210, -90],
        3: [0, 1000, 500, 120, -120, 0],
        4: [0, 1000, 500, 30, -210, -90],
        5: [0, 1000, 500, -120, 120, 0],
    }
    m = mappings[joint]
    angle = ((pulse - m[2]) / (m[1] - m[0])) * (m[4] - m[3]) + m[5]
    return angle
```

## Safety Considerations

1. **Movement Duration**: Always use appropriate duration (minimum 0.02s, maximum 30s)
2. **Position Limits**: Keep pulse values between 0-1000
3. **Collision Avoidance**: Be aware of arm reach and potential collisions with chassis
4. **Torque Control**: Disable torque when manually positioning the arm
5. **Temperature**: Monitor servo temperatures during extended operation

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Servo not responding | Check ID with `bus_servo_read_id(254)` |
| Jerky movement | Increase duration, reduce speed |
| Position drift | Calibrate offset with `bus_servo_set_offset()` |
| Overheating | Add delays between movements, reduce load |
| Cannot reach position | Check if position is within workspace using kinematics |

## References

- ROS2 driver source: `reference/LanderPi/src/driver/servo_controller/`
- Kinematics library: `reference/LanderPi/src/driver/kinematics/`
- SDK source: `reference/LanderPi/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py`
- Message definitions: `reference/LanderPi/src/driver/servo_controller_msgs/`
