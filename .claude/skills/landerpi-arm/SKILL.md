---
name: landerpi-arm
description: Robotic arm control for HiWonder LanderPi robot. Provides 5-DOF arm servo control, gripper operations, kinematics services, and action groups. Requires landerpi-core skill to be loaded first for connection and SDK setup.
---

# LanderPi Arm Control

## Overview

Robotic arm control skill for HiWonder LanderPi 5-DOF arm with gripper. Provides direct servo control via SDK and ROS2-based control with kinematics services.

**Prerequisite:** Load `landerpi-core` skill first for connection configuration and SDK setup.

## Arm Architecture

### Servo Configuration

| Servo ID | Joint | Range (pulse) | Range (degrees) | Description |
|----------|-------|---------------|-----------------|-------------|
| 1 | Joint 1 | 0-1000 | -120 to 120 | Base rotation |
| 2 | Joint 2 | 0-1000 | -180 to 0 | Shoulder |
| 3 | Joint 3 | 0-1000 | -120 to 120 | Elbow |
| 4 | Joint 4 | 0-1000 | -200 to 20 | Wrist pitch |
| 5 | Joint 5 | 0-1000 | -120 to 120 | Wrist roll |
| 10 | Gripper | 0-1000 | N/A | Gripper open/close |

**Note:** Center position is 500 for all servos. Gripper: 200=open, 800=closed.

### Link Lengths

| Link | Length (m) | Description |
|------|------------|-------------|
| base_link | 0.152-0.165 | Base height (varies by chassis) |
| link1 | 0.074 | Upper arm |
| link2 | 0.074 | Forearm |
| link3 | 0.050 | Wrist |
| tool_link | 0.089 | Gripper length |

## Safety Protocol

**CRITICAL: Arm movements can cause collisions or pinch hazards.**

1. Start with arm in home position (all servos at 500)
2. Use slow durations (1-2 seconds) for testing
3. Keep clear of arm reach envelope
4. Test gripper without objects first
5. Know emergency stop procedure

## ROS2 Control (Recommended)

### Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/servo_controller` | `servo_controller_msgs/ServosPosition` | Set servo positions |

### Message Format

```
ServosPosition:
  float64 duration          # Movement time in seconds
  string position_unit      # "pulse" (default)
  ServoPosition[] position  # Array of servo positions

ServoPosition:
  uint16 id        # Servo ID (1-5 for arm, 10 for gripper)
  float32 position # Pulse value (0-1000)
```

### Kinematics Services

| Service | Type | Description |
|---------|------|-------------|
| `/kinematics/set_pose_target` | `SetRobotPose` | Position to servo pulses (IK) |
| `/kinematics/set_joint_value` | `SetJointValue` | Servo pulses to position (FK) |

### ROS2 Python Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from servo_controller_msgs.msg import ServoPosition, ServosPosition

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.pub = self.create_publisher(ServosPosition, 'servo_controller', 1)

    def set_servos(self, duration, positions):
        """Set servo positions. positions: [(id, pulse), ...]"""
        msg = ServosPosition()
        msg.duration = float(duration)
        msg.position_unit = "pulse"
        for servo_id, pulse in positions:
            servo = ServoPosition()
            servo.id = servo_id
            servo.position = float(pulse)
            msg.position.append(servo)
        self.pub.publish(msg)

    def home(self):
        """Move to home position."""
        self.set_servos(2.0, [(1, 500), (2, 500), (3, 500), (4, 500), (5, 500), (10, 500)])

    def open_gripper(self):
        self.set_servos(0.5, [(10, 200)])

    def close_gripper(self):
        self.set_servos(0.5, [(10, 800)])
```

## Direct SDK Control (No ROS2)

For direct control without ROS2, use `ros_robot_controller_sdk.py`:

```python
from ros_robot_controller_sdk import Board
import time

board = Board(device="/dev/rrc", baudrate=1000000)
board.enable_reception()

# Move to home position
board.bus_servo_set_position(2.0, [
    [1, 500], [2, 500], [3, 500], [4, 500], [5, 500], [10, 500]
])
time.sleep(2.5)

# Open gripper
board.bus_servo_set_position(0.5, [[10, 200]])
time.sleep(0.6)

# Close gripper
board.bus_servo_set_position(0.5, [[10, 800]])
time.sleep(0.6)

# Stop all servos
board.bus_servo_stop([1, 2, 3, 4, 5, 10])
```

### SDK Bus Servo Functions

| Function | Description |
|----------|-------------|
| `bus_servo_set_position(duration, [[id, pulse], ...])` | Move servos |
| `bus_servo_stop([servo_ids])` | Stop specified servos |
| `bus_servo_enable_torque(id, enable)` | Enable/disable torque |
| `bus_servo_read_position(id)` | Read current position |
| `bus_servo_read_id(254)` | Scan for servo IDs |
| `bus_servo_read_vin(id)` | Read servo voltage |
| `bus_servo_read_temp(id)` | Read servo temperature |
| `bus_servo_set_offset(id, offset)` | Set position offset |
| `bus_servo_save_offset(id)` | Save offset to EEPROM |

## Common Positions

| Position | J1 | J2 | J3 | J4 | J5 | Grip | Description |
|----------|----|----|----|----|----|----|-------------|
| Home | 500 | 500 | 500 | 500 | 500 | 500 | Neutral |
| Ready | 500 | 720 | 210 | 220 | 500 | 200 | Ready to pick |
| Extended | 500 | 300 | 700 | 500 | 500 | 500 | Forward reach |
| Folded | 500 | 800 | 200 | 800 | 500 | 500 | Folded back |

## Action Groups

Pre-recorded movements stored as `.d6a` files (SQLite databases):

**Location:** `/home/ubuntu/software/arm_pc/ActionGroups/`

```python
from servo_controller.action_group_controller import ActionGroupController
from servo_controller_msgs.msg import ServosPosition

pub = node.create_publisher(ServosPosition, 'servo_controller', 1)
controller = ActionGroupController(pub, '/home/ubuntu/software/arm_pc/ActionGroups')

# Run predefined actions
controller.run_action('init')        # Initialize position
controller.run_action('voice_pick')  # Pick object
controller.run_action('voice_give')  # Give object

# Stop running action
controller.stop_action_group()
```

## Pulse to Angle Conversion

```python
# Joint mappings: [pulse_min, pulse_max, pulse_mid, angle_min, angle_max, angle_mid]
JOINT_MAPS = {
    1: [0, 1000, 500, -120, 120, 0],
    2: [0, 1000, 500, 30, -210, -90],
    3: [0, 1000, 500, 120, -120, 0],
    4: [0, 1000, 500, 30, -210, -90],
    5: [0, 1000, 500, -120, 120, 0],
}

def pulse_to_angle(pulse, joint):
    m = JOINT_MAPS[joint]
    return ((pulse - m[2]) / (m[1] - m[0])) * (m[4] - m[3]) + m[5]

def angle_to_pulse(angle, joint):
    m = JOINT_MAPS[joint]
    return ((angle - m[5]) / (m[4] - m[3])) * (m[1] - m[0]) + m[2]
```

## Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Min duration | 0.02s | Fastest movement |
| Max duration | 30s | Slowest movement |
| Pulse range | 0-1000 | Valid for all servos |
| Serial port | /dev/rrc | Motor controller |
| Baud rate | 1000000 | STM32 communication |

## Troubleshooting

### Problem: Servo not responding

**Symptoms:**
- Command executes but servo doesn't move
- No error messages

**Diagnosis:**
```python
board = Board()
board.enable_reception()
# Scan for connected servos (254 = broadcast)
servo_id = board.bus_servo_read_id(254)
print(f"Found servo: {servo_id}")
```

**Solutions:**
1. Check servo power (should be >10V)
2. Verify servo ID is correct
3. Check servo cable connections
4. Enable torque: `board.bus_servo_enable_torque(id, True)`

### Problem: Servo overheating

**Symptoms:**
- Servo feels hot
- Reduced torque or stuttering

**Diagnosis:**
```python
temp = board.bus_servo_read_temp(1)
print(f"Temperature: {temp}C")
```

**Solutions:**
1. Add delays between movements
2. Reduce load on servo
3. Let servo cool before continuing
4. Check for mechanical binding

### Problem: Position drift

**Symptoms:**
- Servo doesn't return to exact position
- Gradual offset accumulation

**Solution:**
```python
# Calibrate offset
board.bus_servo_set_offset(servo_id, offset_value)
board.bus_servo_save_offset(servo_id)
```

### Problem: Gripper doesn't close fully

**Symptoms:**
- Object slips from gripper
- Gripper stops before closing

**Solutions:**
1. Increase gripper pulse (try 850-900)
2. Check for debris in gripper mechanism
3. Verify servo torque is enabled

### Problem: Kinematics returns no solution

**Symptoms:**
- IK service returns `success: False`
- Empty pulse array

**Solutions:**
1. Target position may be outside workspace
2. Try different pitch angles
3. Increase pitch_range search area
4. Check link lengths match robot configuration

## Reference Files

- Full documentation: `armLanderpiHowTo.md`
- SDK source: `reference/LanderPi/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py`
- Kinematics: `reference/LanderPi/src/driver/kinematics/`
- Servo controller: `reference/LanderPi/src/driver/servo_controller/`
