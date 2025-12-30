---
name: landerpi-motion
description: Chassis motion control for HiWonder LanderPi robot. Provides motor mapping, direction control, speed parameters, and motion testing. Requires landerpi-core skill to be loaded first for connection and SDK setup.
---

# LanderPi Motion Control

## Overview

Motion control skill for HiWonder LanderPi Mecanum chassis. Provides direct serial control via SDK and ROS2-based control via Docker.

**Prerequisite:** Load `landerpi-core` skill first for connection configuration and SDK setup.

## Motion Commands

| Command | Purpose |
|---------|---------|
| `uv run python validation/test_chassis_direct.py test --direction all --duration 2 --yes` | Test all 6 directions |
| `uv run python validation/test_chassis_direct.py test --duration 3 --yes` | Test forward/backward only |
| `uv run python validation/test_chassis_direct.py motor-test` | Test individual motors |
| `uv run python validation/test_chassis_direct.py stop` | Emergency stop |
| `uv run python validation/test_chassis_direct.py status` | Get battery/IMU status |

### ROS2 Stack Testing

For ROS2-based testing (requires deployed stack):
```bash
uv run python validation/test_chassis_motion.py test  # Test via /cmd_vel topic
```

## Safety Protocol

**CRITICAL: The robot WILL MOVE when motion commands are executed.**

1. Ensure clear area around robot (1m radius minimum)
2. Always verify connection before motion commands
3. Keep test durations short (2-5 seconds)
4. Know emergency stop: `validation/test_chassis_direct.py stop`
5. Only use `--yes` flag when user explicitly approves motion

## Motor Mapping (Mecanum Chassis)

```
Front
┌─────────────────┐
│  M1(FL)  M3(FR) │
│                 │
│  M2(BL)  M4(BR) │
└─────────────────┘
Back
```

- **M1** = Front-Left
- **M2** = Back-Left
- **M3** = Front-Right
- **M4** = Back-Right

## Motor Sign Convention

**CRITICAL: The left and right motors have opposite sign conventions.**

| Side | Motors | Forward | Backward |
|------|--------|---------|----------|
| Left | M1, M2 | **Negative** (-) | Positive (+) |
| Right | M3, M4 | **Positive** (+) | Negative (-) |

This is because the motors are mounted facing opposite directions. When implementing kinematics or control:
- For forward motion: left wheels get negative speed, right wheels get positive speed
- For backward motion: left wheels get positive speed, right wheels get negative speed

**Example:** To move forward at 0.3 m/s:
```python
board.set_motor_speed([[1, -0.3], [2, -0.3], [3, 0.3], [4, 0.3]])
#                      ^^ LEFT negative ^^   ^^ RIGHT positive ^^
```

## Direction Control

Speed: 0.3 m/s (maximum safe speed)

| Direction | M1 (FL) | M2 (BL) | M3 (FR) | M4 (BR) | Description |
|-----------|---------|---------|---------|---------|-------------|
| Forward | -0.3 | -0.3 | +0.3 | +0.3 | Left negative, right positive |
| Backward | +0.3 | +0.3 | -0.3 | -0.3 | Opposite of forward |
| Turn Right | -0.3 | -0.3 | -0.3 | -0.3 | All negative (CW rotation) |
| Turn Left | +0.3 | +0.3 | +0.3 | +0.3 | All positive (CCW rotation) |
| Strafe Right | -0.3 | +0.3 | -0.3 | +0.3 | Diagonal pattern |
| Strafe Left | +0.3 | -0.3 | +0.3 | -0.3 | Opposite diagonal |

## Direct Serial Control (SDK)

```python
from ros_robot_controller_sdk import Board

board = Board()  # Opens /dev/ttyACM0 at 1000000 baud
board.enable_reception()

# Move FORWARD at 0.3 m/s
board.set_motor_speed([[1, -0.3], [2, -0.3], [3, 0.3], [4, 0.3]])

# Move BACKWARD
board.set_motor_speed([[1, 0.3], [2, 0.3], [3, -0.3], [4, -0.3]])

# TURN RIGHT (clockwise)
board.set_motor_speed([[1, -0.3], [2, -0.3], [3, -0.3], [4, -0.3]])

# TURN LEFT (counter-clockwise)
board.set_motor_speed([[1, 0.3], [2, 0.3], [3, 0.3], [4, 0.3]])

# STRAFE RIGHT
board.set_motor_speed([[1, -0.3], [2, 0.3], [3, -0.3], [4, 0.3]])

# STRAFE LEFT
board.set_motor_speed([[1, 0.3], [2, -0.3], [3, 0.3], [4, -0.3]])

# STOP all motors
board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
```

## ROS2 Control (Persistent Stack)

The LanderPi uses a Docker Compose-based ROS2 stack that persists across reboots.

### Deployment Commands

| Command | Purpose |
|---------|---------|
| `uv run python deploy_ros2_stack.py deploy` | Deploy and start ROS2 stack |
| `uv run python deploy_ros2_stack.py stop` | Stop ROS2 stack |
| `uv run python deploy_ros2_stack.py logs` | View stack logs |
| `uv run python deploy_ros2_stack.py logs -f` | Follow logs in real-time |

### ROS2 Motion Test

```bash
# Test via ROS2 (requires deployed stack)
uv run python validation/test_chassis_motion.py test
```

### Architecture

```
/cmd_vel (Twist) --> cmd_vel_bridge --> /ros_robot_controller/set_motor --> STM32
```

The `cmd_vel_bridge` node:
- Subscribes to standard `/cmd_vel` topic
- Performs mecanum kinematics calculations
- Publishes to `/ros_robot_controller/set_motor`
- Includes 500ms watchdog (auto-stop if no commands)

### Manual ROS2 Commands

```bash
# Quick test via Docker exec
docker exec landerpi-ros2 bash -c '
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {z: 0.0}}"'

# Check topics
docker exec landerpi-ros2 bash -c '
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 topic list'
```

**Twist message mapping:**
- `linear.x > 0`: Forward
- `linear.x < 0`: Backward
- `linear.y > 0`: Strafe left (Mecanum only)
- `linear.y < 0`: Strafe right (Mecanum only)
- `angular.z > 0`: Turn left (CCW)
- `angular.z < 0`: Turn right (CW)

### Persistence

The stack uses `restart: unless-stopped` in Docker Compose, so it:
- Starts automatically on boot
- Survives reboots
- Only stops when explicitly stopped with `docker compose down`

### Configuration

Robot geometry parameters in `config/robot_params.yaml`:
- `wheel_radius`: 0.05m (50mm)
- `wheel_base`: 0.15m (front-to-back)
- `track_width`: 0.15m (left-to-right)
- `max_wheel_rps`: 3.0 (safety limit)
- `cmd_vel_timeout`: 0.5s (watchdog)

## Motion Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Max speed | 0.3 m/s | Direct serial control |
| ROS2 speed | 0.15 m/s | Conservative for safety |
| Test duration | 2-3 sec | Recommended for testing |
| Serial baud | 1000000 | STM32 communication |
| Serial port | /dev/ttyACM0 | Motor controller |

## Autonomous Navigation Parameters

When implementing autonomous navigation with sensor feedback loops, use **turn commitment** to prevent oscillation.

### Problem: Turn Oscillation

Without commitment, a fast control loop (5-10 Hz) causes:
1. Robot turns left slightly
2. Re-reads sensors, picks slightly different direction
3. Robot turns right slightly
4. Oscillates without making progress

### Solution: Turn Commitment

Once a turn decision is made, commit to it for a minimum duration before re-evaluating.

| Parameter | Recommended | Description |
|-----------|-------------|-------------|
| Turn speed | 1.5 rad/s | ~86°/sec - fast enough to complete turns |
| Min turn duration | 0.8 sec | Minimum time before re-evaluating direction |
| Turn complete threshold | 20° | Consider turn done when target within this angle |
| Escape turn time | 0.5-3.0 sec | Calculate from angle: `angle_rad / turn_speed` |

### Implementation Pattern

```python
# State tracking
turn_direction = 0      # -1=right, 0=none, 1=left
turn_start_time = 0.0
turn_target = ""

def navigate():
    # If currently turning, check commitment
    if turn_direction != 0:
        elapsed = time.time() - turn_start_time
        if elapsed < MIN_TURN_DURATION:
            # Keep turning - don't change direction yet
            robot.move(0, 0, TURN_SPEED * turn_direction)
            return

        # Check if turn complete
        if target_now_in_front():
            turn_direction = 0  # Done turning
        else:
            # Keep turning same direction
            robot.move(0, 0, TURN_SPEED * turn_direction)
            return

    # Not turning - evaluate new direction
    if need_to_turn(target_angle):
        turn_direction = 1 if target_angle > 0 else -1
        turn_start_time = time.time()
        robot.move(0, 0, TURN_SPEED * turn_direction)
    else:
        robot.move(forward_speed, 0, 0)
```

### Escape Maneuver

When stuck, perform a committed escape:
1. Back up for 2 seconds
2. Stop and re-scan environment
3. Calculate turn time: `turn_time = abs(target_angle_rad) / turn_speed`
4. Execute full turn without interruption
5. Resume normal navigation

## Troubleshooting

### Problem: Motors don't respond

**Symptoms:**
- Commands execute but robot doesn't move
- No error messages

**Diagnosis:**
```bash
# Check serial port exists
ls /dev/ttyACM*

# Check battery voltage
uv run python validation/test_chassis_direct.py status
```

**Solutions:**
1. Check battery level (should be >10V)
2. Verify serial port permissions
3. Check motor controller power switch

### Problem: Robot moves in wrong direction

**Symptoms:**
- Forward command moves backward
- Turning is reversed

**Solution:**
Motor wiring may be swapped. Use motor-test to identify:
```bash
uv run python validation/test_chassis_direct.py motor-test --motor 1
```

Test each motor individually and verify physical position matches expected.

### Problem: Uneven movement

**Symptoms:**
- Robot drifts left or right during forward motion
- Turns are asymmetric

**Solution:**
1. Check wheel tightness
2. Verify floor surface is level
3. Calibrate motor speeds if needed
