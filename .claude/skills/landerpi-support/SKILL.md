---
name: landerpi-support
description: This skill provides support for HiWonder LanderPi robot operations including SSH connection management, system diagnostics, chassis motion control, deployment, and troubleshooting. Use when working with LanderPi robots, checking robot status, controlling robot movement, deploying software, or diagnosing issues like missing ROS2 workspace or connectivity problems.
---

# LanderPi Support

## Overview

This skill enables remote management and control of HiWonder LanderPi robots running on Raspberry Pi 5. It covers SSH connectivity, ROS2-based chassis control, system health checks, and troubleshooting common deployment issues.

## Quick Start

### Connection Configuration

Load credentials from `config.json` in the project root:
```json
{
  "host": "<PI_IP>",
  "user": "<USERNAME>",
  "password": "<PASSWORD>"
}
```

### Available Tools

| Tool | Command | Purpose |
|------|---------|---------|
| Health Check | `uv run python checkLanderPi.py check --host <IP> --user <USER> --password <PASS>` | Full system diagnostics |
| Quick Check | `uv run python checkLanderPi.py quick --host <IP> --user <USER> --password <PASS>` | Critical systems only |
| Full Motion Test | `uv run python test_chassis_direct.py test --direction all --duration 2 --yes` | Test all 6 directions |
| Quick Motion Test | `uv run python test_chassis_direct.py test --duration 3 --yes` | Test forward/backward only |
| Motor Test | `uv run python test_chassis_direct.py motor-test` | Test individual motors for mapping |
| ROS2 Motion Test | `uv run python test_chassis_motion.py test --host <IP> --user <USER> --password <PASS>` | Test via ROS2 |
| Emergency Stop | `uv run python test_chassis_direct.py stop` | Stop robot immediately |
| Robot Status | `uv run python test_chassis_direct.py status` | Get battery/IMU status |
| Deploy | `uv run python setup_landerpi.py deploy --host <IP> --user <USER> --password <PASS>` | Full setup deployment |

**Note:** Direct motion test reads credentials from `config.json` automatically.

## Diagnostics

### System Health Check

Before any robot operation, verify system status:

1. **Connection** - SSH connectivity to robot
2. **User verification** - Confirm user exists (`id <username>`)
3. **ROS2 status** - Check if ros2_ws exists and ROS2 is installed
4. **Hardware** - Serial ports, I2C, GPIO availability
5. **Docker** - Images and containers status

### Expected System State

A properly configured LanderPi should have:

| Component | Expected | Check Command |
|-----------|----------|---------------|
| OS | Ubuntu 22.04/24.04 (HiWonder image) | `cat /etc/os-release` |
| ROS2 Workspace | `/home/<user>/ros2_ws` | `ls ~/ros2_ws` |
| ros_robot_controller | Installed in ros2_ws | `ros2 pkg list \| grep robot_controller` |
| Serial ports | `/dev/ttyACM0`, `/dev/ttyUSB*` | `ls /dev/tty*` |
| I2C | `/dev/i2c-1` | `ls /dev/i2c*` |
| Docker | landerpi-ros2 image | `docker images` |

## Chassis Motion Control

### Two Control Methods

1. **Direct Serial (Recommended)** - Uses SDK to communicate directly with STM32
   - No ROS2 installation required
   - Works on any Ubuntu version
   - Uses `test_chassis_direct.py`

2. **ROS2 Topics** - Uses ros_robot_controller node
   - Requires full HiWonder ROS2 stack
   - Uses `test_chassis_motion.py`

### Direct Serial Control (No ROS2)

The `ros_robot_controller_sdk.py` communicates directly with the STM32 controller via `/dev/ttyACM0`.

**Motor Mapping (Mecanum Chassis):**
- M1 = front-left
- M2 = back-left
- M3 = front-right
- M4 = back-right

**Direction Control (all speeds at 0.3 m/s max):**
| Direction | M1 (FL) | M2 (BL) | M3 (FR) | M4 (BR) |
|-----------|---------|---------|---------|---------|
| Forward | - | - | + | + |
| Backward | + | + | - | - |
| Turn Right | - | - | - | - |
| Turn Left | + | + | + | + |
| Strafe Right | - | + | - | + |
| Strafe Left | + | - | + | - |

```python
from ros_robot_controller_sdk import Board

board = Board()  # Opens /dev/ttyACM0 at 1000000 baud
board.enable_reception()

# Move FORWARD at 0.3 m/s (max speed)
# Left side negative, right side positive
board.set_motor_speed([[1, -0.3], [2, -0.3], [3, 0.3], [4, 0.3]])

# Move BACKWARD
board.set_motor_speed([[1, 0.3], [2, 0.3], [3, -0.3], [4, -0.3]])

# Stop all motors
board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

# Get battery voltage (mV)
voltage = board.get_battery()

# Get IMU data (ax, ay, az, gx, gy, gz)
imu = board.get_imu()

# Beep buzzer
board.set_buzzer(1000, 0.1, 0.1, 1)  # freq, on_time, off_time, repeat
```

### ROS2 Control (Requires HiWonder Image)

Prerequisites:
1. ROS2 controller must be running on robot
2. Clear area around robot (safety zone)
3. User approval required

Starting the Controller:
```bash
ros2 launch ros_robot_controller ros_robot_controller.launch.py
```

### Motion Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Max distance | 20 cm | Safety limit (can override with --duration) |
| Speed | 0.3 m/s | Max speed for direct serial |
| Duration | 3 sec | Recommended for testing |
| Topic (ROS2) | `/ros_robot_controller/cmd_vel` | Twist messages |
| Message type | `geometry_msgs/msg/Twist` | linear.x for forward/back |

### ROS2 Velocity Command Format

```bash
ros2 topic pub --once /ros_robot_controller/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

- `linear.x > 0`: Forward
- `linear.x < 0`: Backward
- `linear.y`: Lateral (Mecanum only)
- `angular.z`: Rotation

## Troubleshooting

### Problem: ROS2 workspace not found

**Symptoms:**
- `ls ~/ros2_ws` returns "No such file or directory"
- `ros2` command not found

**Diagnosis:**
```bash
cat /etc/os-release  # Check OS version
ls /home/*/ros2_ws   # Search all users
```

**Cause:** Robot not running HiWonder system image (likely fresh Ubuntu install)

**Solution:** Flash HiWonder system image or install ROS2 packages manually

### Problem: cmd_vel topic not found

**Symptoms:**
- Motion test fails with "topic not found"
- `ros2 topic list` doesn't show `/ros_robot_controller/cmd_vel`

**Solution:** Start the robot controller:
```bash
ros2 launch ros_robot_controller ros_robot_controller.launch.py
```

### Problem: User does not exist

**Symptoms:**
- SSH connection fails
- `id <username>` returns "no such user"

**Solution:**
1. Check available users: `cat /etc/passwd | grep -E "^[^:]+:" | cut -d: -f1`
2. Update config.json with correct username
3. Default HiWonder user is typically `ubuntu` or `pi`

### Problem: Permission denied on serial port

**Symptoms:**
- Cannot access `/dev/ttyUSB*` or `/dev/ttyACM*`

**Solution:**
```bash
sudo usermod -aG dialout $USER
# Logout and login again
```

### Problem: Robot on wrong network

**Symptoms:**
- Cannot ping robot IP
- SSH connection timeout

**Solution:**
1. Connect to robot via HDMI/monitor
2. Check IP: `ip addr show`
3. Update config.json with correct IP

## Chassis Types

LanderPi supports three chassis configurations:

| Type | Movement | Kinematics File |
|------|----------|-----------------|
| Mecanum | Omnidirectional | `mecanum.py` |
| Ackerman | Car-like steering | `ackermann.py` |
| Tank | Differential drive | `tank.py` (uses mecanum) |

## Resources

### references/

- `ros2-control.md` - Detailed ROS2 topic and message reference
- `troubleshooting.md` - Extended troubleshooting guide
- `system-requirements.md` - Full system requirements and expected state
