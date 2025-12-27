---
name: landerpi-core
description: Core infrastructure for HiWonder LanderPi robot operations. Provides SSH connection management, system diagnostics, Docker configuration, and base SDK setup. This skill should be loaded first before landerpi-motion or landerpi-lidar skills.
---

# LanderPi Core

## Overview

Core infrastructure skill for HiWonder LanderPi robots running on Raspberry Pi 5. Provides connection management, diagnostics, and Docker-based ROS2 environment setup.

**IMPORTANT: ROS2 runs inside Docker containers on the robot, not natively installed.**

## Connection Configuration

Load credentials from `config.json` in the project root:
```json
{
  "host": "<PI_IP>",
  "user": "<USERNAME>",
  "password": "<PASSWORD>"
}
```

## Available Tools

| Tool | Command | Purpose |
|------|---------|---------|
| Health Check | `uv run python checkLanderPi.py check` | Full system diagnostics |
| Quick Check | `uv run python checkLanderPi.py quick` | Critical systems only |
| Connect Test | `uv run python setup_landerpi.py connect` | Test SSH connection |
| Deploy | `uv run python setup_landerpi.py deploy` | Full setup deployment |
| Robot Status | `uv run python test_chassis_direct.py status` | Battery/IMU status |

All tools read credentials from `config.json` automatically.

## System Health Check

Before any robot operation, verify system status:

1. **Connection** - SSH connectivity to robot
2. **User verification** - Confirm user exists (`id <username>`)
3. **Hardware** - Serial ports, I2C, GPIO availability
4. **Docker** - Images and containers status

## Expected System State

A properly configured LanderPi should have:

| Component | Expected | Check Command |
|-----------|----------|---------------|
| OS | Ubuntu 22.04/24.04 | `cat /etc/os-release` |
| Docker | Installed with `landerpi-ros2:latest` image | `docker images` |
| ROS2 | Available via Docker | `docker run --rm landerpi-ros2:latest ros2 topic list` |
| Motor controller | `/dev/ttyACM0` | `ls /dev/ttyACM*` |
| Lidar device | `/dev/ttyUSB0` or `/dev/ttyUSB1` | `ls /dev/ttyUSB*` |
| I2C | `/dev/i2c-1` | `ls /dev/i2c*` |
| Motor SDK | `~/ros_robot_controller/ros_robot_controller_sdk.py` | `ls ~/ros_robot_controller/` |

## Docker Configuration

**Always use `landerpi-ros2:latest`** - it has pip3 and pyserial support.

Standard Docker run pattern:
```bash
docker run --rm --privileged --network host -v /dev:/dev landerpi-ros2:latest \
    bash -c 'source /opt/ros/humble/setup.bash && <ROS2_COMMAND>'
```

## Motor Controller SDK

The `ros_robot_controller_sdk.py` enables direct serial communication with the STM32 motor controller.

Location on robot: `~/ros_robot_controller/ros_robot_controller_sdk.py`

Basic usage:
```python
from ros_robot_controller_sdk import Board

board = Board()  # Opens /dev/ttyACM0 at 1000000 baud
board.enable_reception()

# Get battery voltage (mV)
voltage = board.get_battery()

# Get IMU data (ax, ay, az, gx, gy, gz)
imu = board.get_imu()

# Beep buzzer
board.set_buzzer(1000, 0.1, 0.1, 1)  # freq, on_time, off_time, repeat
```

## Troubleshooting

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

### Problem: Docker image not found

**Symptoms:**
- "Unable to find image 'landerpi-ros2:latest' locally"

**Solution:**
Run deployment to build the image:
```bash
uv run python setup_landerpi.py deploy
```

## Chassis Types

LanderPi supports three chassis configurations:

| Type | Movement | Kinematics |
|------|----------|------------|
| Mecanum | Omnidirectional | 4 independent wheels |
| Ackerman | Car-like steering | Front wheel steering |
| Tank | Differential drive | Left/right track control |
