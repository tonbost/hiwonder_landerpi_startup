# LanderPi System Requirements

## Hardware

### Raspberry Pi 5

| Component | Specification |
|-----------|---------------|
| Model | Raspberry Pi 5 |
| RAM | 4GB or 8GB recommended |
| Storage | 32GB+ microSD (Class 10 or better) |
| Power | 5V/5A USB-C power supply |

### Peripherals

| Device | Interface | Device Path |
|--------|-----------|-------------|
| STM32 Controller | USB Serial | `/dev/ttyACM0` |
| CH340 Serial | USB | `/dev/ttyUSB0`, `/dev/ttyUSB1` |
| I2C Bus | GPIO | `/dev/i2c-1` |
| SPI Bus | GPIO | `/dev/spidev0.0`, `/dev/spidev0.1` |
| GPIO | GPIO | `/dev/gpiochip0` |
| Aurora 930 Camera | USB | VID:3251 |

## Software

### Operating System

| Property | Expected Value |
|----------|----------------|
| Distribution | Ubuntu |
| Version | 22.04 LTS (Jammy) or 24.04 LTS (Noble) |
| Architecture | arm64 (aarch64) |
| Image | HiWonder LanderPi System Image |

**Note:** Ubuntu 25.x is NOT supported for ROS2 Humble.

### ROS2

| Component | Version |
|-----------|---------|
| Distribution | Humble Hawksbill |
| Installation | `/opt/ros/humble/` |
| Workspace | `~/ros2_ws/` |

### Required ROS2 Packages

Core packages in ros2_ws:
- `ros_robot_controller` - Main chassis controller
- `controller` - Kinematics and odometry
- `peripherals` - IMU, sensors
- `calibration` - Velocity calibration tools
- `app` - Application launch files

Navigation packages (optional):
- `navigation2`
- `nav2-bringup`
- `slam-toolbox`

### Docker

| Component | Details |
|-----------|---------|
| Engine | Docker CE |
| Image | `landerpi-ros2:latest` |
| Base | `ros:humble-ros-base` |

## Directory Structure

### Expected Home Directory

```
/home/<user>/
├── ros2_ws/
│   ├── src/
│   │   ├── driver/
│   │   │   └── controller/
│   │   │       ├── controller/
│   │   │       │   ├── mecanum.py
│   │   │       │   ├── ackermann.py
│   │   │       │   └── odom_publisher_node.py
│   │   │       └── config/
│   │   │           └── calibrate_params.yaml
│   │   ├── ros_robot_controller/
│   │   ├── peripherals/
│   │   ├── calibration/
│   │   └── app/
│   ├── build/
│   ├── install/
│   └── log/
├── third_party_ros2/
├── Music/
└── Share/
```

### Configuration Files

| File | Purpose |
|------|---------|
| `/boot/firmware/config.txt` | Hardware config (I2C, SPI) |
| `~/.bashrc` | ROS2 environment setup |
| `~/ros2_ws/src/driver/controller/config/calibrate_params.yaml` | Velocity calibration |

## User Groups

The robot user must be in these groups:

| Group | Purpose |
|-------|---------|
| `dialout` | Serial port access |
| `audio` | Audio devices |
| `video` | Video devices, vcgencmd |
| `i2c` | I2C bus access |
| `gpio` | GPIO access |
| `docker` | Docker without sudo |

Verify with: `groups`

## Network

### Default Configuration

| Setting | Value |
|---------|-------|
| Mode | AP (Access Point) by default |
| SSID | HW_LanderPi_XXXX |
| Default IP | 192.168.149.1 (in AP mode) |

### Station Mode

When connected to external WiFi:
- IP assigned by DHCP
- Check with `ip addr show wlan0`

## Services

### Auto-start Services

| Service | Purpose | Control |
|---------|---------|---------|
| ros_app | ROS2 auto-start | `~/.stop_ros.sh` to stop |
| ssh | Remote access | `systemctl status ssh` |

### Stopping Auto-start

Before manual ROS2 work:
```bash
~/.stop_ros.sh
```

## Verification Commands

### Quick System Check

```bash
# OS version
cat /etc/os-release | grep VERSION

# ROS2 installed
source /opt/ros/humble/setup.bash && ros2 --version

# Workspace exists
ls ~/ros2_ws/src

# Serial ports
ls /dev/ttyACM* /dev/ttyUSB*

# I2C
ls /dev/i2c*

# Docker
docker images | grep landerpi

# User groups
groups | tr ' ' '\n' | grep -E "dialout|audio|video|i2c|gpio|docker"
```

### Full Health Check

Use the project's health check tool:
```bash
uv run python checkLanderPi.py check --host <IP> --user <USER> --password <PASS>
```
