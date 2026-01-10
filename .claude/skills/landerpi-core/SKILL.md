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
| Robot Status | `uv run python validation/test_chassis_direct.py status` | Battery/IMU status |
| Voice Deploy | `uv run python deploy_voicecontroller.py deploy` | Deploy TARS voice control |
| Voice Test | `uv run python deploy_voicecontroller.py test` | Test voice components |
| Hailo Check | `uv run python deploy_hailo8.py check` | Hailo hardware/driver status |
| Hailo Install | `uv run python deploy_hailo8.py install` | Install HailoRT 4.23 driver |
| Hailo Install (Force) | `uv run python deploy_hailo8.py install --force` | Force reinstall HailoRT |
| Hailo Deploy | `uv run python deploy_hailo8.py deploy` | Upload models and ROS2 node |
| Hailo Test | `uv run python deploy_hailo8.py test` | Run validation test |
| Hailo Status | `uv run python deploy_hailo8.py status` | Show device info |
| Whisplay Check | `uv run python deploy_whisplay.py check` | Check Whisplay HAT status |
| Whisplay Install | `uv run python deploy_whisplay.py install` | Install WM8960 audio driver |
| Whisplay Test | `uv run python deploy_whisplay.py test` | LCD/button/LED test |
| Whisplay Mic Test | `uv run python deploy_whisplay.py mic-test` | Microphone/speaker test |
| Whisplay Status | `uv run python deploy_whisplay.py status` | Detailed audio status |

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
| OS | Ubuntu 22.04/24.04/25.10 | `cat /etc/os-release` |
| Docker | Installed with `landerpi-ros2:latest` image | `docker images` |
| ROS2 | Available via Docker | `docker run --rm landerpi-ros2:latest ros2 topic list` |
| Motor controller | `/dev/ttyACM0` | `ls /dev/ttyACM*` |
| Lidar device | `/dev/ttyUSB0` or `/dev/ttyUSB1` | `ls /dev/ttyUSB*` |
| I2C | `/dev/i2c-1` | `ls /dev/i2c*` |
| Motor SDK | `~/ros_robot_controller/ros_robot_controller_sdk.py` | `ls ~/ros_robot_controller/` |
| Hailo-8 (optional) | `/dev/hailo0`, HailoRT 4.23.0 | `hailortcli --version` |

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

## ROS2 Stack Architecture

The persistent ROS2 stack runs all drivers in a single Docker Compose service.

### Stack Components

| Node | Topic | Purpose |
|------|-------|---------|
| `ros_robot_controller` | `/ros_robot_controller/*` | STM32 communication (vendor) |
| `cmd_vel_bridge` | `/cmd_vel` -> motors | Motion control |
| `arm_controller` | `/arm/cmd`, `/arm/state` | Arm servo control |
| `lidar_driver` | `/scan` | Lidar scanning |
| `camera_driver` | `/aurora/*` | Depth camera (vendor) |

### Voice Control (TARS)

Voice control runs as a separate systemd service (`tars-voice`) that uses the ROS2 stack for motion.

| Component | Location | Purpose |
|-----------|----------|---------|
| Voice Controller | `~/robot_voicecontroller.py` | ASR + LLM + TTS + ROS2 control |
| Systemd Service | `/etc/systemd/system/tars-voice.service` | Auto-start on boot |
| Audio | WonderEcho Pro (card 1) | Microphone + speaker |

Voice control commands robot via `docker exec landerpi-ros2 ros2 topic pub /controller/cmd_vel`.

### Deployment

```bash
# Deploy and start persistent ROS2 stack
uv run python deploy_ros2_stack.py deploy

# Stop stack
uv run python deploy_ros2_stack.py stop

# View logs
uv run python deploy_ros2_stack.py logs
uv run python deploy_ros2_stack.py logs -f  # Follow
```

### Testing Pattern

All `*_ros2.py` test scripts follow this pattern:
1. SSH to robot
2. `docker exec landerpi-ros2` to run ROS2 commands
3. Use `ros2 topic pub` to send commands
4. Use `ros2 topic echo` to read data

### Direct vs ROS2 Testing

| Script Type | When to Use |
|-------------|-------------|
| `test_*.py` (direct) | Quick testing, debugging, offline recovery |
| `test_*_ros2.py` | Production, nav2/SLAM integration |

## ROS2 Topic Reading Best Practices

### Problem: `head -1` fails with ros2 topic hz

When checking topic publish rates, `ros2 topic hz` outputs to stderr, not stdout. Using `head -1` on stdout fails:

```bash
# FAILS - ros2 topic hz writes to stderr
docker exec landerpi-ros2 bash -c 'ros2 topic hz /aurora/rgb/image_raw' | head -1

# WORKS - redirect stderr to stdout first
docker exec landerpi-ros2 bash -c 'ros2 topic hz /aurora/rgb/image_raw 2>&1' | grep -m 1 "average rate"
```

**Implementation in ros2_hardware.py:**
```python
def check_camera_health(self) -> Tuple[bool, float]:
    """Check if camera is publishing at expected rate."""
    cmd = (
        f'{DOCKER_EXEC} "{ROS_SOURCE} timeout 3 ros2 topic hz /aurora/rgb/image_raw 2>&1" '
        f'| grep -m 1 "average rate"'
    )
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5.0)
    # Parse "average rate: 9.876" from output
```

### Problem: ros2 topic echo initialization overhead

`ros2 topic echo` creates a new ROS2 node each invocation, taking **300-500ms** to initialize. This blocks control loops.

**Solution:** Use caching for non-critical data:
```python
# Cache hazards for 0.5s to avoid blocking control loop
if time.time() - self._last_hazard_time < 0.5:
    return self._cached_hazards
```

See `landerpi-dev` skill for detailed caching patterns.

## Whisplay HAT (Audio/LCD)

The Whisplay HAT (PiSugar) provides audio I/O, LCD display, buttons, and LEDs via GPIO.

### Hardware Overview

| Component | Interface | I2C Address |
|-----------|-----------|-------------|
| WM8960 Audio Codec | I2S/I2C | 0x1a |
| LCD Screen | SPI | - |
| RGB LEDs | GPIO | - |
| Physical Buttons | GPIO | - |

### Expected State After Installation

| Check | Command | Expected |
|-------|---------|----------|
| I2C device | `i2cdetect -y 1` | `1a` visible |
| Audio playback | `aplay -l` | `wm8960` card |
| Audio capture | `arecord -l` | `wm8960` card |
| Kernel module | `lsmod \| grep snd_soc_wm8960` | Module loaded |
| SPI (LCD) | `ls /dev/spidev*` | `/dev/spidev0.0` |

### Installation Notes

- **Requires reboot** after `deploy_whisplay.py install` to load kernel modules
- Uses custom Ubuntu installer (`whisplay/install_wm8960_ubuntu.sh`) since upstream requires raspi-config
- Device tree overlays added to `/boot/firmware/config.txt`: `dtoverlay=wm8960-soundcard`, `dtoverlay=i2s-mmap`

### Troubleshooting

**Problem: WM8960 not detected at 0x1a**
- Check HAT is physically connected
- Verify I2C enabled: `ls /dev/i2c*` should show `/dev/i2c-1`

**Problem: No audio devices after reboot**
- Check kernel module: `lsmod | grep snd_soc_wm8960`
- Check service: `systemctl status wm8960-soundcard.service`
- Check config.txt overlays: `grep wm8960 /boot/firmware/config.txt`
