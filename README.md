# Remote Hiwonder LanderPi Setup Walkthrough

This guide explains how to use the `landerpi-setup` tool to remotely provision a Raspberry Pi 5 from your local computer.

## Prerequisites

1. **Local Machine**:
    - Python 3.10+ installed.
    - `uv` installed (recommended) or `pip`.
    - Network access to the Raspberry Pi.

2. **Raspberry Pi**:
    - Powered on and connected to the network.
    - **SSH Enabled**: If using a fresh install, ensure you enabled SSH in the Raspberry Pi Imager options.
    - You must know the IP address (e.g., `192.168.1.100`) and the username.
    - Connection details can be stored in `config.json` (see below).

## Installation (Local)

1. **Install Dependencies**:
   ```bash
   cd hiwonderSetup
   uv sync
   # Or with pip: pip install -r requirements.txt
   ```

2. **Configure Connection** (optional):
   Create a `config.json` file with your robot's connection details:
   ```json
   {
     "host": "<PI_IP>",
     "user": "<USERNAME>",
     "password": "<PASSWORD>"
   }
   ```
   This file is git-ignored to protect credentials.

## Usage

### 1. Health Check (Recommended First Step)
Run a comprehensive health check to verify all peripherals and system status:
```bash
# Full health check
uv run python checkLanderPi.py check --host <PI_IP> --user <USERNAME> --password <PASSWORD>

# Quick check (connection + critical systems only)
uv run python checkLanderPi.py quick --host <PI_IP> --user <USERNAME> --password <PASSWORD>
```

The health check verifies:
- System info (temperature, memory, disk, CPU load)
- User group memberships (dialout, audio, video, docker)
- USB devices (lidar, serial adapters, audio)
- Serial port accessibility
- Audio (ALSA and PipeWire)
- I2C/SPI interfaces
- GPIO chips
- Network connectivity
- Docker status and images
- ROS2 workspace

### 2. Chassis Motion Test (Direct Serial - No ROS2 Required)
Test robot movement using direct serial communication with the STM32 controller:
```bash
# Full test suite - all 6 directions (2 seconds each)
uv run python test_chassis_direct.py test --direction all --duration 2 --yes

# Test forward and backward only
uv run python test_chassis_direct.py test --direction both --duration 3 --yes

# Test specific direction
uv run python test_chassis_direct.py test --direction forward --duration 3 --yes
uv run python test_chassis_direct.py test --direction turn_right --duration 2 --yes
uv run python test_chassis_direct.py test --direction strafe_left --duration 2 --yes

# Test individual motors (for debugging)
uv run python test_chassis_direct.py motor-test
uv run python test_chassis_direct.py motor-test --motor 1

# Get robot status (battery voltage)
uv run python test_chassis_direct.py status

# Emergency stop
uv run python test_chassis_direct.py stop
```

**Directions:**
- `forward` / `backward` - Linear movement
- `turn_right` / `turn_left` - Rotate in place
- `strafe_right` / `strafe_left` - Lateral movement (mecanum)
- `both` - Forward + backward (default)
- `all` - Full test suite (all 6 directions)

**Options:**
- `--distance`: Distance to move in meters (max 0.20, default 0.10)
- `--direction`: Movement direction (see above)
- `--duration`: Override duration in seconds (ignores distance calculation)
- `-y, --yes`: Skip approval prompt (use with caution)

**Safety features:**
- Maximum movement capped at 20cm (unless using --duration override)
- Requires user approval before moving
- Automatic stop after each movement
- Emergency stop command available

**Motor Mapping (Mecanum Chassis):**
- M1 = front-left, M2 = back-left, M3 = front-right, M4 = back-right
- Forward: left side negative speed, right side positive speed
- Uses `ros_robot_controller_sdk.py` for direct STM32 communication via `/dev/ttyACM0`

**Note:** This test uses direct serial communication and does NOT require ROS2 or the HiWonder system image.

### 2b. Chassis Motion Test (ROS2 - Requires HiWonder Image)
If you have the HiWonder system image with ROS2 installed:
```bash
# Test forward and backward
uv run python test_chassis_motion.py test --host <PI_IP> --user <USERNAME> --password <PASSWORD>

# Emergency stop
uv run python test_chassis_motion.py stop --host <PI_IP> --user <USERNAME> --password <PASSWORD>
```

**Prerequisites:**
- ROS2 controller must be running on robot:
  ```bash
  ros2 launch ros_robot_controller ros_robot_controller.launch.py
  ```

### 3. Lidar Test (Docker-based ROS2)
Test MS200/LD19 lidar connectivity and functionality. ROS2 runs inside Docker containers.

```bash
# Check lidar device, Docker, and ROS2 availability
uv run python test_lidar.py check

# Start lidar driver in Docker container (publishes to /scan topic)
uv run python test_lidar.py start-driver

# Read lidar scan data and display statistics
uv run python test_lidar.py scan --samples 5

# Stop the lidar driver container
uv run python test_lidar.py stop-driver

# Stop all lidar functionality
uv run python test_lidar.py stop
```

**Check Output Example:**
```
Summary:
  Docker: PASS
  Lidar Device: PASS
  ROS2 in Docker: PASS
  Scan Topic: WARN (start driver first)
```

**Lidar Modes (Requires HiWonder ROS2 Workspace):**
If the full HiWonder ROS2 workspace is installed, additional modes are available:
```bash
# Obstacle avoidance - robot turns to avoid obstacles
uv run python test_lidar.py test --mode 1 --duration 10 --yes

# Tracking - robot follows objects at 35cm distance
uv run python test_lidar.py test --mode 2 --duration 10 --yes

# Guard - robot rotates to face detected objects
uv run python test_lidar.py test --mode 3 --duration 10 --yes
```

**Lidar Configuration:**
| Property | Value |
|----------|-------|
| Device | `/dev/ldlidar` or `/dev/ttyUSB0` |
| Baud Rate | 230400 |
| ROS2 Topic | `/scan` |
| Docker Image | `landerpi-ros2:latest` or `ros:humble-ros-base` |

**Note:** ROS2 is NOT installed natively on the robot. All ROS2 commands run inside Docker containers with the pattern:
```bash
docker run --rm --privileged --network host -v /dev:/dev <IMAGE> \
    bash -c 'source /opt/ros/humble/setup.bash && <ROS2_COMMAND>'
```

### 4. Robotic Arm Test (Direct Serial - No ROS2 Required)
Test the 5-DOF robotic arm and gripper using direct serial communication:

```bash
# Full arm test (home position + gripper + arm movements)
uv run python test_arm.py test --yes

# Get servo status (positions, voltages, temperatures)
uv run python test_arm.py status

# Move arm to home position (all servos at 500)
uv run python test_arm.py home --yes

# Test individual servo
uv run python test_arm.py servo-test --servo 1 --yes

# Test all servos sequentially
uv run python test_arm.py servo-test --yes

# Emergency stop (disable all servo torque)
uv run python test_arm.py stop
```

**Servo Configuration:**
| Servo ID | Function | Position Range |
|----------|----------|----------------|
| 1 | Base rotation (Joint 1) | 0-1000 |
| 2 | Shoulder (Joint 2) | 0-1000 |
| 3 | Elbow (Joint 3) | 0-1000 |
| 4 | Wrist pitch (Joint 4) | 0-1000 |
| 5 | Wrist roll (Joint 5) | 0-1000 |
| 10 | Gripper | 0-1000 (200=open, 700=closed) |

**Options:**
- `--duration`: Movement duration in seconds (default: 2.0)
- `--servo`: Test specific servo (1-5 for arm, 10 for gripper)
- `-y, --yes`: Skip approval prompt

**Safety Features:**
- Requires user approval before moving
- Audio feedback (buzzer) during operations
- Emergency stop command available
- Torque disable on stop

**Status Output Example:**
```
┌──────────────────────────────────────────────────────────┐
│                    Arm Servo Status                       │
├──────────┬─────────┬──────────┬─────────┬───────────────┤
│ Servo ID │  Type   │ Position │ Voltage │  Temperature  │
├──────────┼─────────┼──────────┼─────────┼───────────────┤
│    1     │ Joint 1 │   500    │  8.0V   │     24C       │
│    2     │ Joint 2 │   500    │  7.9V   │     23C       │
│    3     │ Joint 3 │   500    │  8.0V   │     24C       │
│    4     │ Joint 4 │   500    │  7.9V   │     25C       │
│    5     │ Joint 5 │   500    │  8.0V   │     22C       │
│   10     │ Gripper │   500    │  8.0V   │     24C       │
└──────────┴─────────┴──────────┴─────────┴───────────────┘
```

**Note:** Uses `ros_robot_controller_sdk.py` for direct STM32 communication. No ROS2 required.

### 5. Test Connection (Optional)
Verify SSH connectivity only:
```bash
uv run python setup_landerpi.py connect --host <PI_IP> --user <USERNAME> --password <PASSWORD>
```
*Tip: If you use SSH keys, you can omit the password.*

### 6. Run Setup (with Resume capability)
The script tracks progress on the remote Pi in `~/.landerpi_setup_state.json`. If it fails or disconnects, simply run the **exact same command** again. It will detect completed steps and skip them.

```bash
uv run python setup_landerpi.py deploy --host 192.168.1.100 --user pi
```

### Options
- `--dry-run`: Print commands without executing them on the remote host.
- `--reset`: Ignore previous progress and force re-run of all steps (careful!).
- `--skip-docker`: Skip Docker installation.

## What it Does (Remote Steps)
1. **System Updates**: `apt update/upgrade`, installs essential tools.
2. **Hardware Config**: Modifies `/boot/firmware/config.txt` to enable I2C/SPI.
3. **Peripheral Access**: Configures user permissions for robot hardware:
   - Adds user to `dialout` group (serial ports for lidar, motor controllers)
   - Adds user to `audio` group (USB speakers)
   - Adds user to `video` group (cameras, vcgencmd)
   - Adds user to `i2c` and `gpio` groups (sensor access)
   - Creates `/dev/vcio` device and udev rule for `vcgencmd` temperature monitoring
4. **Camera Setup**: Uploads and extracts Deptrum ROS2 driver for Aurora 930.
5. **Docker**: Installs Docker Engine, configures permissions, builds `landerpi-ros2` container.
6. **Camera Driver Build**: Builds Deptrum ROS2 driver inside Docker container.
7. **Remote Access**: Enables SSH, optionally installs VNC server.

## Supported Peripherals
After setup, the following peripherals will be accessible:
- **Lidar** (MS200/LD19): `/dev/ldlidar` or `/dev/ttyUSB0` - 230400 baud, tested via `test_lidar.py`
- **Depth Camera** (Aurora 930): USB device (VID:PID 3251:1930) - Fully configured with ROS2 driver
- **Motor Controller** (STM32): `/dev/ttyACM0` - Direct serial communication via SDK
- **Robotic Arm** (5-DOF + Gripper): Servo IDs 1-5 (arm) + 10 (gripper), tested via `test_arm.py`
- **Serial Controllers** (CH340): `/dev/ttyUSB0`, `/dev/ttyUSB1`
- **USB Audio** (Speaker): Detected via PipeWire/ALSA
- **I2C Devices**: `/dev/i2c-1`
- **SPI Devices**: `/dev/spidev0.0`, `/dev/spidev0.1`
- **GPIO**: `/dev/gpiochip0`
- **Temperature**: `vcgencmd measure_temp`
- **Docker**: ROS2 runs inside `landerpi-ros2` or `ros:humble-ros-base` containers

## Aurora 930 Depth Camera Setup

The Aurora 930 depth camera is **automatically configured** during `deploy`. The setup script:
1. Uploads the Deptrum ROS2 driver from `drivers/depth-camera/`
2. Extracts and fixes the package.xml
3. Builds the driver inside Docker

After setup completes, launch the camera with:
```bash
docker run --rm -it --privileged -v /dev:/dev -v ~/deptrum_ws:/deptrum_ws landerpi-ros2:latest \
    bash -c "source /opt/ros/humble/setup.bash && source /deptrum_ws/install/local_setup.bash && \
             ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py"
```

See [DepthCameraHowTo.md](DepthCameraHowTo.md) for detailed information and troubleshooting.

### Manual Setup (if needed)
```bash
# 1. Transfer driver to Pi (from this repo directory)
scp drivers/depth-camera/deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz user@pi:~/

# 2. On Pi: Create workspace and extract
mkdir -p ~/deptrum_ws/src
tar -xzvf deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz -C ~/deptrum_ws/src/

# 3. Fix package name
sed -i 's/<name>deptrum-ros-driver<\/name>/<name>deptrum-ros-driver-aurora930<\/name>/' \
    ~/deptrum_ws/src/deptrum-ros-driver-aurora930-0.2.1001/package.xml

# 4. Setup udev rules
cd ~/deptrum_ws/src/deptrum-ros-driver-aurora930-0.2.1001/ext/deptrum-stream-aurora900-linux-aarch64-v1.1.19-18.04/scripts
sudo bash setup_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger

# 5. Build in Docker
docker run --rm -v ~/deptrum_ws:/deptrum_ws -w /deptrum_ws landerpi-ros2:latest \
    bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DSTREAM_SDK_TYPE=AURORA930"

# 6. Launch camera
docker run --rm -it --privileged -v /dev:/dev -v ~/deptrum_ws:/deptrum_ws landerpi-ros2:latest \
    bash -c "source /opt/ros/humble/setup.bash && source /deptrum_ws/install/local_setup.bash && \
             ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py"
```

## Post-Setup
After the script completes:
1. **Reboot is required** for group changes to take effect:
   ```bash
   ssh pi@<PI_IP> sudo reboot
   ```
2. Re-connect and verify peripherals:
   ```bash
   # Check groups are active
   groups
   # Should include: dialout audio video i2c gpio docker

   # Test serial ports
   stty -F /dev/ttyACM0
   stty -F /dev/ttyUSB0

   # Test audio
   aplay -l

   # Test temperature
   vcgencmd measure_temp

   # Test Docker
   docker ps
   ```

## Troubleshooting

### Serial port permission denied
If you get "Permission denied" on `/dev/ttyUSB*` or `/dev/ttyACM*`:
```bash
# Verify group membership
groups | grep dialout

# If missing, re-run setup or manually add:
sudo usermod -aG dialout $USER
# Then logout/login or reboot
```

### Audio not detected by PipeWire
If `wpctl status` shows only "Dummy Output":
```bash
# Check ALSA detects the device
aplay -l

# If ALSA sees it but PipeWire doesn't, restart audio services:
systemctl --user restart pipewire wireplumber
```

### vcgencmd: Can't open device file
```bash
# Create the device manually
sudo mknod /dev/vcio c 100 0
sudo chmod 666 /dev/vcio

# Make it persistent
echo 'KERNEL=="vcio", MODE="0666"' | sudo tee /etc/udev/rules.d/99-vcio.rules
```
