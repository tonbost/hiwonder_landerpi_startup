# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Remote setup tool for HiWonder LanderPi robot on Raspberry Pi 5. Uses Fabric to execute commands over SSH, with state tracking to resume interrupted deployments.

## Commands

```bash
# Install dependencies
uv sync

# Run comprehensive health check (recommended first)
uv run python checkLanderPi.py check --host <IP> --user <USER> --password <PASS>

# Quick health check (critical systems only)
uv run python checkLanderPi.py quick --host <IP> --user <USER> --password <PASS>

# Test SSH connection to Pi
uv run python setup_landerpi.py connect --host <IP> --user <USER> --password <PASS>

# Deploy full setup (resumes if interrupted)
uv run python setup_landerpi.py deploy --host <IP> --user <USER> --password <PASS>

# Dry run (preview commands without executing)
uv run python setup_landerpi.py deploy --host <IP> --user <USER> --dry-run

# Reset state and start over
uv run python setup_landerpi.py deploy --host <IP> --user <USER> --reset

# Skip Docker/ROS2 installation
uv run python setup_landerpi.py deploy --host <IP> --user <USER> --skip-docker

# Test chassis motion - DIRECT (no ROS2 needed, uses config.json)
uv run python test_chassis_direct.py test --direction all --duration 2 --yes  # Full test (6 directions)
uv run python test_chassis_direct.py test --duration 3 --yes  # Forward + backward only
uv run python test_chassis_direct.py test --direction turn_right --duration 2 --yes
uv run python test_chassis_direct.py test --direction strafe_left --duration 2 --yes

# Test individual motors (for debugging motor mapping)
uv run python test_chassis_direct.py motor-test
uv run python test_chassis_direct.py motor-test --motor 1

# Test chassis motion - via ROS2 (requires HiWonder image)
uv run python test_chassis_motion.py test --host <IP> --user <USER> --password <PASS>

# Emergency stop
uv run python test_chassis_direct.py stop

# Get robot status (battery, serial port)
uv run python test_chassis_direct.py status

# Test lidar - check device, Docker, and ROS2 connectivity
uv run python test_lidar.py check

# Start lidar driver in Docker container
uv run python test_lidar.py start-driver

# Read lidar scan data (requires driver running)
uv run python test_lidar.py scan --samples 5

# Stop lidar driver container
uv run python test_lidar.py stop-driver

# Test lidar modes (robot will MOVE! requires HiWonder ROS2 workspace)
uv run python test_lidar.py test --mode 1 --duration 10 --yes  # Obstacle avoidance
uv run python test_lidar.py test --mode 2 --duration 10 --yes  # Tracking (35cm following)
uv run python test_lidar.py test --mode 3 --duration 10 --yes  # Guard (face objects)

# Stop all lidar functionality
uv run python test_lidar.py stop

# Test robotic arm - DIRECT (no ROS2 needed, uses config.json)
uv run python test_arm.py test --yes  # Full arm test (positions + gripper)
uv run python test_arm.py test --duration 3 --yes  # Custom movement duration

# Get arm servo status (positions, voltages, temperatures)
uv run python test_arm.py status

# Move arm to home position (all servos at 500)
uv run python test_arm.py home --yes

# Test individual servos
uv run python test_arm.py servo-test --servo 1 --yes  # Test servo 1
uv run python test_arm.py servo-test --yes  # Test all servos (1-5, 10)

# Emergency stop (disable all arm servos)
uv run python test_arm.py stop

# Deploy persistent ROS2 stack (uses config.json, survives reboots)
uv run python deploy_ros2_stack.py deploy    # Upload files and start stack
uv run python deploy_ros2_stack.py stop      # Stop the ROS2 stack
uv run python deploy_ros2_stack.py logs      # View container logs
uv run python deploy_ros2_stack.py logs -f   # Follow logs in real-time
```

## Architecture

**`checkLanderPi.py`** - Health check tool using Typer
- `LanderPiChecker` class runs diagnostic checks via SSH
- `HealthReport` dataclass aggregates all check results
- Two modes: `check` (comprehensive) and `quick` (critical only)
- Exit codes: 0 = all OK, 1 = warnings, 2 = errors

**`setup_landerpi.py`** - Main CLI tool using Typer
- `RemoteSetupManager` class handles SSH connections via Fabric
- State tracking: Creates marker files in `~/.landerpi_setup/` on remote Pi
- `is_step_done()`/`mark_step_done()` enable idempotent, resumable deployments

**`test_chassis_direct.py`** - Direct chassis motion test (NO ROS2 REQUIRED)
- Uses `ros_robot_controller_sdk.py` for direct serial communication with STM32
- Speed: 0.3 m/s (max), configurable duration with `--duration`
- Reads credentials from `config.json` automatically
- Commands: `test` (move robot), `motor-test` (individual motors), `stop` (emergency stop), `status` (battery/IMU)
- Motor mapping: M1=front-left, M2=back-left, M3=front-right, M4=back-right
- Forward motion: left side negative speed, right side positive speed

**`test_chassis_motion.py`** - ROS2-based chassis motion test
- `ChassisMotionTest` class handles SSH + ROS2 velocity commands
- Requires deployed ROS2 stack (deploy_ros2_stack.py)
- Uses `/cmd_vel` topic with `geometry_msgs/msg/Twist`
- Commands: `test` (move robot), `stop` (emergency stop)

**`deploy_ros2_stack.py`** - ROS2 stack deployment (persistent across reboots)
- Uploads `ros2_nodes/`, `docker/`, `config/`, `drivers/` to `~/landerpi/` on robot
- Starts Docker Compose stack with `restart: unless-stopped`
- `cmd_vel_bridge` node converts `/cmd_vel` (Twist) to `/ros_robot_controller/set_motor`
- Commands: `deploy` (upload + start), `stop` (stop stack), `logs` (view logs)

**`test_lidar.py`** - Lidar validation test (Docker-based ROS2)
- Tests MS200/LD19 lidar connectivity and functionality via Docker
- ROS2 runs inside Docker container (`ros:humble-ros-base` or `landerpi-ros2:latest`)
- Reads credentials from `config.json` automatically
- Lidar device: `/dev/ldlidar` or `/dev/ttyUSB0` (baud: 230400)
- ROS2 topic: `/scan`
- Commands:
  - `check` - Verify device, Docker, ROS2 availability
  - `start-driver` - Start lidar driver in Docker container
  - `scan` - Read scan data and display statistics
  - `stop-driver` - Stop lidar driver container
  - `test` - Test lidar modes (requires HiWonder ROS2 workspace)
  - `stop` - Stop all lidar functionality

**`test_arm.py`** - Robotic arm test (NO ROS2 REQUIRED)
- Uses `ros_robot_controller_sdk.py` for direct serial communication with STM32
- Arm servos: IDs 1-5 (5-DOF arm) + ID 10 (gripper)
- Position range: 0-1000, with 500 as center/home position
- Reads credentials from `config.json` automatically
- Commands:
  - `test` - Run comprehensive arm test (positions + gripper)
  - `status` - Read all servo positions, voltages, temperatures
  - `home` - Move arm to home position (all servos at 500)
  - `servo-test` - Test individual servos or all servos
  - `stop` - Emergency stop (disable torque on all servos)

**Deployment Steps** (in order):
1. `system_update` - apt update/upgrade, install base tools
2. `hardware_config` - Enable I2C/SPI in `/boot/firmware/config.txt`
3. `peripheral_access` - Configure user groups (dialout, audio, video, i2c, gpio) and vcgencmd
4. `motion_controller` - Upload ros_robot_controller_sdk.py to ~/ros_robot_controller/
5. `camera_setup` - Install USB libs, udev rules, upload and extract Deptrum driver
6. `docker_ros2` - Install Docker, pull `ros:humble-ros-base`, build custom image with camera deps
7. `camera_driver_build` - Build Deptrum ROS2 driver in Docker container
8. `remote_access` - Enable SSH, optionally install VNC

**Peripheral Access** (Step 3 details):
- Adds user to groups: `dialout` (serial), `audio`, `video`, `i2c`, `gpio`
- Creates `/dev/vcio` device for `vcgencmd` temperature monitoring
- Installs udev rule `/etc/udev/rules.d/99-vcio.rules` for persistence

**Motion Controller** (Step 4 details):
- Uploads `ros_robot_controller_sdk.py` to `~/ros_robot_controller/`
- SDK enables direct serial communication with STM32 motor controller
- Used by `test_chassis_direct.py` for motion control without ROS2

**Camera Setup** (Step 5 details):
- Installs libusb-1.0-0-dev and libudev-dev for USB camera access
- Creates udev rule `/etc/udev/rules.d/99-deptrum-libusb.rules` for Deptrum cameras (VID 3251)
- Uploads Deptrum ROS2 driver from `drivers/depth-camera/` to `~/deptrum_ws/`
- Extracts driver and fixes package.xml name mismatch

**Camera Driver Build** (Step 7 details):
- Runs `colcon build` inside Docker container with `-DSTREAM_SDK_TYPE=AURORA930`
- Fixes ownership of build files (Docker creates as root)
- Verifies build succeeded by checking for install directory

**Docker Image** (`landerpi-ros2`):
- Based on `ros:humble-ros-base`
- Includes camera dependencies: cv-bridge, image-transport, angles, tf2-ros, tf2-geometry-msgs, diagnostic-updater
- Includes navigation: navigation2, nav2-bringup, slam-toolbox

**Documentation Files:**
- `DepthCameraHowTo.md` - Aurora 930 depth camera setup guide with Deptrum SDK
- `CameraCheck.md` - Camera status report and troubleshooting
- `LanderPi_Ubuntu_Setup_Guide.md` - Complete manual setup guide with Docker-based ROS2
- `LanderPi_Linux_Command_Reference.md` - Linux command reference for robot setup

**Drivers:**
- `drivers/depth-camera/deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz` - Aurora 930 ROS2 driver (ARM64)

## Key Patterns

- Remote commands use `conn.run()` or `conn.sudo()` with `hide=True`
- `append_to_remote_file()` is idempotent - checks for existing content before appending
- Step completion markers allow safe re-runs without duplicating work
