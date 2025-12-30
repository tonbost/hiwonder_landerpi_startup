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

# Test chassis motion - via ROS2 (requires deployed stack, uses config.json)
uv run python test_chassis_motion_ros2.py test --direction all --distance 1.0 --yes  # Full test (all 6 movements)
uv run python test_chassis_motion_ros2.py test --direction forward --distance 0.5 --yes
uv run python test_chassis_motion_ros2.py test --direction strafe_left --distance 0.5 --yes
uv run python test_chassis_motion_ros2.py test --direction turn_right --yes

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

# Test sensors via ROS2 stack (requires deploy_ros2_stack.py deploy)
uv run python test_arm_ros2.py test --yes        # Arm via ROS2 topics
uv run python test_arm_ros2.py home --yes        # Home position
uv run python test_arm_ros2.py status            # Arm state
uv run python test_lidar_ros2.py check           # Lidar status
uv run python test_lidar_ros2.py scan --samples 5  # Read scan data
uv run python test_cameradepth_ros2.py check     # Camera topics
uv run python test_cameradepth_ros2.py stream    # Read streams

# Voice Control (runs ON the robot, requires WonderEcho Pro + ROS2 stack)
uv run python deploy_voicecontroller.py deploy   # Full deployment (script + deps + service)
uv run python deploy_voicecontroller.py upload   # Upload script only
uv run python deploy_voicecontroller.py install-service  # Install systemd service
uv run python deploy_voicecontroller.py test     # Run check on robot

# Systemd service management (auto-starts on boot):
# sudo systemctl start tars-voice    # Start voice control
# sudo systemctl stop tars-voice     # Stop voice control
# sudo systemctl status tars-voice   # Check status
# journalctl -u tars-voice -f        # View logs

# On the robot directly:
# python3 ~/robot_voicecontroller.py check       # Check components
# python3 ~/robot_voicecontroller.py listen      # Single voice command
# python3 ~/robot_voicecontroller.py loop        # Continuous mode (always listening)
# python3 ~/robot_voicecontroller.py loop --wake-word  # Wake word mode ("Hey TARS")
# python3 ~/robot_voicecontroller.py test-tts "Hello"  # Test TTS
# python3 ~/robot_voicecontroller.py test-llm "go forward"  # Test LLM command generation

# Autonomous Exploration (uses config.json)
uv run python validation/test_exploration.py start --yes        # Start exploration (30 min)
uv run python validation/test_exploration.py start --duration 60 --yes  # 60 min exploration
uv run python validation/test_exploration.py start --rosbag --yes       # With ROS2 bag recording
uv run python validation/test_exploration.py stop               # Stop exploration
uv run python validation/test_exploration.py status             # Check robot status
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

**`test_arm_ros2.py`** - ROS2-based arm test (requires deployed stack)
- Uses `/arm/cmd` and `/arm/state` topics via `docker exec landerpi-ros2`
- Commands: `test`, `home`, `stop`, `status`

**`test_lidar_ros2.py`** - ROS2-based lidar test (requires deployed stack)
- Uses `/scan` topic via `docker exec landerpi-ros2`
- Commands: `check`, `scan`, `status`

**`test_cameradepth_ros2.py`** - ROS2-based camera test (requires deployed stack)
- Uses `/aurora/*` topics via `docker exec landerpi-ros2`
- Commands: `check`, `stream`, `status`

**`ros2_nodes/arm_controller/`** - ROS2 arm controller node
- Subscribes to `/arm/cmd` (JSON commands)
- Publishes to `/arm/state` (JSON state)
- Supports: set_position, home, stop, gripper actions

**`ros2_nodes/lidar_driver/`** - ROS2 lidar driver node
- Publishes to `/scan` (sensor_msgs/LaserScan)
- Reads LD19/MS200 serial protocol at 230400 baud
- Accumulates 360° scan data before publishing

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

**Voice Control System:**

**`robot_voicecontroller.py`** - Voice controller (runs ON the robot)
- Uses WonderEcho Pro for audio I/O (card 1, stereo 48kHz)
- Local ASR via faster-whisper (base.en model)
- AWS Bedrock Claude Haiku 4.5 for command generation
- ElevenLabs Flash v2.5 for TTS response
- ROS2-based motor control via `docker exec landerpi-ros2` (requires deployed ROS2 stack)
- Wake word detection: "Hey TARS" activation with beep confirmation
- Startup announcement via TTS when service starts
- Commands: `listen` (single), `loop` (continuous), `loop --wake-word`, `check`, `test-tts`, `test-llm`

**Voice Commands Supported:**
- Movement: `forward`, `backward`, `turn_left`, `turn_right`, `strafe_left`, `strafe_right`, `stop`
- Compound: `look_around` (360° scan), `patrol` (square pattern), `come_here` (approach user)
- Modes: `follow_me` (continuous tracking - toggle on/off)
- Parameters: speed (0.1-0.5 m/s), duration (0.5-10.0 seconds)

**`deploy_voicecontroller.py`** - Deployment tool (runs locally)
- Uploads script and installs deps on robot via SSH
- Configures AWS credentials and ElevenLabs API key
- Installs systemd service (`tars-voice.service`) for auto-start on boot
- Generates activation beep sound using sox
- Commands: `deploy`, `upload`, `install`, `configure`, `install-service`, `test`

**`systemd/tars-voice.service`** - Systemd service for boot persistence
- Starts voice controller in wake word mode after boot
- Depends on network and Docker service
- Auto-restarts on failure

**Voice Control Architecture (all on robot):**
```
WonderEcho Pro (arecord) → faster-whisper (ASR) → Bedrock Haiku (LLM)
                                                         ↓
                                                 JSON command
                                                         ↓
                         docker exec landerpi-ros2 ros2 topic pub /controller/cmd_vel
                                                         ↓
                         ElevenLabs TTS → WonderEcho Pro (aplay)
```

**AWS IAM Policy** (`aws/landerpi-bedrock-policy.json`):
- BedrockInvokeOnly: Minimal permissions for robot
- Only allows `bedrock:InvokeModel` on Claude models

**`validation/exploration/`** - Autonomous exploration package
- `explorer.py` - Main exploration controller
- `sensor_fusion.py` - Depth camera + lidar fusion
- `frontier_planner.py` - Direction selection based on freshness
- `safety_monitor.py` - Battery and runtime monitoring
- `data_logger.py` - Logging for future SLAM

**`validation/test_exploration.py`** - Exploration CLI
- Start/stop autonomous exploration
- Sensor fusion: depth camera (20cm stop) + lidar (360° awareness)
- Frontier-based exploration (seeks unexplored directions)
- Safety: runtime limit + battery cutoff

## Key Patterns

- Remote commands use `conn.run()` or `conn.sudo()` with `hide=True`
- `append_to_remote_file()` is idempotent - checks for existing content before appending
- Step completion markers allow safe re-runs without duplicating work
