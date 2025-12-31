# HiWonder LanderPi Setup

Remote setup and control tools for HiWonder LanderPi robot on Raspberry Pi 5.

## Features

- **Remote Deployment** - Fabric-based SSH automation with resumable state tracking
- **Health Checks** - Comprehensive system diagnostics
- **ROS2 Stack** - Docker-based ROS2 Humble with persistent deployment
- **Voice Control** - "Hey TARS" wake word with natural language commands
- **Sensor Testing** - Direct SDK and ROS2-based tests for chassis, arm, lidar, depth camera
- **Autonomous Exploration** - Frontier-based navigation with obstacle avoidance

## Deployment Guide

Follow these steps in order to deploy the full stack on a new LanderPi robot.

### Step 1: Prerequisites (Local Machine)

Install Python dependencies using [uv](https://docs.astral.sh/uv/):

```bash
uv sync
```

Required on your local machine:
- Python 3.11+
- SSH access to the robot's network
- AWS CLI configured (`aws configure`) - for voice control
- ElevenLabs API key - for voice control TTS

### Step 2: Configure Robot Connection

Create a `config.json` file with your robot's credentials:

```bash
cat > config.json << EOF
{
  "host": "192.168.50.169",
  "user": "your_username",
  "password": "your_password"
}
EOF
```

This file is git-ignored and used by all scripts automatically.

### Step 3: Health Check

Run the health check to verify hardware is connected and the robot is accessible:

```bash
# Comprehensive check (all systems)
uv run python checkLanderPi.py check

# Quick check (critical systems only)
uv run python checkLanderPi.py quick
```

The health check verifies:
- SSH connectivity
- Serial port (`/dev/ttyTHS0`) for motor controller
- I2C and SPI interfaces
- Lidar device (`/dev/ttyUSB*`)
- Depth camera (USB VID 3251)
- Audio devices (WonderEcho Pro)
- CPU temperature and memory

**Exit codes:** 0 = all OK, 1 = warnings, 2 = errors

Fix any hardware issues before proceeding.

### Step 4: Initial Setup

Run the main setup script to configure the robot's software environment:

```bash
# Test SSH connection first
uv run python setup_landerpi.py connect

# Deploy full setup (resumes if interrupted)
uv run python setup_landerpi.py deploy

# Optional: Dry run to preview commands
uv run python setup_landerpi.py deploy --dry-run

# Optional: Skip Docker/ROS2 if already installed
uv run python setup_landerpi.py deploy --skip-docker

# Reset state and start fresh
uv run python setup_landerpi.py deploy --reset
```

The setup script performs these steps in order:
1. **system_update** - apt update/upgrade, install base tools
2. **hardware_config** - Enable I2C/SPI in `/boot/firmware/config.txt`
3. **peripheral_access** - Configure user groups (dialout, audio, video, i2c, gpio)
4. **motion_controller** - Upload motor SDK to `~/ros_robot_controller/`
5. **camera_setup** - Install USB libs, udev rules, upload Deptrum driver
6. **docker_ros2** - Install Docker, pull ROS2 image, build custom image
7. **camera_driver_build** - Build Deptrum ROS2 driver in Docker
8. **remote_access** - Enable SSH, optionally install VNC

State tracking allows safe re-runs without duplicating work.

### Step 5: Validate Hardware (Direct SDK)

Test hardware components directly without ROS2:

```bash
# Chassis motion test (robot will MOVE!)
uv run python validation/test_chassis_direct.py test --direction all --duration 2 --yes
uv run python validation/test_chassis_direct.py stop  # Emergency stop

# Arm test
uv run python validation/test_arm.py test --yes
uv run python validation/test_arm.py home --yes

# Lidar check
uv run python validation/test_lidar.py check
uv run python validation/test_lidar.py scan --samples 5
```

### Step 6: Deploy ROS2 Stack

Deploy the persistent ROS2 stack for integrated sensor control:

```bash
# Deploy stack (uploads files, starts Docker Compose)
uv run python deploy_ros2_stack.py deploy

# Manage the stack
uv run python deploy_ros2_stack.py stop   # Stop stack
uv run python deploy_ros2_stack.py logs   # View logs
uv run python deploy_ros2_stack.py logs -f  # Follow logs
```

The ROS2 stack:
- Runs in Docker with `restart: unless-stopped` (survives reboots)
- Includes `cmd_vel_bridge` node for velocity commands
- Publishes lidar data on `/scan`, camera on `/aurora/*`
- Includes `depth_stats` node for processed depth data on `/depth_stats`

### Step 7: Validate Hardware (ROS2)

Test hardware through the ROS2 stack:

```bash
# Chassis via ROS2
uv run python validation/test_chassis_motion_ros2.py test --direction forward --distance 0.5 --yes

# Arm via ROS2
uv run python validation/test_arm_ros2.py test --yes
uv run python validation/test_arm_ros2.py status

# Lidar via ROS2
uv run python validation/test_lidar_ros2.py check
uv run python validation/test_lidar_ros2.py scan --samples 5

# Depth camera via ROS2
uv run python validation/test_cameradepth_ros2.py check
uv run python validation/test_cameradepth_ros2.py stream
```

### Step 8: Deploy Voice Control (Optional)

Deploy TARS voice assistant for natural language robot control:

```bash
# Full deployment (script + deps + systemd service)
uv run python deploy_voicecontroller.py deploy

# Service auto-starts on boot. Manual control:
sudo systemctl start tars-voice
sudo systemctl stop tars-voice
sudo systemctl status tars-voice
journalctl -u tars-voice -f  # View logs
```

Prerequisites:
- WonderEcho Pro audio module connected
- AWS credentials on robot (`aws configure`)
- ElevenLabs API key in `~/.env` on robot

---

## Voice Control

TARS is an AI voice assistant that controls the robot using natural language.

### Architecture

```
WonderEcho Pro (mic) → faster-whisper (ASR) → Bedrock Claude Haiku (LLM)
                                                        ↓
                                               JSON command
                                                        ↓
                    docker exec landerpi-ros2 ros2 topic pub /controller/cmd_vel
                                                        ↓
                              ElevenLabs TTS → WonderEcho Pro (speaker)
```

### Wake Word Mode

1. Robot announces "TARS online. Say Hey TARS to activate."
2. Say "Hey TARS" - beep confirms activation
3. Give command: "go forward", "look around", "follow me"
4. TARS executes and responds via TTS
5. Returns to listening for wake word

### Supported Commands

| Category | Commands | Description |
|----------|----------|-------------|
| Movement | forward, backward, turn_left, turn_right, strafe_left, strafe_right, stop | Basic motion |
| Compound | look_around, patrol, come_here | Multi-step sequences |
| Modes | follow_me | Continuous tracking (toggle) |

### Manual Usage (on robot)

```bash
python3 ~/robot_voicecontroller.py loop --wake-word  # Wake word mode
python3 ~/robot_voicecontroller.py loop              # Always listening
python3 ~/robot_voicecontroller.py listen            # Single command
python3 ~/robot_voicecontroller.py check             # Check components
python3 ~/robot_voicecontroller.py test-tts "Hello"  # Test TTS
```

---

## Autonomous Exploration

Frontier-based exploration using lidar for obstacle avoidance.

```bash
# Check prerequisites
uv run python validation/test_exploration.py status

# Start exploration (robot will MOVE!)
uv run python validation/test_exploration.py start --yes
uv run python validation/test_exploration.py start --duration 5 --yes  # 5 minutes
uv run python validation/test_exploration.py start --rosbag --yes      # With recording

# Emergency stop
uv run python validation/test_exploration.py stop
```

| Flag | Description | Default |
|------|-------------|---------|
| `--duration` | Max runtime (minutes) | 30 |
| `--min-battery` | Battery cutoff voltage | 6.6V |
| `--rosbag` | Enable ROS2 bag recording | Off |

---

## Project Structure

```
hiwonderSetup/
├── checkLanderPi.py           # Step 3: Health check tool
├── setup_landerpi.py          # Step 4: Main deployment script
├── deploy_ros2_stack.py       # Step 6: ROS2 stack deployment
├── deploy_voicecontroller.py  # Step 8: Voice control deployment
├── robot_voicecontroller.py   # Voice controller (runs on robot)
├── config.json                # Robot credentials (git-ignored)
├── validation/
│   ├── test_chassis_direct.py # Direct motor SDK test
│   ├── test_arm.py            # Direct arm SDK test
│   ├── test_lidar.py          # Lidar test
│   ├── test_*_ros2.py         # ROS2-based tests
│   ├── test_exploration.py    # Autonomous exploration
│   └── exploration/           # Exploration module
├── ros2_nodes/                # Custom ROS2 nodes
├── docker/                    # Docker configurations
├── drivers/                   # Hardware drivers (depth camera)
├── systemd/                   # Systemd service files
└── docs/                      # Additional documentation
```

## Documentation

- `CLAUDE.md` - Detailed architecture and full command reference
- `docs/LanderPi_Ubuntu_Setup_Guide.md` - Manual setup guide
- `docs/DepthCameraHowTo.md` - Aurora 930 camera setup
- `docs/armLanderpiHowTo.md` - Arm control reference

## Troubleshooting

**Health check fails:**
- Verify robot is powered on and connected to network
- Check `config.json` has correct IP and credentials
- Ensure hardware is properly connected (serial, USB, I2C)

**Setup script hangs:**
- Run with `--dry-run` to preview commands
- Use `--reset` to clear state and restart
- Check robot has internet access for apt/Docker pulls

**ROS2 stack won't start:**
- Verify Docker is installed: `docker --version`
- Check container logs: `uv run python deploy_ros2_stack.py logs`
- Ensure ports aren't in use: `docker ps`

**Voice control issues:**
- Run `python3 ~/robot_voicecontroller.py check` on robot
- Verify audio: `arecord -l` and `aplay -l`
- Check AWS credentials: `aws sts get-caller-identity`

## License

MIT
