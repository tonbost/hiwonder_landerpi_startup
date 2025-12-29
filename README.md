# HiWonder LanderPi Setup

Remote setup and control tools for HiWonder LanderPi robot on Raspberry Pi 5.

## Features

- **Remote Deployment** - Fabric-based SSH automation with resumable state tracking
- **Health Checks** - Comprehensive system diagnostics via `checkLanderPi.py`
- **ROS2 Stack** - Docker-based ROS2 Humble with persistent deployment
- **Voice Control** - "Hey TARS" wake word, natural language commands, TTS responses
- **Sensor Testing** - Direct SDK and ROS2-based tests for chassis, arm, lidar, depth camera

## Quick Start

```bash
# Install dependencies
uv sync

# Configure robot connection
cat > config.json << EOF
{
  "host": "192.168.50.169",
  "user": "tonbost",
  "password": "your_password"
}
EOF

# Run health check
uv run python checkLanderPi.py check

# Deploy ROS2 stack (required for voice control)
uv run python deploy_ros2_stack.py deploy

# Deploy voice control
uv run python deploy_voicecontroller.py deploy
```

## Voice Control

TARS is an AI voice assistant that controls the robot using natural language commands.

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

### Supported Commands

| Category | Commands | Description |
|----------|----------|-------------|
| Movement | forward, backward, turn_left, turn_right, strafe_left, strafe_right, stop | Basic motion control |
| Compound | look_around, patrol, come_here | Multi-step sequences |
| Modes | follow_me | Continuous tracking (toggle on/off) |

### Usage

```bash
# Deploy to robot (installs deps, configures service)
uv run python deploy_voicecontroller.py deploy

# Service auto-starts on boot. Manual control:
sudo systemctl start tars-voice    # Start
sudo systemctl stop tars-voice     # Stop
sudo systemctl status tars-voice   # Status
journalctl -u tars-voice -f        # Logs

# Or run manually on robot:
python3 ~/robot_voicecontroller.py loop --wake-word  # Wake word mode
python3 ~/robot_voicecontroller.py loop              # Always listening
python3 ~/robot_voicecontroller.py listen            # Single command
```

### Wake Word Mode

1. Robot announces "TARS online. Say Hey TARS to activate."
2. Say "Hey TARS" - beep confirms activation
3. Give command: "go forward", "look around", "follow me"
4. TARS executes and responds via TTS
5. Returns to listening for wake word

### Prerequisites

- WonderEcho Pro audio module
- AWS credentials configured (`aws configure`)
- ElevenLabs API key in `~/.env`
- ROS2 stack deployed (`deploy_ros2_stack.py deploy`)

## Hardware Tests

### Direct SDK (No ROS2)

```bash
# Chassis
uv run python test_chassis_direct.py test --direction all --duration 2 --yes
uv run python test_chassis_direct.py stop

# Arm
uv run python test_arm.py test --yes
uv run python test_arm.py home --yes

# Lidar
uv run python test_lidar.py check
uv run python test_lidar.py scan --samples 5
```

### ROS2 Stack (Integrated)

```bash
# Deploy stack first
uv run python deploy_ros2_stack.py deploy

# Then test
uv run python test_chassis_motion_ros2.py test --direction forward --yes
uv run python test_arm_ros2.py test --yes
uv run python test_lidar_ros2.py scan --samples 5
uv run python test_cameradepth_ros2.py stream
```

## Project Structure

```
hiwonderSetup/
├── setup_landerpi.py          # Main deployment script
├── checkLanderPi.py           # Health check tool
├── deploy_ros2_stack.py       # ROS2 stack deployment
├── deploy_voicecontroller.py  # Voice control deployment
├── robot_voicecontroller.py   # Voice controller (runs on robot)
├── test_*.py                  # Hardware test scripts
├── config.json                # Robot credentials (git-ignored)
├── systemd/
│   └── tars-voice.service     # Systemd service for voice control
├── ros2_nodes/                # Custom ROS2 nodes
├── docker/                    # Docker configurations
└── drivers/                   # Hardware drivers
```

## Documentation

- `CLAUDE.md` - Detailed architecture and command reference
- `docs/LanderPi_Ubuntu_Setup_Guide.md` - Manual setup guide
- `docs/DepthCameraHowTo.md` - Aurora 930 camera setup
- `docs/armLanderpiHowTo.md` - Arm control reference

## License

MIT
