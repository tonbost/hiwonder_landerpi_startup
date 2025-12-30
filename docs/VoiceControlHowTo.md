# Voice Control HowTo - TARS for LanderPi

TARS is an AI voice assistant that controls the LanderPi robot using natural language commands. It runs entirely on the robot with local speech recognition, cloud-based LLM processing, and cloud-based text-to-speech.

## Overview

**Date**: 2024-12-29 (initial implementation)

### Architecture

```
                          ┌──────────────────────────────────────────────────────────┐
                          │                    On the Robot                           │
                          ├──────────────────────────────────────────────────────────┤
                          │                                                          │
 User speaks  ──────────► │  WonderEcho Pro (arecord)                               │
                          │         │                                                │
                          │         ▼                                                │
                          │  faster-whisper (local ASR)  ◄── base.en model          │
                          │         │                                                │
                          │         ▼                                                │
                          │  AWS Bedrock Claude Haiku 4.5  ◄── JSON command gen     │
                          │         │                                                │
                          │         ▼                                                │
                          │  docker exec landerpi-ros2  ◄── ROS2 /controller/cmd_vel│
                          │         │                                                │
                          │         ▼                                                │
 User hears   ◄────────── │  ElevenLabs TTS (streaming)  ──► WonderEcho Pro speaker │
                          │                                                          │
                          └──────────────────────────────────────────────────────────┘
```

## What's Possible Today

### Movement Commands

| Voice Command | Action | Parameters |
|---------------|--------|------------|
| "go forward" / "move ahead" | Move forward | speed: 0.3 m/s, duration: 2s |
| "go back" / "reverse" | Move backward | speed: 0.3 m/s, duration: 2s |
| "turn left" / "rotate left" | Rotate counter-clockwise | angular: 0.5 rad/s |
| "turn right" / "spin right" | Rotate clockwise | angular: -0.5 rad/s |
| "strafe left" / "move left" | Slide sideways left | speed: 0.3 m/s |
| "strafe right" / "move right" | Slide sideways right | speed: 0.3 m/s |
| "stop" / "halt" / "freeze" | Emergency stop | Immediate |

**Speed modifiers**: "slowly" → 0.15 m/s, "fast" → 0.5 m/s

### Compound Commands

| Voice Command | Behavior | Duration |
|---------------|----------|----------|
| "look around" / "scan" | 360-degree rotation | ~7 seconds |
| "patrol" / "patrol the area" | Square pattern (4 legs) | ~30 seconds |
| "come here" / "come to me" | Move forward toward user | ~3 seconds |

### Mode Commands

| Voice Command | Behavior |
|---------------|----------|
| "follow me" | Start continuous following (slow forward motion) |
| "stop following" | Disable follow mode |

### Wake Word

- **Activation phrase**: "Hey TARS"
- Robot confirms with a beep sound
- After activation, give your command
- Returns to listening for wake word after command execution

### Exit Commands

- "stop listening"
- "goodbye"
- "exit"
- Ctrl+C (keyboard interrupt)

## What's NOT Possible Yet

The following features are planned but not yet implemented:

| Feature | Description | Status |
|---------|-------------|--------|
| **Arm Control via Voice** | "wave", "pick up", "put down" | Prompt defined, execution not implemented |
| **SLAM Integration** | Patrol with map building and obstacle avoidance | Future |
| **Object Detection** | "find the red ball", "go to the chair" | Requires vision pipeline |
| **Sound Localization** | "come here" uses sound angle from WonderEcho Pro | Future |
| **Multi-language** | Commands in languages other than English | Future |
| **Lidar Modes** | "avoid obstacles", "track me", "guard" | Available via CLI, not voice |
| **Camera Commands** | "show me what you see", "stream video" | Future |
| **Navigation** | "go to kitchen", "remember this location" | Requires nav2 integration |

## Prerequisites

### Hardware

- HiWonder LanderPi robot with Raspberry Pi 5
- WonderEcho Pro audio module (USB audio)
- Network connectivity (WiFi or Ethernet)

### Software (on robot)

- ROS2 stack deployed (`deploy_ros2_stack.py deploy`)
- Docker running with `landerpi-ros2` container
- Python 3.11+ with required packages:
  - `faster-whisper` (local ASR)
  - `boto3` (AWS SDK)
  - `elevenlabs` (TTS)
  - `typer`, `rich` (CLI)
  - `ffmpeg` (audio conversion)

### Cloud Services

| Service | Purpose | Configuration |
|---------|---------|---------------|
| AWS Bedrock | LLM (Claude Haiku 4.5) | `aws configure` on robot |
| ElevenLabs | Text-to-Speech | `ELEVENLABS_API_KEY` in `~/.env` |

### AWS IAM Policy

Create a minimal policy for the robot (`aws/landerpi-bedrock-policy.json`):

```json
{
    "Version": "2012-10-17",
    "Statement": [{
        "Effect": "Allow",
        "Action": "bedrock:InvokeModel",
        "Resource": "arn:aws:bedrock:*::foundation-model/anthropic.*"
    }]
}
```

## Setup Instructions

### 1. Deploy ROS2 Stack First

Voice control requires the ROS2 stack for motor control:

```bash
# From your development machine
uv run python deploy_ros2_stack.py deploy
```

Verify it's running:

```bash
ssh user@robot "docker ps | grep landerpi-ros2"
```

### 2. Deploy Voice Controller

```bash
# Full deployment (script + deps + service)
uv run python deploy_voicecontroller.py deploy
```

Or step-by-step:

```bash
# Upload script only
uv run python deploy_voicecontroller.py upload

# Install dependencies
uv run python deploy_voicecontroller.py install

# Configure credentials
uv run python deploy_voicecontroller.py configure

# Install systemd service
uv run python deploy_voicecontroller.py install-service
```

### 3. Configure AWS Credentials (on robot)

```bash
ssh user@robot
aws configure
# Enter Access Key, Secret Key, Region (us-east-1)
```

### 4. Configure ElevenLabs (on robot)

```bash
echo 'ELEVENLABS_API_KEY=your_api_key_here' >> ~/.env
echo 'ELEVENLABS_VOICE_ID=tTV9ysSyKVtziN1ESbZV' >> ~/.env  # Optional, uses default
```

### 5. Generate Activation Sound (optional)

```bash
# On robot, with sox installed
sudo apt install sox
sox -n ~/audio/beep.wav synth 0.1 sine 880 vol 0.5
```

## Usage

### Systemd Service (Recommended)

The voice controller auto-starts on boot as a systemd service:

```bash
# Start
sudo systemctl start tars-voice

# Stop
sudo systemctl stop tars-voice

# Restart
sudo systemctl restart tars-voice

# Check status
sudo systemctl status tars-voice

# View logs
journalctl -u tars-voice -f

# Enable/disable auto-start
sudo systemctl enable tars-voice
sudo systemctl disable tars-voice
```

### Manual Operation (on robot)

```bash
# Wake word mode (recommended)
python3 ~/robot_voicecontroller.py loop --wake-word

# Always listening mode (no wake word)
python3 ~/robot_voicecontroller.py loop

# Single command
python3 ~/robot_voicecontroller.py listen

# Component check
python3 ~/robot_voicecontroller.py check

# Test TTS
python3 ~/robot_voicecontroller.py test-tts "Hello, I am TARS"

# Test LLM command generation (no execution)
python3 ~/robot_voicecontroller.py test-llm "go forward slowly"
```

### Remote Operation (from dev machine)

```bash
# Run check remotely
uv run python deploy_voicecontroller.py test

# Start voice control remotely
uv run python deploy_voicecontroller.py run loop
```

## Example Interaction

```
[TARS announces] "TARS online. Say Hey TARS to activate."

[User] "Hey TARS"
[Beep sound]

[TARS] "Recording 4s... Speak now!"

[User] "Go forward slowly"

[TARS processes]
  - faster-whisper: "go forward slowly"
  - Bedrock Haiku: {"action": "movement", "command": "forward", "params": {"speed": 0.15, "duration": 2.0}, "response": "Moving forward slowly, sir."}
  - ROS2: ros2 topic pub /controller/cmd_vel ...

[Robot moves forward for 2 seconds at 0.15 m/s]

[TARS speaks] "Moving forward slowly, sir."

[Returns to listening for wake word]
```

## Troubleshooting

### No audio input

```bash
# Check audio devices
arecord -l

# Test recording
arecord -D hw:1,0 -f S16_LE -r 48000 -c 2 -d 3 test.wav
aplay test.wav
```

**Solutions**:
- Verify WonderEcho Pro is connected (card 1)
- Check USB connection
- Ensure user is in `audio` group: `sudo usermod -aG audio $USER`

### Wake word not detected

- Speak clearly at normal volume
- Reduce background noise
- Try variations: "Hey Tars", "Hey TARS"
- Check whisper model is loaded (first run downloads ~150MB)

### Robot doesn't move

```bash
# Check ROS2 stack
docker ps | grep landerpi-ros2

# Check cmd_vel topic
docker exec landerpi-ros2 ros2 topic list | grep cmd_vel

# Manual test
docker exec landerpi-ros2 ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
```

**Solutions**:
- Redeploy ROS2 stack: `uv run python deploy_ros2_stack.py deploy`
- Check battery level
- Verify Docker is running

### TTS not working

```bash
# Check API key
cat ~/.env | grep ELEVENLABS

# Test speaker
aplay -D plughw:1,0 /usr/share/sounds/alsa/Front_Center.wav

# Check ffmpeg
which ffmpeg
```

**Solutions**:
- Verify ElevenLabs API key is valid
- Install ffmpeg: `sudo apt install ffmpeg`
- Check speaker volume

### Service fails on boot

```bash
# View detailed logs
journalctl -u tars-voice -n 100

# Common causes:
# - Network not ready (service waits 10s, may need more)
# - Docker not ready (dependency in service file)
# - Credentials missing
```

**Solutions**:
- Increase ExecStartPre sleep: edit `/etc/systemd/system/tars-voice.service`
- Run `deploy_voicecontroller.py configure` to fix credentials

### AWS Bedrock errors

```bash
# Check credentials
aws sts get-caller-identity

# Check region
echo $AWS_REGION  # Should be us-east-1

# Test Bedrock access
aws bedrock list-foundation-models --query 'modelSummaries[?contains(modelId, `claude`)]'
```

**Solutions**:
- Run `aws configure` with correct credentials
- Verify IAM policy includes `bedrock:InvokeModel`
- Check region supports Claude Haiku 4.5

## Files on Robot

| File | Purpose |
|------|---------|
| `~/robot_voicecontroller.py` | Main voice controller script |
| `~/.env` | API keys (ELEVENLABS_API_KEY, etc.) |
| `~/audio/beep.wav` | Wake word activation sound |
| `/etc/systemd/system/tars-voice.service` | Systemd service |
| `~/.cache/huggingface/` | Whisper model cache |

## Performance Notes

- **Startup time**: ~5-10 seconds (loading Whisper model)
- **Response latency**: ~2-3 seconds (ASR + LLM + TTS)
- **Whisper model**: `base.en` (~150MB, runs on CPU)
- **Memory usage**: ~500MB (mostly Whisper)

## Future Roadmap

1. **Arm control** - Voice commands for the robotic arm
2. **SLAM patrol** - Autonomous mapping during patrol
3. **Object detection** - Camera-based object identification
4. **Sound localization** - Use WonderEcho Pro beam direction for "come here"
5. **Multi-language** - Support for non-English commands
6. **Offline mode** - Local LLM for basic commands without cloud

## References

- Design document: `docs/plans/2024-12-29-voice-control-enhancements-design.md`
- Implementation plan: `docs/plans/2024-12-29-voice-control-enhancements.md`
- Voice skill: `.claude/skills/landerpi-voice/SKILL.md`
- Deployment script: `deploy_voicecontroller.py`
- Robot script: `robot_voicecontroller.py`
- Test script: `validation/test_voicecontroller.py`
