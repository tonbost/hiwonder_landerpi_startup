---
name: landerpi-voice
description: Voice control for HiWonder LanderPi robot using TARS assistant. Provides wake word detection ("Hey TARS"), natural language commands, and TTS responses. Requires landerpi-core skill and deployed ROS2 stack.
---

# LanderPi Voice Control (TARS)

## Overview

TARS is an AI voice assistant that controls the LanderPi robot using natural language commands. It runs on the robot with cloud ASR (AWS Transcribe Streaming), cloud LLM (Bedrock), and cloud TTS (ElevenLabs).

**Prerequisites:**
- Load `landerpi-core` skill first
- ROS2 stack deployed (`deploy_ros2_stack.py deploy`)
- WonderEcho Pro audio module connected
- AWS credentials configured on robot
- ElevenLabs API key in `~/.env`

## Architecture

```
WonderEcho Pro (arecord) --> AWS Transcribe Streaming (ASR) --> Bedrock Claude Haiku (LLM)
                                     |                                   |
                              (STT_ENGINE=transcribe)            JSON command
                                     |                                   |
                             [fallback: faster-whisper]                  v
                                                docker exec landerpi-ros2 ros2 topic pub /cmd_vel
                                                                         |
                                                                         v
                                       ElevenLabs TTS --> WonderEcho Pro (aplay)
```

**Speech-to-Text Engine:**
- Default: `STT_ENGINE=transcribe` (AWS Transcribe Streaming - faster, ~1-2s latency)
- Fallback: `STT_ENGINE=whisper` (local faster-whisper - slower, ~3-5s latency)

## Deployment Commands

| Command | Purpose |
|---------|---------|
| `uv run python deploy_voicecontroller.py deploy` | Full deployment (script + deps + service) |
| `uv run python deploy_voicecontroller.py upload` | Upload script only |
| `uv run python deploy_voicecontroller.py install` | Install dependencies only |
| `uv run python deploy_voicecontroller.py configure` | Configure credentials |
| `uv run python deploy_voicecontroller.py install-service` | Install systemd service |
| `uv run python deploy_voicecontroller.py test` | Run component check on robot |

## Service Management

The voice controller runs as a systemd service that auto-starts on boot.

```bash
# On the robot:
sudo systemctl start tars-voice    # Start service
sudo systemctl stop tars-voice     # Stop service
sudo systemctl restart tars-voice  # Restart service
sudo systemctl status tars-voice   # Check status
sudo systemctl enable tars-voice   # Enable auto-start
sudo systemctl disable tars-voice  # Disable auto-start
journalctl -u tars-voice -f        # Follow logs
```

## Manual Operation (on robot)

```bash
# Single voice command
python3 ~/robot_voicecontroller.py listen

# Continuous mode (always listening)
python3 ~/robot_voicecontroller.py loop

# Wake word mode (requires "Hey TARS" activation)
python3 ~/robot_voicecontroller.py loop --wake-word

# Component check
python3 ~/robot_voicecontroller.py check

# Test TTS
python3 ~/robot_voicecontroller.py test-tts "Hello, I am TARS"

# Test LLM command generation (no execution)
python3 ~/robot_voicecontroller.py test-llm "go forward slowly"
```

## Supported Voice Commands

### Movement Commands
| Command | Action | Example Phrases |
|---------|--------|-----------------|
| forward | Move forward | "go forward", "move ahead" |
| backward | Move backward | "go back", "reverse" |
| turn_left | Rotate left | "turn left", "rotate left" |
| turn_right | Rotate right | "turn right", "spin right" |
| strafe_left | Slide left | "move left", "strafe left" |
| strafe_right | Slide right | "move right", "strafe right" |
| stop | Emergency stop | "stop", "halt", "freeze" |

### Compound Commands
| Command | Action | Duration |
|---------|--------|----------|
| look_around | 360-degree rotation scan | ~7 seconds |
| patrol | Square pattern (4 legs) | ~30 seconds |
| come_here | Approach user | ~3 seconds |

### Mode Commands
| Command | Action |
|---------|--------|
| follow_me (enable) | Start continuous following |
| follow_me (disable) | Stop following ("stop following") |

### Parameters
- **Speed**: 0.1-0.5 m/s (default: 0.3)
- **Duration**: 0.5-10.0 seconds (default: 2.0)

Example: "go forward slowly" -> speed=0.15, "turn left fast" -> speed=0.5

## Wake Word Mode

1. On startup: TARS announces "TARS online. Say Hey TARS to activate."
2. Listen for wake phrase: "Hey TARS"
3. Beep confirms activation
4. User gives command
5. TARS executes and responds via TTS
6. Returns to listening for wake word

**Exit commands:** "stop listening", "goodbye", "exit"

## Component Requirements

| Component | Purpose | Check |
|-----------|---------|-------|
| WonderEcho Pro | Audio I/O | `arecord -l` shows card 1 |
| AWS Transcribe | Cloud ASR (default) | `python3 -c "from amazon_transcribe.client import TranscribeStreamingClient"` |
| faster-whisper | Local ASR (fallback) | `python3 -c "from faster_whisper import WhisperModel"` |
| AWS Bedrock | LLM | `aws sts get-caller-identity` |
| ElevenLabs | TTS | `ELEVENLABS_API_KEY` in ~/.env |
| ROS2 Stack | Motion control | `docker ps` shows landerpi-ros2 |
| ffmpeg | Audio conversion | `which ffmpeg` |

## LLM Prompt

TARS uses a structured prompt that generates JSON commands:

```json
{
    "action": "movement" | "compound" | "mode" | "arm" | "stop" | "chat",
    "command": "<command_name>",
    "params": {"speed": 0.3, "duration": 2.0},
    "response": "<brief TTS response, max 15 words>"
}
```

## Troubleshooting

### Problem: No audio input

**Symptoms:** "Recording failed" or no speech detected

**Solution:**
```bash
# Check audio device
arecord -l

# Test recording
arecord -D hw:1,0 -f S16_LE -r 48000 -c 2 -d 3 test.wav
aplay test.wav
```

### Problem: Wake word not detected

**Symptoms:** "Hey TARS" not recognized

**Solution:**
1. Speak clearly and at normal volume
2. Check microphone positioning
3. Reduce background noise
4. Try "Hey Tars" or "Hey TARS" variations

### Problem: Commands not executing

**Symptoms:** TARS responds but robot doesn't move

**Solution:**
```bash
# Check ROS2 stack is running
docker ps | grep landerpi-ros2

# If not running, deploy stack
uv run python deploy_ros2_stack.py deploy
```

### Problem: TTS not working

**Symptoms:** No spoken response

**Solution:**
```bash
# Check ElevenLabs key
cat ~/.env | grep ELEVENLABS

# Check speaker
aplay -D plughw:1,0 /usr/share/sounds/alsa/Front_Center.wav
```

### Problem: Service fails on boot

**Symptoms:** `systemctl status tars-voice` shows failure

**Solution:**
```bash
# Check logs
journalctl -u tars-voice -n 50

# Common causes:
# - Network not ready: service waits 10s, may need more
# - Docker not ready: check docker.service dependency
# - Credentials missing: run deploy_voicecontroller.py configure
```

## Files

| File | Location | Purpose |
|------|----------|---------|
| Voice controller | `~/robot_voicecontroller.py` | Main script (on robot) |
| Systemd service | `/etc/systemd/system/tars-voice.service` | Boot service |
| Credentials | `~/.env` | ElevenLabs API key |
| Beep sound | `~/audio/beep.wav` | Wake word confirmation |

## AWS IAM Policy

The robot uses minimal permissions for Bedrock and Transcribe:

```json
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Sid": "BedrockInvokeOnly",
            "Effect": "Allow",
            "Action": ["bedrock:InvokeModel", "bedrock:InvokeModelWithResponseStream"],
            "Resource": [
                "arn:aws:bedrock:*::foundation-model/anthropic.claude-*",
                "arn:aws:bedrock:us-east-1:*:inference-profile/us.anthropic.claude-*"
            ]
        },
        {
            "Sid": "TranscribeStreaming",
            "Effect": "Allow",
            "Action": ["transcribe:StartStreamTranscription", "transcribe:StartStreamTranscriptionWebSocket"],
            "Resource": "*"
        }
    ]
}
```

Policy file: `aws/lp-robot-bedrock-policy.json`
