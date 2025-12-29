# Voice Control Enhancements Design

**Date**: 2024-12-29
**Status**: Approved
**Author**: Claude + Julien

## Overview

Enhance the LanderPi voice controller with working motor control, expanded commands, wake word detection, and auto-start on boot.

## Current State

- Voice recognition (faster-whisper): Working
- LLM processing (Bedrock Haiku 4.5): Working
- TTS response (ElevenLabs): Working
- Motor control: NOT working (uses broken direct SDK path)

## Target State

```
┌─────────────────────────────────────────────────────────────┐
│                   robot_voicecontroller.py                   │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  [IDLE] ──► Wake word detector (listens for "Hey TARS")     │
│                         │                                    │
│                         ▼                                    │
│  [LISTENING] ──► arecord (3-4 sec voice capture)            │
│                         │                                    │
│                         ▼                                    │
│  [PROCESSING] ──► faster-whisper ──► Bedrock Haiku          │
│                         │                                    │
│                         ▼                                    │
│  [EXECUTING] ──► ROS2 cmd_vel (via docker exec)             │
│                         │                                    │
│                         ▼                                    │
│  [SPEAKING] ──► ElevenLabs TTS ──► aplay                    │
│                         │                                    │
│                         ▼                                    │
│                   Back to [IDLE]                             │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Feature 1: ROS2 Movement Control

### Approach

Replace direct SDK calls with ROS2 topic publishing via Docker.

### Implementation

```python
def execute_ros2_cmd_vel(linear_x, linear_y, angular_z, duration):
    """Publish velocity command via ROS2."""
    cmd = f'''docker exec landerpi-ros2 bash -c "
        ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist \\
        '{{linear: {{x: {linear_x}, y: {linear_y}, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {angular_z}}}}}'"
    '''
    subprocess.run(cmd, shell=True)
    time.sleep(duration)
    # Stop command
    stop_cmd = cmd.replace(str(linear_x), "0.0").replace(str(linear_y), "0.0").replace(str(angular_z), "0.0")
    subprocess.run(stop_cmd, shell=True)
```

### Movement Mappings

| Command | linear.x | linear.y | angular.z |
|---------|----------|----------|-----------|
| forward | 0.3 | 0 | 0 |
| backward | -0.3 | 0 | 0 |
| turn_left | 0 | 0 | 0.5 |
| turn_right | 0 | 0 | -0.5 |
| strafe_left | 0 | 0.3 | 0 |
| strafe_right | 0 | -0.3 | 0 |
| stop | 0 | 0 | 0 |

### Compound Commands

| Command | Behavior |
|---------|----------|
| look_around | Rotate 360° slowly (angular.z=0.3 for ~7s) |
| patrol | Square pattern: forward→turn_right→forward→turn_right (4x) |
| come_here | Move forward 2s (future: use sound angle) |

### Mode Commands

| Command | Behavior |
|---------|----------|
| follow_me | Continuous tracking mode (toggle on/off) |

## Feature 2: Updated TARS Prompt

### Design Goals

- More professional, less jokey
- Concise responses (max 15 words)
- Support new command types

### Prompt

```python
ROBOT_CONTROL_PROMPT = """You are TARS, an efficient AI assistant controlling a LanderPi robot.
Be concise, professional, and direct. Address user as "sir" occasionally.

Available commands:
- Movement: forward, backward, turn_left, turn_right, strafe_left, strafe_right, stop
- Compound: look_around (360° scan), patrol (square pattern), come_here (approach user)
- Mode: follow_me (continuous tracking - starts/stops a mode)
- Arm: home, wave (if equipped)

Parameters:
- speed: 0.1-0.5 m/s (default 0.3)
- duration: 0.5-10.0 seconds (default 2.0)

Output JSON only:
{
    "action": "movement" | "compound" | "mode" | "arm" | "stop" | "chat",
    "command": "<command_name>",
    "params": {"speed": 0.3, "duration": 2.0},
    "response": "<brief confirmation, max 15 words>"
}

Examples:
- "go forward" → {"action": "movement", "command": "forward", "params": {"speed": 0.3, "duration": 2.0}, "response": "Moving forward."}
- "look around" → {"action": "compound", "command": "look_around", "params": {}, "response": "Scanning surroundings."}
- "come here" → {"action": "compound", "command": "come_here", "params": {}, "response": "Approaching your position."}
- "follow me" → {"action": "mode", "command": "follow_me", "params": {"enable": true}, "response": "Following mode engaged."}
- "stop following" → {"action": "mode", "command": "follow_me", "params": {"enable": false}, "response": "Following mode disengaged."}
- "what time is it" → {"action": "chat", "command": "none", "params": {}, "response": "I control motors, not clocks, sir."}

Rules:
1. Output ONLY valid JSON
2. Responses: professional, under 15 words
3. Unknown commands: use "chat" action
"""
```

## Feature 3: Wake Word Detection

### Approach

Use faster-whisper in a lightweight continuous loop to detect "Hey TARS".

### Implementation

```python
def wait_for_wake_word(timeout=None):
    """Listen continuously for 'Hey TARS' wake phrase."""
    start = time.time()
    while True:
        if timeout and (time.time() - start) > timeout:
            return False

        # Record short clip (1.5 seconds)
        audio = record_audio(duration=1.5)

        # Quick transcription
        text = transcribe_audio(audio)

        if text and "hey tars" in text.lower():
            play_activation_sound()  # Short beep
            return True

        # Cleanup temp file
        cleanup(audio)

        # Small pause to reduce CPU
        time.sleep(0.1)
```

### Flow

```
START → wait_for_wake_word()
      → "Hey TARS" detected
      → play_beep()
      → record_command(4 seconds)
      → transcribe → LLM → execute → speak
      → back to wait_for_wake_word()
```

### Exit Commands

- "Stop listening" - exits loop gracefully
- "Goodbye" - exits loop gracefully
- Ctrl+C - keyboard interrupt

## Feature 4: Systemd Service

### Service File

Location: `/etc/systemd/system/tars-voice.service`

```ini
[Unit]
Description=TARS Voice Controller for LanderPi
After=network-online.target docker.service
Wants=network-online.target
Requires=docker.service

[Service]
Type=simple
User=tonbost
Environment=HOME=/home/tonbost
EnvironmentFile=/home/tonbost/.env
WorkingDirectory=/home/tonbost
ExecStartPre=/bin/sleep 10
ExecStart=/usr/bin/python3 /home/tonbost/robot_voicecontroller.py loop --wake-word
ExecStop=/bin/kill -SIGINT $MAINPID
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### Features

- Waits for Docker service (ROS2 stack must be running)
- 10-second delay after boot for system stability
- Loads `.env` file for API keys (ELEVENLABS_API_KEY, etc.)
- `--wake-word` flag enables wake-word-only mode
- Auto-restarts on failure with 5s delay
- Graceful shutdown via SIGINT

### Management Commands

```bash
# Install and enable
sudo cp systemd/tars-voice.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable tars-voice

# Control
sudo systemctl start tars-voice
sudo systemctl stop tars-voice
sudo systemctl restart tars-voice

# Monitor
sudo systemctl status tars-voice
sudo journalctl -u tars-voice -f
```

## Files to Modify/Create

| File | Action | Description |
|------|--------|-------------|
| `robot_voicecontroller.py` | Modify | Add ROS2 control, wake word, new commands |
| `deploy_voicecontroller.py` | Modify | Add systemd service installation |
| `systemd/tars-voice.service` | Create | Systemd unit file |
| `audio/beep.wav` | Create | Activation sound for wake word |

## Future Enhancements

Not in scope for this iteration:

1. **Patrol with SLAM** - Use lidar for mapping during patrol
2. **Object Detection** - Camera-based object identification
3. **Sound Localization** - Use WonderEcho Pro angle for "come here"
4. **Multi-language** - Support languages other than English

## Testing Plan

1. **Unit tests**: Test each ROS2 command mapping
2. **Integration test**: Full voice → movement pipeline
3. **Wake word test**: Verify "Hey TARS" detection accuracy
4. **Service test**: Verify auto-start on reboot

## Success Criteria

- [ ] "Go forward" moves robot forward via ROS2
- [ ] "Look around" performs 360° rotation
- [ ] "Patrol" executes square pattern
- [ ] "Hey TARS" activates listening mode
- [ ] Service starts automatically on boot
- [ ] Service recovers from failures
