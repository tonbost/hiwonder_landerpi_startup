# Voice Control Integration Plan for LanderPi

## Executive Summary

This document analyzes the HiWonder LanderPi voice control system and outlines a plan to integrate voice control capabilities via a `test_voicecontroller.py` script.

## Findings: Two Voice Control Architectures

The LanderPi supports **two distinct voice control approaches**:

### 1. Offline Voice Control (xf_mic_asr_offline)

**NO LLM REQUIRED** - Uses pre-defined command mapping.

| Component | Description |
|-----------|-------------|
| Wake Word | "Hello Hiwonder" (iFlytek offline detection) |
| ASR | iFlytek offline recognition (30+ pre-defined commands) |
| Command Mapping | Direct string-to-action mapping |
| Response | Pre-recorded WAV files |

**Pre-defined Commands (English):**
- Movement: "go forward", "go backward", "turn left", "turn right", "stop", "come here"
- Arm: "pick a carrot", "pass me please"
- Color: "start color recognition", "track red object", "gripping red"
- Navigation: "go to A point", "go back to the start"

**Source Files:**
- `reference/LanderPi/src/xf_mic_asr_offline/scripts/voice_control_move.py`
- `reference/LanderPi/src/xf_mic_asr_offline/scripts/wonder_echo_pro_node.py`

### 2. Large Model Voice Control (large_models)

**LLM REQUIRED** - Natural language understanding via cloud APIs.

| Component | Description |
|-----------|-------------|
| Wake Word | Same hardware-based detection |
| ASR | OpenAI Whisper (English) or Aliyun Paraformer (Chinese) |
| NLU | GPT-4o-mini, Qwen, or other LLMs |
| TTS | OpenAI TTS-1 or Aliyun Sambert |
| Command Generation | LLM generates structured JSON commands from prompts |

**Flow:**
```
Voice → Wake Detection → Cloud ASR → LLM (with prompt) → JSON action → Robot Control → TTS Response
```

**Example Prompt (from llm_control_move.py):**
```
You are an intelligent car controlled via linear velocity (x,y) and angular velocity (z).
Generate JSON: {"action": [[x, y, z, time], ...], "response": "..."}
```

**Source Files:**
- `reference/LanderPi/src/large_models/large_models/large_models/vocal_detect.py`
- `reference/LanderPi/src/large_models/large_models/large_models/agent_process.py`
- `reference/LanderPi/src/large_models/large_models/large_models/tts_node.py`
- `reference/LanderPi/src/large_models_examples/large_models_examples/llm_control_move.py`

---

## Hardware Prerequisites

### Required: WonderEcho Pro

<img src="../docs/LanderPi/source/_static/media/chapter_11/section_2/media/image1.png" width="300"/>

| Specification | Value |
|--------------|-------|
| Interface | USB 2.0 |
| Voice Chip | CL1302 |
| Speaker Output | 3.0W per channel (4Ω BTL) |
| Power | 5V DC |
| Device Path | `/dev/ring_mic` or `/dev/ttyUSB1` |

**Alternative:** 6-Microphone Circular Array (same interface)

### Audio Verification Commands
```bash
# List recording devices
arecord -l

# Test recording (5 seconds)
arecord -D hw:0,0 -f S16_LE -r 16000 -c 2 test.wav

# Test playback
aplay test.wav
```

---

## Software Prerequisites

### 1. Core Speech Module
The `speech` Python module is a **compiled binary** (.so file) that must be installed:
```bash
cd ~/large_models/speech_pkg/
pip3 install -r requirements.txt --break-system-packages
cd speech/rpi5/
cp -r speech.so ..
cd ../..
pip3 install .
```

This module provides:
- `speech.awake.WonderEchoPro()` - Wake word detection
- `speech.awake.CircleMic()` - 6-mic wake detection
- `speech.RealTimeASR()` - Chinese ASR
- `speech.RealTimeOpenAIASR()` - English ASR (via OpenAI Whisper)
- `speech.RealTimeTTS()` - Chinese TTS
- `speech.RealTimeOpenAITTS()` - English TTS
- `speech.OpenAIAPI()` - LLM interface
- `speech.play_audio()` - Audio playback
- `speech.set_volume()` - Volume control

### 2. API Keys (for LLM mode)
Configure in `~/large_models/config.py`:
```python
# OpenAI (International)
llm_api_key = 'sk-...'
llm_base_url = 'https://api.openai.com/v1'
llm_model = 'gpt-4o-mini'

# OR Aliyun (China)
aliyun_api_key = 'sk-...'
aliyun_base_url = 'https://dashscope.aliyuncs.com/compatible-mode/v1'
```

### 3. Environment Variables
```bash
export ASR_LANGUAGE=English  # or Chinese
export MIC_TYPE=xf  # or other
export MACHINE_TYPE=LanderPi
```

### 4. ROS2 Dependencies
Required ROS2 packages:
- `xf_mic_asr_offline` - Offline voice control
- `large_models` - LLM voice control
- `large_models_msgs` - Custom message types
- `ros_robot_controller_msgs` - Robot control messages

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        Voice Input                               │
│                   (WonderEcho Pro USB)                          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│               Wake Word Detection                                │
│    speech.awake.WonderEchoPro() or CircleMic()                  │
│    Triggers on "Hello Hiwonder"                                  │
└─────────────────────────────────────────────────────────────────┘
                              │
            ┌─────────────────┴─────────────────┐
            ▼                                   ▼
┌─────────────────────┐            ┌─────────────────────┐
│   OFFLINE MODE      │            │    LLM MODE         │
│   (No Internet)     │            │   (Cloud APIs)      │
├─────────────────────┤            ├─────────────────────┤
│ • iFlytek offline   │            │ • OpenAI Whisper    │
│   ASR               │            │   ASR               │
│ • 30+ pre-defined   │            │ • LLM (GPT/Qwen)    │
│   commands          │            │   processes text    │
│ • Direct action     │            │ • Returns JSON      │
│   mapping           │            │   commands          │
│ • WAV response      │            │ • TTS response      │
└─────────────────────┘            └─────────────────────┘
            │                                   │
            └─────────────────┬─────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Robot Control                                │
│    /controller/cmd_vel (Twist) - Movement                       │
│    /ros_robot_controller/set_motor - Direct motor              │
│    /arm/cmd - Arm control                                        │
└─────────────────────────────────────────────────────────────────┘
```

---

## ROS2 Topics & Services

### Offline Mode (xf_mic_asr_offline)
| Topic/Service | Type | Description |
|---------------|------|-------------|
| `/asr_node/voice_words` | String | Recognized command |
| `/awake_node/angle` | Int32 | Sound source angle (0-360°) |
| `/awake_node/awake_flag` | Bool | Wake status |
| `/voice_control/get_offline_result` | Service | Get ASR result |

### LLM Mode (large_models)
| Topic/Service | Type | Description |
|---------------|------|-------------|
| `vocal_detect/asr_result` | String | Transcribed speech |
| `vocal_detect/wakeup` | Bool | Wake status |
| `vocal_detect/angle` | Int32 | Sound source angle |
| `agent_process/result` | String | LLM response (JSON) |
| `tts_node/tts_text` | String | Text to speak |
| `tts_node/play_finish` | Bool | TTS completed |
| `agent_process/set_model` | Service | Configure LLM |
| `agent_process/set_prompt` | Service | Set system prompt |

---

## Implementation Plan for test_voicecontroller.py

### Phase 1: Hardware Check (No ROS2)

Create basic connectivity test:
```python
# test_voicecontroller.py - Phase 1

def check_audio_hardware():
    """Verify WonderEcho Pro is connected."""
    # Check /dev/ring_mic or /dev/ttyUSB1 exists
    # Test recording capability
    # Test playback capability

def check_speech_module():
    """Verify speech module is installed."""
    # Import speech module
    # Check ASR_LANGUAGE env var
```

### Phase 2: Wake Word Detection (Minimal ROS2)

Test wake word without full ROS2 stack:
```python
from speech import awake

def test_wake_detection():
    """Test wake word detection."""
    kws = awake.WonderEchoPro('/dev/ring_mic')
    kws.start()
    print("Say 'Hello Hiwonder'...")
    result = kws.wakeup()
    if result:
        print(f"Wake detected! Angle: {result}")
```

### Phase 3: Offline Voice Commands

Implement offline command recognition:
```python
def test_offline_commands():
    """Test offline voice command recognition."""
    # Use xf_mic_asr_offline approach
    # Map recognized commands to actions
    # Execute via direct SDK (no ROS2)
```

### Phase 4: LLM Integration (Optional)

Add LLM-based natural language control:
```python
from speech import speech

def test_llm_voice_control():
    """Test LLM-based voice control."""
    client = speech.OpenAIAPI(api_key, base_url)
    asr = speech.RealTimeOpenAIASR()
    tts = speech.RealTimeOpenAITTS()

    # Record and transcribe
    text = asr.asr()

    # Send to LLM with robot control prompt
    response = client.llm(text, ROBOT_PROMPT, model='gpt-4o-mini')

    # Parse and execute action
    action = parse_action(response)
    execute_robot_command(action)

    # Speak response
    tts.tts(response['response'])
```

---

## Recommended Approach

### Simplest Path: Offline Mode WITHOUT Full ROS2

For a `test_voicecontroller.py` that works with our existing direct control scripts:

1. **Use WonderEcho Pro** for wake + offline command detection
2. **Map commands to existing functions** in `test_chassis_direct.py`, `test_arm.py`
3. **Pre-recorded WAV responses** (no TTS API needed)
4. **Optional**: Add LLM mode for natural language

### Why This Works
- `wonder_echo_pro_node.py` shows the hardware can detect commands locally
- Our existing `ros_robot_controller_sdk.py` handles motor control
- No need for full ROS2 stack or cloud connectivity for basic control

---

## Open Questions

1. **Hardware Availability**: Do you have the WonderEcho Pro module?
2. **Speech Module**: Is the `speech.so` binary available for your Pi 5 Python version?
3. **Network**: Do you want cloud-based (LLM) or purely offline control?
4. **Commands**: Which robot actions should voice commands trigger?

---

## Next Steps

1. **Verify Hardware**: Check if WonderEcho Pro is connected
2. **Check Speech Module**: Verify `/home/ubuntu/large_models/speech_pkg/` exists on robot
3. **Test Audio**: Run `arecord`/`aplay` tests on robot
4. **Create test_voicecontroller.py**: Start with Phase 1 (hardware check)

---

## File References

| File | Purpose |
|------|---------|
| `reference/LanderPi/src/xf_mic_asr_offline/scripts/voice_control_move.py` | Offline voice→movement |
| `reference/LanderPi/src/xf_mic_asr_offline/scripts/wonder_echo_pro_node.py` | Hardware detection node |
| `reference/LanderPi/src/large_models/large_models/large_models/vocal_detect.py` | Cloud ASR node |
| `reference/LanderPi/src/large_models/large_models/large_models/agent_process.py` | LLM processing |
| `reference/LanderPi/src/large_models/large_models/large_models/tts_node.py` | Text-to-speech |
| `reference/LanderPi/src/large_models/large_models/large_models/config.py` | API keys |
| `reference/LanderPi/src/large_models_examples/large_models_examples/llm_control_move.py` | LLM→robot control example |
| `docs/LanderPi/source/docs/11_Voice_Control_Course.md` | Official voice tutorial |
| `docs/LanderPi/source/docs/12_Large_AI_Model_Course.md` | LLM integration tutorial |
