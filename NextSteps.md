# Voice Control Setup - Next Steps

This document outlines the steps to complete voice control integration for the LanderPi robot.

## Prerequisites

- WonderEcho Pro connected to robot (verified: card 1 USB Audio Device)
- Robot has WiFi internet access
- AWS CLI configured on local machine
- ElevenLabs API key (already in `.env`)

## Step 1: Create IAM User for Robot

Create a dedicated AWS IAM user with minimal Bedrock-only permissions.

```bash
# Create the IAM user
aws iam create-user --user-name lp-robot

# Attach the Bedrock-only policy
aws iam put-user-policy \
  --user-name lp-robot \
  --policy-name BedrockInvokeOnly \
  --policy-document file://aws/lp-robot-bedrock-policy.json

# Create access key (SAVE THE OUTPUT!)
aws iam create-access-key --user-name lp-robot
```

**Save the `AccessKeyId` and `SecretAccessKey` from the output** - you'll need them in Step 3.

## Step 2: Deploy to Robot

From your local machine, run the deployment:

```bash
# Full deployment: upload script + install dependencies + configure
uv run python deploy_voicecontroller.py deploy

# Or run individual steps:
uv run python deploy_voicecontroller.py upload     # Upload script only
uv run python deploy_voicecontroller.py install    # Install Python packages
uv run python deploy_voicecontroller.py configure  # Setup credentials
```

This will:
- Upload `robot_voicecontroller.py` to `~/robot_voicecontroller.py` on robot
- Install: `typer`, `rich`, `boto3`, `faster-whisper`, `elevenlabs`, `ffmpeg`
- Copy ElevenLabs credentials from local `.env` to robot

## Step 3: Configure AWS on Robot

SSH to the robot and configure AWS credentials:

```bash
ssh tonbost@192.168.50.169

# Configure AWS CLI with the credentials from Step 1
aws configure
# AWS Access Key ID: <from step 1>
# AWS Secret Access Key: <from step 1>
# Default region name: us-east-1
# Default output format: json

# Verify it works
aws sts get-caller-identity
```

## Step 4: Test Components

On the robot, verify all components are working:

```bash
# Check all components
python3 ~/robot_voicecontroller.py check

# Expected output:
# ┌─────────────────┬────────┬─────────────────────┐
# │ Component       │ Status │ Details             │
# ├─────────────────┼────────┼─────────────────────┤
# │ WonderEcho Pro  │ OK     │ Audio device        │
# │ faster-whisper  │ OK     │ Model: base.en      │
# │ AWS Credentials │ OK     │ Account: ...xxxx    │
# │ AWS Bedrock     │ OK     │ Region: us-east-1   │
# │ ElevenLabs      │ OK     │ Voice: ...xxxxx     │
# │ Robot SDK       │ OK     │ ~/ros_robot_ctrl... │
# │ ffmpeg          │ OK     │ For TTS conversion  │
# └─────────────────┴────────┴─────────────────────┘
```

## Step 5: Test Individual Components

```bash
# Test TTS (speaks through WonderEcho Pro speaker)
python3 ~/robot_voicecontroller.py test-tts "Hello sir, TARS online."

# Test LLM command generation (no robot movement)
python3 ~/robot_voicecontroller.py test-llm "move forward for 2 seconds"

# Test audio recording and transcription
python3 ~/robot_voicecontroller.py test-record --duration 3

# Test motors (1 second forward movement)
python3 ~/robot_voicecontroller.py test-motors
```

## Step 6: Run Voice Control

```bash
# Single voice command
python3 ~/robot_voicecontroller.py listen

# Continuous listening mode (say "stop listening" to exit)
python3 ~/robot_voicecontroller.py loop

# With shorter recording duration
python3 ~/robot_voicecontroller.py loop --duration 3
```

## Voice Commands Supported

| Say | Robot Action |
|-----|--------------|
| "go forward" | Move forward 2 seconds |
| "move backward slowly" | Move backward at low speed |
| "turn left" | Rotate left |
| "turn right" | Rotate right |
| "strafe left" | Sideways movement left |
| "stop" | Emergency stop |
| "move the arm home" | Arm to home position |
| "hello" | Chat response (no movement) |

## Troubleshooting

### Audio not recording
```bash
# Check audio devices
arecord -l

# Test manual recording
arecord -D hw:1,0 -f S16_LE -r 48000 -c 2 -d 3 /tmp/test.wav
aplay /tmp/test.wav
```

### AWS Bedrock errors
```bash
# Verify credentials
aws sts get-caller-identity

# Test Bedrock access
aws bedrock-runtime invoke-model \
  --model-id us.anthropic.claude-haiku-4-5-20251001-v1:0 \
  --body '{"anthropic_version":"bedrock-2023-05-31","max_tokens":10,"messages":[{"role":"user","content":"hi"}]}' \
  --content-type application/json \
  /tmp/response.json
```

### ElevenLabs not working
```bash
# Check environment variable
echo $ELEVENLABS_API_KEY

# Source .env if needed
source ~/.env
```

### faster-whisper model download
On first run, the Whisper model will download (~150MB for base.en). This requires internet and may take a few minutes.

## Files Reference

| File | Location | Purpose |
|------|----------|---------|
| `robot_voicecontroller.py` | Robot: `~/` | Main voice controller |
| `deploy_voicecontroller.py` | Local | Deployment tool |
| `aws/landerpi-bedrock-policy.json` | Local | IAM policy |
| `.env` | Robot: `~/` | API keys |

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    LanderPi Robot                            │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  WonderEcho Pro ──► arecord (48kHz stereo)                  │
│        │                    │                                │
│        │                    ▼                                │
│        │           faster-whisper (local ASR)               │
│        │                    │                                │
│        │                    ▼                                │
│        │           AWS Bedrock Claude Haiku 4.5             │
│        │                    │                                │
│        │                    ▼                                │
│        │           JSON: {action, command, response}        │
│        │                    │                                │
│        │         ┌─────────┴─────────┐                      │
│        │         ▼                   ▼                      │
│        │   ros_robot_controller   ElevenLabs TTS            │
│        │   (motor control)        (cloud API)               │
│        │         │                   │                      │
│        │         ▼                   ▼                      │
│        │    Motors move         MP3 audio                   │
│        │                             │                      │
│        │                             ▼                      │
│        ◄─────────────────── aplay (speaker)                 │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```
