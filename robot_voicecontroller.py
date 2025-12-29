#!/usr/bin/env python3
"""
Voice Controller for LanderPi Robot (runs ON the robot)

Uses WonderEcho Pro for audio I/O, AWS Bedrock for LLM,
and ElevenLabs for TTS - all processed locally on the robot.

Architecture (all on robot):
    1. WonderEcho Pro microphone → arecord
    2. faster-whisper (local) → Speech-to-text
    3. AWS Bedrock Claude Haiku 4.5 → Command generation
    4. ros_robot_controller_sdk → Direct robot control
    5. ElevenLabs TTS → aplay through WonderEcho speaker

Prerequisites:
    pip install faster-whisper boto3 elevenlabs typer rich

AWS Setup:
    aws configure  # Use robot-specific IAM credentials
    # IAM policy: BedrockInvokeModel only

Usage:
    python3 robot_voicecontroller.py listen          # Single voice command
    python3 robot_voicecontroller.py loop            # Continuous listening
    python3 robot_voicecontroller.py check           # Check all components
    python3 robot_voicecontroller.py test-tts "Hi"   # Test TTS
    python3 robot_voicecontroller.py test-llm "go forward"  # Test LLM
"""

import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Optional

# Check for required modules and provide install instructions
REQUIRED_MODULES = {
    "typer": "typer",
    "rich": "rich",
    "boto3": "boto3",
    "faster_whisper": "faster-whisper",
}

missing = []
for module, package in REQUIRED_MODULES.items():
    try:
        __import__(module)
    except ImportError:
        missing.append(package)

if missing:
    print(f"Missing packages: {', '.join(missing)}")
    print(f"Install with: pip install {' '.join(missing)}")
    print("For ElevenLabs: pip install elevenlabs")
    sys.exit(1)

import typer
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

app = typer.Typer(help="Voice Controller for LanderPi Robot (runs on robot)")
console = Console()

# ============================================================================
# Configuration
# ============================================================================

# Audio settings for WonderEcho Pro
AUDIO_DEVICE = "hw:1,0"  # Card 1, Device 0
SAMPLE_RATE = 48000  # WonderEcho native rate
CHANNELS = 2  # Stereo required

# AWS settings
AWS_REGION = os.getenv("AWS_REGION", "us-east-1")
BEDROCK_MODEL = "us.anthropic.claude-haiku-4-5-20251001-v1:0"

# ElevenLabs settings
ELEVENLABS_API_KEY = os.getenv("ELEVENLABS_API_KEY", "")
ELEVENLABS_VOICE_ID = os.getenv("ELEVENLABS_VOICE_ID", "tTV9ysSyKVtziN1ESbZV")

# Whisper model (will be downloaded on first use)
WHISPER_MODEL = "base.en"  # Options: tiny.en, base.en, small.en

# Robot SDK path
SDK_PATH = Path.home() / "ros_robot_controller"

# Robot control prompt
ROBOT_CONTROL_PROMPT = """You are TARS, an efficient AI assistant controlling a LanderPi robot.
Be concise, professional, and direct. Address user as "sir" occasionally.

Available commands:
- Movement: forward, backward, turn_left, turn_right, strafe_left, strafe_right, stop
- Compound: look_around (360 scan), patrol (square pattern), come_here (approach user)
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
- "move backward slowly" → {"action": "movement", "command": "backward", "params": {"speed": 0.15, "duration": 2.0}, "response": "Reversing slowly."}
- "turn left fast" → {"action": "movement", "command": "turn_left", "params": {"speed": 0.5, "duration": 2.0}, "response": "Turning left."}
- "look around" → {"action": "compound", "command": "look_around", "params": {}, "response": "Scanning surroundings."}
- "patrol the area" → {"action": "compound", "command": "patrol", "params": {}, "response": "Beginning patrol."}
- "come here" → {"action": "compound", "command": "come_here", "params": {}, "response": "Approaching."}
- "follow me" → {"action": "mode", "command": "follow_me", "params": {"enable": true}, "response": "Following mode engaged."}
- "stop following" → {"action": "mode", "command": "follow_me", "params": {"enable": false}, "response": "Following mode disengaged."}
- "stop" → {"action": "stop", "command": "stop", "params": {}, "response": "All stop."}
- "hello" → {"action": "chat", "command": "none", "params": {}, "response": "Standing by, sir."}

Rules:
1. Output ONLY valid JSON, no markdown
2. Responses: professional, max 15 words
3. Unknown commands: use "chat" action"""


# ============================================================================
# ROS2 Movement Control
# ============================================================================

def execute_ros2_cmd_vel(linear_x: float, linear_y: float, angular_z: float, duration: float = 2.0) -> bool:
    """Publish velocity command via ROS2 docker container."""
    try:
        # Publish movement command
        twist_msg = (
            f"'{{linear: {{x: {linear_x}, y: {linear_y}, z: 0.0}}, "
            f"angular: {{x: 0.0, y: 0.0, z: {angular_z}}}}}'"
        )
        cmd = f'docker exec landerpi-ros2 ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist {twist_msg}'

        result = subprocess.run(cmd, shell=True, capture_output=True, timeout=5)
        if result.returncode != 0:
            console.print(f"[red]ROS2 command failed: {result.stderr.decode()}[/red]")
            return False

        # Wait for duration
        time.sleep(duration)

        # Stop command
        stop_msg = "'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
        stop_cmd = f'docker exec landerpi-ros2 ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist {stop_msg}'
        subprocess.run(stop_cmd, shell=True, capture_output=True, timeout=5)

        return True

    except subprocess.TimeoutExpired:
        console.print("[red]ROS2 command timeout[/red]")
        return False
    except Exception as e:
        console.print(f"[red]ROS2 error: {e}[/red]")
        return False


# ============================================================================
# Audio Recording
# ============================================================================

def record_audio(duration: float = 4.0, output_path: Optional[str] = None) -> Optional[str]:
    """Record audio from WonderEcho Pro microphone."""
    if output_path is None:
        output_path = tempfile.mktemp(suffix=".wav")

    console.print(f"[yellow]Recording {duration}s... Speak now![/yellow]")

    try:
        # Record stereo at native sample rate
        cmd = [
            "arecord",
            "-D", AUDIO_DEVICE,
            "-f", "S16_LE",
            "-r", str(SAMPLE_RATE),
            "-c", str(CHANNELS),
            "-d", str(int(duration)),
            output_path
        ]

        result = subprocess.run(cmd, capture_output=True, text=True, timeout=duration + 5)

        if result.returncode == 0 and Path(output_path).exists():
            console.print("[green]Recording complete[/green]")
            return output_path
        else:
            console.print(f"[red]Recording failed: {result.stderr}[/red]")
            return None

    except subprocess.TimeoutExpired:
        console.print("[red]Recording timeout[/red]")
        return None
    except Exception as e:
        console.print(f"[red]Recording error: {e}[/red]")
        return None


def play_audio(audio_path: str) -> bool:
    """Play audio through WonderEcho Pro speaker."""
    try:
        # Use aplay for local playback
        cmd = ["aplay", "-D", "plughw:1,0", audio_path]
        result = subprocess.run(cmd, capture_output=True, timeout=30)
        return result.returncode == 0
    except Exception as e:
        console.print(f"[red]Playback error: {e}[/red]")
        return False


# ============================================================================
# Speech-to-Text (Local Whisper)
# ============================================================================

_whisper_model = None

def get_whisper_model():
    """Load Whisper model (cached)."""
    global _whisper_model
    if _whisper_model is None:
        from faster_whisper import WhisperModel
        console.print(f"[yellow]Loading Whisper model ({WHISPER_MODEL})...[/yellow]")
        _whisper_model = WhisperModel(WHISPER_MODEL, device="cpu", compute_type="int8")
        console.print("[green]Whisper model loaded[/green]")
    return _whisper_model


def transcribe_audio(audio_path: str) -> Optional[str]:
    """Transcribe audio using local Whisper."""
    try:
        model = get_whisper_model()

        console.print("[yellow]Transcribing...[/yellow]")
        segments, info = model.transcribe(audio_path, language="en", beam_size=5)

        text = " ".join([segment.text for segment in segments]).strip()

        if text:
            console.print(f"[green]Heard: \"{text}\"[/green]")
            return text
        else:
            console.print("[dim]No speech detected[/dim]")
            return None

    except Exception as e:
        console.print(f"[red]Transcription error: {e}[/red]")
        return None


# ============================================================================
# AWS Bedrock (LLM)
# ============================================================================

def generate_robot_command(text: str) -> Optional[dict]:
    """Generate robot command using AWS Bedrock Claude Haiku."""
    import boto3

    console.print(f"[yellow]Processing: \"{text}\"[/yellow]")

    try:
        bedrock = boto3.client(
            service_name="bedrock-runtime",
            region_name=AWS_REGION
        )

        body = json.dumps({
            "anthropic_version": "bedrock-2023-05-31",
            "max_tokens": 150,
            "system": ROBOT_CONTROL_PROMPT,
            "messages": [{"role": "user", "content": text}]
        })

        response = bedrock.invoke_model(
            modelId=BEDROCK_MODEL,
            body=body,
            contentType="application/json",
            accept="application/json"
        )

        response_body = json.loads(response["body"].read())
        result_text = response_body["content"][0]["text"].strip()

        # Parse JSON
        try:
            # Clean up potential markdown
            if "```" in result_text:
                result_text = result_text.split("```")[1] if "```json" in result_text else result_text.split("```")[1]
                result_text = result_text.replace("json", "").strip()

            result = json.loads(result_text)
            console.print(f"[cyan]Action: {result.get('action')}/{result.get('command')}[/cyan]")
            return result

        except json.JSONDecodeError:
            # Treat as chat response
            return {
                "action": "chat",
                "command": "none",
                "params": {},
                "response": result_text[:80]
            }

    except Exception as e:
        console.print(f"[red]Bedrock error: {e}[/red]")
        return None


# ============================================================================
# Compound Commands
# ============================================================================

def execute_compound_command(cmd: str, params: dict) -> bool:
    """Execute compound movement sequences."""
    console.print(f"[yellow]Executing compound: {cmd}[/yellow]")

    if cmd == "look_around":
        # Rotate 360 degrees slowly (~7 seconds at 0.9 rad/s)
        return execute_ros2_cmd_vel(0.0, 0.0, 0.9, 7.0)

    elif cmd == "patrol":
        # Square pattern: forward, turn right, repeat 4x
        for i in range(4):
            console.print(f"[cyan]Patrol leg {i+1}/4[/cyan]")
            if not execute_ros2_cmd_vel(0.3, 0.0, 0.0, 2.0):  # Forward 2s
                return False
            time.sleep(0.5)
            if not execute_ros2_cmd_vel(0.0, 0.0, -0.5, 3.14):  # Turn right 90deg
                return False
            time.sleep(0.5)
        return True

    elif cmd == "come_here":
        # Simple: move forward toward user (future: use sound angle)
        return execute_ros2_cmd_vel(0.3, 0.0, 0.0, 3.0)

    console.print(f"[yellow]Unknown compound command: {cmd}[/yellow]")
    return False


# ============================================================================
# Robot Control (ROS2-based)
# ============================================================================

# Movement command mappings
MOVEMENT_COMMANDS = {
    "forward":      {"linear_x": 0.3,  "linear_y": 0.0,  "angular_z": 0.0},
    "backward":     {"linear_x": -0.3, "linear_y": 0.0,  "angular_z": 0.0},
    "turn_left":    {"linear_x": 0.0,  "linear_y": 0.0,  "angular_z": 0.5},
    "turn_right":   {"linear_x": 0.0,  "linear_y": 0.0,  "angular_z": -0.5},
    "strafe_left":  {"linear_x": 0.0,  "linear_y": 0.3,  "angular_z": 0.0},
    "strafe_right": {"linear_x": 0.0,  "linear_y": -0.3, "angular_z": 0.0},
    "stop":         {"linear_x": 0.0,  "linear_y": 0.0,  "angular_z": 0.0},
}

# Follow mode state
_follow_mode_active = False
_follow_thread = None


def follow_mode_loop():
    """Continuous following loop - moves forward slowly."""
    global _follow_mode_active
    console.print("[cyan]Follow mode active - moving forward slowly[/cyan]")

    while _follow_mode_active:
        # Publish slow forward movement
        execute_ros2_cmd_vel(0.15, 0.0, 0.0, 0.5)
        time.sleep(0.1)

    # Stop when mode disabled
    execute_ros2_cmd_vel(0.0, 0.0, 0.0, 0.1)
    console.print("[cyan]Follow mode stopped[/cyan]")


def toggle_follow_mode(enable: bool) -> bool:
    """Enable or disable follow mode."""
    global _follow_mode_active, _follow_thread
    import threading

    if enable and not _follow_mode_active:
        _follow_mode_active = True
        _follow_thread = threading.Thread(target=follow_mode_loop, daemon=True)
        _follow_thread.start()
        return True

    elif not enable and _follow_mode_active:
        _follow_mode_active = False
        if _follow_thread:
            _follow_thread.join(timeout=2.0)
        return True

    return True


def execute_robot_command(command: dict) -> bool:
    """Execute robot command via ROS2."""
    action = command.get("action", "chat")
    cmd = command.get("command", "none")
    params = command.get("params", {})

    if action == "chat" or cmd == "none":
        return True  # No physical action needed

    console.print(f"[yellow]Executing: {action}/{cmd}[/yellow]")

    # Get duration and speed multiplier from params
    duration = params.get("duration", 2.0)
    speed = params.get("speed", 0.3)

    if action == "movement" and cmd in MOVEMENT_COMMANDS:
        mapping = MOVEMENT_COMMANDS[cmd]
        # Apply speed multiplier
        linear_x = mapping["linear_x"] * (speed / 0.3) if mapping["linear_x"] != 0 else 0
        linear_y = mapping["linear_y"] * (speed / 0.3) if mapping["linear_y"] != 0 else 0
        angular_z = mapping["angular_z"] * (speed / 0.3) if mapping["angular_z"] != 0 else 0

        return execute_ros2_cmd_vel(linear_x, linear_y, angular_z, duration)

    elif action == "stop":
        return execute_ros2_cmd_vel(0.0, 0.0, 0.0, 0.1)

    elif action == "compound":
        return execute_compound_command(cmd, params)

    elif action == "mode":
        if cmd == "follow_me":
            enable = params.get("enable", True)
            return toggle_follow_mode(enable)

    console.print(f"[yellow]Unknown action: {action}/{cmd}[/yellow]")
    return True


# ============================================================================
# ElevenLabs TTS
# ============================================================================

def speak_response(text: str) -> bool:
    """Speak response using ElevenLabs TTS."""
    if not ELEVENLABS_API_KEY:
        console.print(f"[dim]TTS disabled (no API key). Response: {text}[/dim]")
        return False

    console.print(f"[yellow]Speaking: \"{text}\"[/yellow]")

    try:
        from elevenlabs.client import ElevenLabs

        client = ElevenLabs(api_key=ELEVENLABS_API_KEY)

        # Generate audio
        audio_generator = client.text_to_speech.convert(
            text=text,
            voice_id=ELEVENLABS_VOICE_ID,
            model_id="eleven_flash_v2_5",
            output_format="mp3_44100_128",
        )

        # Save to temp file
        temp_path = tempfile.mktemp(suffix=".mp3")
        with open(temp_path, "wb") as f:
            for chunk in audio_generator:
                f.write(chunk)

        # Convert to WAV for aplay (or use mpv/ffplay if available)
        wav_path = tempfile.mktemp(suffix=".wav")

        # Try ffmpeg conversion
        result = subprocess.run(
            ["ffmpeg", "-y", "-i", temp_path, "-ar", "48000", wav_path],
            capture_output=True,
            timeout=10
        )

        if result.returncode == 0:
            play_audio(wav_path)
            os.remove(wav_path)
        else:
            # Try mpv directly
            subprocess.run(["mpv", "--no-video", temp_path], capture_output=True, timeout=30)

        os.remove(temp_path)
        console.print("[green]Speech complete[/green]")
        return True

    except ImportError:
        console.print("[red]elevenlabs package not installed[/red]")
        return False
    except Exception as e:
        console.print(f"[red]TTS error: {e}[/red]")
        return False


# ============================================================================
# CLI Commands
# ============================================================================

@app.command()
def check():
    """Check all voice controller components."""
    console.print(Panel("Voice Controller Component Check", style="bold blue"))

    table = Table(show_header=True, header_style="bold magenta")
    table.add_column("Component", style="cyan")
    table.add_column("Status")
    table.add_column("Details")

    # Check audio device
    result = subprocess.run(["arecord", "-l"], capture_output=True, text=True)
    if "USB" in result.stdout or "card 1" in result.stdout:
        table.add_row("WonderEcho Pro", "[green]OK[/green]", "Audio device detected")
    else:
        table.add_row("WonderEcho Pro", "[red]MISSING[/red]", "Check USB connection")

    # Check Whisper
    try:
        from faster_whisper import WhisperModel
        table.add_row("faster-whisper", "[green]OK[/green]", f"Model: {WHISPER_MODEL}")
    except ImportError:
        table.add_row("faster-whisper", "[red]MISSING[/red]", "pip install faster-whisper")

    # Check AWS credentials
    try:
        import boto3
        sts = boto3.client('sts')
        identity = sts.get_caller_identity()
        table.add_row("AWS Credentials", "[green]OK[/green]", f"Account: ...{identity['Account'][-4:]}")
    except Exception as e:
        table.add_row("AWS Credentials", "[red]MISSING[/red]", "Run: aws configure")

    # Check Bedrock
    try:
        import boto3
        bedrock = boto3.client('bedrock-runtime', region_name=AWS_REGION)
        # Quick test
        table.add_row("AWS Bedrock", "[green]OK[/green]", f"Region: {AWS_REGION}")
    except Exception as e:
        table.add_row("AWS Bedrock", "[yellow]?[/yellow]", str(e)[:30])

    # Check ElevenLabs
    if ELEVENLABS_API_KEY:
        table.add_row("ElevenLabs", "[green]OK[/green]", f"Voice: ...{ELEVENLABS_VOICE_ID[-6:]}")
    else:
        table.add_row("ElevenLabs", "[yellow]DISABLED[/yellow]", "Set ELEVENLABS_API_KEY")

    # Check robot SDK
    sdk_file = SDK_PATH / "ros_robot_controller_sdk.py"
    if sdk_file.exists():
        table.add_row("Robot SDK", "[green]OK[/green]", str(SDK_PATH))
    else:
        table.add_row("Robot SDK", "[red]MISSING[/red]", f"Expected at {SDK_PATH}")

    # Check ffmpeg (for TTS playback)
    result = subprocess.run(["which", "ffmpeg"], capture_output=True)
    if result.returncode == 0:
        table.add_row("ffmpeg", "[green]OK[/green]", "For TTS audio conversion")
    else:
        table.add_row("ffmpeg", "[yellow]MISSING[/yellow]", "apt install ffmpeg")

    console.print(table)


@app.command()
def listen(
    duration: float = typer.Option(4.0, "--duration", "-d", help="Recording duration"),
):
    """Listen for a single voice command and execute it."""
    console.print(Panel("Voice Command", style="bold green"))

    # Record
    audio_path = record_audio(duration=duration)
    if not audio_path:
        raise typer.Exit(1)

    try:
        # Transcribe
        text = transcribe_audio(audio_path)
        if not text:
            console.print("[yellow]No speech detected[/yellow]")
            raise typer.Exit(0)

        # Generate command
        command = generate_robot_command(text)
        if not command:
            raise typer.Exit(1)

        # Execute
        execute_robot_command(command)

        # Speak response
        response = command.get("response", "Done, sir.")
        speak_response(response)

    finally:
        # Cleanup
        try:
            os.remove(audio_path)
        except:
            pass

    console.print("[bold green]Done![/bold green]")


@app.command()
def loop(
    duration: float = typer.Option(3.0, "--duration", "-d", help="Recording duration per cycle"),
    pause: float = typer.Option(0.5, "--pause", "-p", help="Pause between recordings"),
):
    """Continuous voice command loop."""
    console.print(Panel("Continuous Voice Control", style="bold green"))
    console.print("[cyan]Say 'stop listening' or press Ctrl+C to exit[/cyan]\n")

    # Pre-load Whisper model
    get_whisper_model()

    try:
        while True:
            console.print("\n" + "─" * 40)

            # Record
            audio_path = record_audio(duration=duration)
            if not audio_path:
                time.sleep(pause)
                continue

            try:
                # Transcribe
                text = transcribe_audio(audio_path)
                if not text or len(text.strip()) < 2:
                    time.sleep(pause)
                    continue

                # Check for exit command
                if "stop listening" in text.lower() or "exit" in text.lower():
                    speak_response("Voice control disabled, sir. Standing by.")
                    break

                # Generate and execute
                command = generate_robot_command(text)
                if command:
                    execute_robot_command(command)
                    response = command.get("response", "Done.")
                    speak_response(response)

            finally:
                try:
                    os.remove(audio_path)
                except:
                    pass

            time.sleep(pause)

    except KeyboardInterrupt:
        console.print("\n[yellow]Stopping...[/yellow]")
        speak_response("Voice control offline.")


@app.command("test-tts")
def test_tts(
    text: str = typer.Argument("Hello sir, TARS online and ready."),
):
    """Test ElevenLabs TTS."""
    speak_response(text)


@app.command("test-llm")
def test_llm(
    text: str = typer.Argument("move forward"),
):
    """Test LLM command generation (no execution)."""
    command = generate_robot_command(text)
    if command:
        console.print(Panel(json.dumps(command, indent=2), title="Generated Command"))


@app.command("test-record")
def test_record(
    duration: float = typer.Option(3.0, "--duration", "-d"),
):
    """Test audio recording and transcription."""
    audio_path = record_audio(duration=duration)
    if audio_path:
        text = transcribe_audio(audio_path)
        os.remove(audio_path)


@app.command("test-motors")
def test_motors():
    """Quick motor test."""
    try:
        board = get_board()
        console.print("[yellow]Testing motors (1 second forward)...[/yellow]")

        # Forward motion
        board.set_motor_speed([[1, -0.2], [2, -0.2], [3, 0.2], [4, 0.2]])
        time.sleep(1)
        board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

        console.print("[green]Motor test complete[/green]")
    except Exception as e:
        console.print(f"[red]Motor test failed: {e}[/red]")


if __name__ == "__main__":
    app()
