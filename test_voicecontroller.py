#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "typer",
#     "rich",
#     "fabric",
#     "python-dotenv",
#     "boto3",
#     "elevenlabs",
# ]
# ///
"""
Voice Controller for LanderPi Robot

Uses WonderEcho Pro for audio capture, AWS services for ASR/LLM,
and ElevenLabs for TTS response.

Architecture:
    Robot (WonderEcho Pro) → SSH → Local Machine
    1. Record audio on robot (arecord)
    2. Transfer WAV file via SFTP
    3. AWS Transcribe for ASR
    4. AWS Bedrock Claude Haiku 4.5 for command generation
    5. Execute robot command via SSH
    6. ElevenLabs TTS for spoken response

Usage:
    uv run python test_voicecontroller.py listen          # Single voice command
    uv run python test_voicecontroller.py loop            # Continuous listening
    uv run python test_voicecontroller.py check           # Check all components
    uv run python test_voicecontroller.py test-tts "Hi"   # Test TTS only
    uv run python test_voicecontroller.py test-llm "move forward"  # Test LLM only
"""

import json
import os
import sys
import tempfile
import time
from pathlib import Path
from typing import Optional

import typer
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass

app = typer.Typer(help="Voice Controller for LanderPi Robot")
console = Console()

# Robot connection config
CONFIG_FILE = Path(__file__).parent / "config.json"

# Robot control prompt for Claude Haiku
ROBOT_CONTROL_PROMPT = """You are TARS, a witty AI controlling a LanderPi robot. Your job is to interpret voice commands and generate robot control actions.

Available robot capabilities:
- Movement: forward, backward, turn_left, turn_right, strafe_left, strafe_right, stop
- Arm: home, pick, place, wave
- Speed: 0.1 to 0.5 m/s (default 0.3)
- Duration: 0.5 to 5.0 seconds (default 2.0)

Output JSON format:
{
    "action": "movement" | "arm" | "stop" | "chat",
    "command": "<specific command>",
    "params": {"speed": 0.3, "duration": 2.0},
    "response": "<brief witty TARS-style response, under 25 words>"
}

Examples:
- "go forward" → {"action": "movement", "command": "forward", "params": {"speed": 0.3, "duration": 2.0}, "response": "Moving forward, sir. Try not to crash us."}
- "turn left slowly" → {"action": "movement", "command": "turn_left", "params": {"speed": 0.15, "duration": 2.0}, "response": "Turning left. Slowly, as requested."}
- "move the arm home" → {"action": "arm", "command": "home", "params": {}, "response": "Arm returning home, sir."}
- "what's the weather" → {"action": "chat", "command": "none", "params": {}, "response": "I'm a robot, sir. I control motors, not meteorology."}
- "stop" → {"action": "stop", "command": "stop", "params": {}, "response": "All stop. Standing by."}

Rules:
1. Always output valid JSON
2. Keep responses brief and witty (TARS personality)
3. If command is unclear, use "chat" action and ask for clarification
4. Address user as "sir"
5. NO markdown formatting in response"""


def load_config() -> dict:
    """Load robot connection config."""
    if CONFIG_FILE.exists():
        with open(CONFIG_FILE) as f:
            return json.load(f)
    return {}


def get_ssh_connection():
    """Create SSH connection to robot using Fabric."""
    from fabric import Connection

    config = load_config()
    if not config:
        raise ValueError("No config.json found. Create one with host, user, password.")

    return Connection(
        host=config["host"],
        user=config["user"],
        connect_kwargs={"password": config["password"]}
    )


# ============================================================================
# Audio Recording (on robot)
# ============================================================================

def record_audio_on_robot(duration: float = 5.0, output_path: str = "/tmp/voice_cmd.wav") -> bool:
    """Record audio on robot using arecord."""
    console.print(f"[yellow]Recording {duration}s of audio on robot...[/yellow]")

    try:
        conn = get_ssh_connection()

        # Record using card 1 (WonderEcho Pro)
        # Format: 16-bit signed LE, 16kHz, mono
        cmd = f"arecord -D hw:1,0 -f S16_LE -r 16000 -c 1 -d {int(duration)} {output_path}"
        result = conn.run(cmd, hide=True, warn=True)

        if result.return_code != 0:
            console.print(f"[red]Recording failed: {result.stderr}[/red]")
            return False

        console.print("[green]Recording complete![/green]")
        return True

    except Exception as e:
        console.print(f"[red]SSH error: {e}[/red]")
        return False


def download_audio_from_robot(remote_path: str = "/tmp/voice_cmd.wav") -> Optional[str]:
    """Download recorded audio from robot."""
    try:
        conn = get_ssh_connection()

        # Create local temp file
        local_path = tempfile.mktemp(suffix=".wav")

        # Download via SFTP
        conn.get(remote_path, local_path)

        console.print(f"[green]Downloaded audio to {local_path}[/green]")
        return local_path

    except Exception as e:
        console.print(f"[red]Download failed: {e}[/red]")
        return None


# ============================================================================
# AWS Transcribe (ASR)
# ============================================================================

def transcribe_audio(audio_path: str) -> Optional[str]:
    """Transcribe audio using AWS Transcribe."""
    import boto3
    import uuid

    console.print("[yellow]Transcribing audio with AWS Transcribe...[/yellow]")

    try:
        # Upload to S3 first (Transcribe needs S3 URI)
        s3 = boto3.client('s3')
        transcribe = boto3.client('transcribe', region_name=os.getenv("AWS_REGION", "us-east-1"))

        # Use a bucket for transcription (create if needed)
        bucket_name = os.getenv("AWS_TRANSCRIBE_BUCKET", "landerpi-voice-temp")
        job_name = f"voice-cmd-{uuid.uuid4().hex[:8]}"
        s3_key = f"voice-input/{job_name}.wav"

        # Upload audio file
        s3.upload_file(audio_path, bucket_name, s3_key)
        s3_uri = f"s3://{bucket_name}/{s3_key}"

        # Start transcription job
        transcribe.start_transcription_job(
            TranscriptionJobName=job_name,
            Media={'MediaFileUri': s3_uri},
            MediaFormat='wav',
            LanguageCode='en-US',
            Settings={
                'ShowSpeakerLabels': False,
            }
        )

        # Wait for completion
        while True:
            status = transcribe.get_transcription_job(TranscriptionJobName=job_name)
            job_status = status['TranscriptionJob']['TranscriptionJobStatus']

            if job_status == 'COMPLETED':
                # Get transcript
                transcript_uri = status['TranscriptionJob']['Transcript']['TranscriptFileUri']

                import urllib.request
                with urllib.request.urlopen(transcript_uri) as response:
                    transcript_data = json.loads(response.read().decode())
                    text = transcript_data['results']['transcripts'][0]['transcript']

                # Cleanup
                s3.delete_object(Bucket=bucket_name, Key=s3_key)
                transcribe.delete_transcription_job(TranscriptionJobName=job_name)

                console.print(f"[green]Transcribed: {text}[/green]")
                return text

            elif job_status == 'FAILED':
                console.print("[red]Transcription failed[/red]")
                return None

            time.sleep(0.5)

    except Exception as e:
        console.print(f"[red]Transcription error: {e}[/red]")
        # Fallback: try local whisper if available
        return transcribe_audio_local(audio_path)


def transcribe_audio_local(audio_path: str) -> Optional[str]:
    """Fallback: transcribe using local whisper if AWS fails."""
    try:
        # Try faster-whisper if installed
        from faster_whisper import WhisperModel

        console.print("[yellow]Using local Whisper model...[/yellow]")
        model = WhisperModel("base.en", device="cpu", compute_type="int8")
        segments, _ = model.transcribe(audio_path, language="en")
        text = " ".join([seg.text for seg in segments]).strip()

        console.print(f"[green]Transcribed (local): {text}[/green]")
        return text

    except ImportError:
        console.print("[red]No ASR available. Install faster-whisper or configure AWS Transcribe.[/red]")
        return None
    except Exception as e:
        console.print(f"[red]Local transcription error: {e}[/red]")
        return None


# ============================================================================
# AWS Bedrock (LLM)
# ============================================================================

def generate_robot_command(text: str) -> Optional[dict]:
    """Generate robot command using AWS Bedrock Claude Haiku 4.5."""
    import boto3

    console.print(f"[yellow]Processing command with Claude Haiku: '{text}'[/yellow]")

    try:
        bedrock = boto3.client(
            service_name="bedrock-runtime",
            region_name=os.getenv("AWS_REGION", "us-east-1")
        )

        body = json.dumps({
            "anthropic_version": "bedrock-2023-05-31",
            "max_tokens": 200,
            "system": ROBOT_CONTROL_PROMPT,
            "messages": [{"role": "user", "content": text}]
        })

        response = bedrock.invoke_model(
            modelId="us.anthropic.claude-haiku-4-5-20251001-v1:0",
            body=body,
            contentType="application/json",
            accept="application/json"
        )

        response_body = json.loads(response["body"].read())
        result_text = response_body["content"][0]["text"].strip()

        # Parse JSON from response
        try:
            # Handle potential markdown code blocks
            if "```json" in result_text:
                result_text = result_text.split("```json")[1].split("```")[0]
            elif "```" in result_text:
                result_text = result_text.split("```")[1].split("```")[0]

            result = json.loads(result_text)
            console.print(f"[green]Command: {result}[/green]")
            return result

        except json.JSONDecodeError:
            console.print(f"[yellow]Raw response (not JSON): {result_text}[/yellow]")
            return {
                "action": "chat",
                "command": "none",
                "params": {},
                "response": result_text[:100]
            }

    except Exception as e:
        console.print(f"[red]Bedrock error: {e}[/red]")
        return None


# ============================================================================
# Robot Command Execution
# ============================================================================

def execute_robot_command(command: dict) -> bool:
    """Execute robot command via SSH."""
    action = command.get("action", "chat")
    cmd = command.get("command", "none")
    params = command.get("params", {})

    if action == "chat" or cmd == "none":
        console.print("[cyan]No robot action needed (chat only)[/cyan]")
        return True

    console.print(f"[yellow]Executing: {action}/{cmd} with {params}[/yellow]")

    try:
        conn = get_ssh_connection()
        config = load_config()

        # Build command based on action type
        if action == "movement":
            speed = params.get("speed", 0.3)
            duration = params.get("duration", 2.0)

            # Map command to test_chassis_direct.py direction
            direction_map = {
                "forward": "forward",
                "backward": "backward",
                "turn_left": "turn_left",
                "turn_right": "turn_right",
                "strafe_left": "strafe_left",
                "strafe_right": "strafe_right",
            }

            direction = direction_map.get(cmd, "forward")

            # Execute via test_chassis_direct.py
            ssh_cmd = (
                f"cd ~/landerpi && "
                f"python3 -c \""
                f"import sys; sys.path.insert(0, '.'); "
                f"from ros_robot_controller_sdk import Board; "
                f"import time; "
                f"board = Board(); "
                f"board.set_motor_speed([[1, {-speed if 'left' not in cmd else speed}], "
                f"[2, {-speed if 'left' not in cmd else speed}], "
                f"[3, {speed if 'left' not in cmd else -speed}], "
                f"[4, {speed if 'left' not in cmd else -speed}]]); "
                f"time.sleep({duration}); "
                f"board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])"
                f"\""
            )

            # Simpler: use test_chassis_direct.py if available
            ssh_cmd = (
                f"cd /home/{config['user']} && "
                f"python3 test_chassis_direct.py test --direction {direction} --duration {duration} --yes 2>/dev/null || "
                f"echo 'Using direct SDK...' && "
                f"python3 -c \"from ros_robot_controller_sdk import Board; import time; "
                f"b=Board(); b.set_motor_speed([[1,{-0.3 if direction in ['forward','strafe_right'] else 0.3}],"
                f"[2,{-0.3 if direction in ['forward','strafe_left'] else 0.3}],"
                f"[3,{0.3 if direction in ['forward','strafe_left'] else -0.3}],"
                f"[4,{0.3 if direction in ['forward','strafe_right'] else -0.3}]]); "
                f"time.sleep({duration}); b.set_motor_speed([[1,0],[2,0],[3,0],[4,0]])\""
            )

        elif action == "arm":
            if cmd == "home":
                ssh_cmd = f"cd /home/{config['user']} && python3 test_arm.py home --yes 2>/dev/null || echo 'Arm home'"
            else:
                ssh_cmd = f"echo 'Arm command: {cmd}'"

        elif action == "stop":
            ssh_cmd = (
                f"cd /home/{config['user']} && "
                f"python3 test_chassis_direct.py stop 2>/dev/null || "
                f"python3 -c \"from ros_robot_controller_sdk import Board; Board().set_motor_speed([[1,0],[2,0],[3,0],[4,0]])\""
            )
        else:
            console.print(f"[yellow]Unknown action: {action}[/yellow]")
            return True

        result = conn.run(ssh_cmd, hide=True, warn=True)

        if result.return_code == 0:
            console.print("[green]Command executed successfully[/green]")
            return True
        else:
            console.print(f"[yellow]Command output: {result.stdout} {result.stderr}[/yellow]")
            return True  # Don't fail on warnings

    except Exception as e:
        console.print(f"[red]Execution error: {e}[/red]")
        return False


# ============================================================================
# ElevenLabs TTS
# ============================================================================

def speak_response(text: str) -> bool:
    """Speak response using ElevenLabs TTS."""
    console.print(f"[yellow]Speaking: {text}[/yellow]")

    try:
        from elevenlabs.client import ElevenLabs
        from elevenlabs import play

        api_key = os.getenv("ELEVENLABS_API_KEY")
        voice_id = os.getenv("ELEVENLABS_VOICE_ID", "RXtWW6etvimS8QJ5nhVk")

        if not api_key:
            console.print("[red]ELEVENLABS_API_KEY not set[/red]")
            return False

        client = ElevenLabs(api_key=api_key)

        audio = client.text_to_speech.convert(
            text=text,
            voice_id=voice_id,
            model_id="eleven_flash_v2_5",
            output_format="mp3_44100_128",
        )

        play(audio)
        console.print("[green]Speech complete[/green]")
        return True

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
    table.add_column("Status", style="green")
    table.add_column("Details")

    # Check config
    config = load_config()
    if config:
        table.add_row("Config", "[green]OK[/green]", f"Host: {config.get('host', 'N/A')}")
    else:
        table.add_row("Config", "[red]MISSING[/red]", "Create config.json")

    # Check SSH connection
    try:
        conn = get_ssh_connection()
        result = conn.run("echo 'connected'", hide=True)
        table.add_row("SSH Connection", "[green]OK[/green]", "Robot reachable")
    except Exception as e:
        table.add_row("SSH Connection", "[red]FAILED[/red]", str(e)[:40])

    # Check audio device on robot
    try:
        conn = get_ssh_connection()
        result = conn.run("arecord -l | grep -i 'USB'", hide=True, warn=True)
        if "USB" in result.stdout:
            table.add_row("WonderEcho Pro", "[green]OK[/green]", "USB Audio detected")
        else:
            table.add_row("WonderEcho Pro", "[yellow]?[/yellow]", "Check connection")
    except:
        table.add_row("WonderEcho Pro", "[red]FAILED[/red]", "SSH error")

    # Check AWS credentials
    try:
        import boto3
        sts = boto3.client('sts')
        identity = sts.get_caller_identity()
        table.add_row("AWS Credentials", "[green]OK[/green]", f"Account: {identity['Account'][:8]}...")
    except Exception as e:
        table.add_row("AWS Credentials", "[red]MISSING[/red]", "Configure AWS CLI")

    # Check Bedrock access
    try:
        import boto3
        bedrock = boto3.client('bedrock-runtime', region_name=os.getenv("AWS_REGION", "us-east-1"))
        table.add_row("AWS Bedrock", "[green]OK[/green]", "Client created")
    except Exception as e:
        table.add_row("AWS Bedrock", "[yellow]?[/yellow]", str(e)[:40])

    # Check ElevenLabs
    api_key = os.getenv("ELEVENLABS_API_KEY")
    voice_id = os.getenv("ELEVENLABS_VOICE_ID")
    if api_key:
        table.add_row("ElevenLabs API", "[green]OK[/green]", f"Voice: {voice_id[:8]}..." if voice_id else "Default voice")
    else:
        table.add_row("ElevenLabs API", "[red]MISSING[/red]", "Set ELEVENLABS_API_KEY")

    console.print(table)


@app.command()
def listen(
    duration: float = typer.Option(5.0, "--duration", "-d", help="Recording duration in seconds"),
):
    """Listen for a single voice command and execute it."""
    console.print(Panel("Voice Command Mode", style="bold green"))

    # Step 1: Record audio
    console.print("\n[bold]Step 1: Recording...[/bold]")
    console.print("[cyan]Speak your command now![/cyan]")

    if not record_audio_on_robot(duration=duration):
        console.print("[red]Recording failed[/red]")
        raise typer.Exit(1)

    # Step 2: Download audio
    console.print("\n[bold]Step 2: Downloading audio...[/bold]")
    local_audio = download_audio_from_robot()
    if not local_audio:
        console.print("[red]Download failed[/red]")
        raise typer.Exit(1)

    # Step 3: Transcribe
    console.print("\n[bold]Step 3: Transcribing...[/bold]")
    text = transcribe_audio_local(local_audio)  # Use local whisper first (faster)
    if not text:
        console.print("[red]Transcription failed[/red]")
        raise typer.Exit(1)

    # Step 4: Generate command
    console.print("\n[bold]Step 4: Processing command...[/bold]")
    command = generate_robot_command(text)
    if not command:
        console.print("[red]Command generation failed[/red]")
        raise typer.Exit(1)

    # Step 5: Execute
    console.print("\n[bold]Step 5: Executing...[/bold]")
    execute_robot_command(command)

    # Step 6: Speak response
    console.print("\n[bold]Step 6: Speaking response...[/bold]")
    response = command.get("response", "Command executed, sir.")
    speak_response(response)

    # Cleanup
    try:
        os.remove(local_audio)
    except:
        pass

    console.print("\n[bold green]Done![/bold green]")


@app.command()
def loop(
    duration: float = typer.Option(4.0, "--duration", "-d", help="Recording duration per command"),
    wake_word: str = typer.Option("", "--wake", "-w", help="Optional wake word to wait for"),
):
    """Continuous voice command loop."""
    console.print(Panel("Continuous Voice Control Mode", style="bold green"))
    console.print("[cyan]Press Ctrl+C to stop[/cyan]\n")

    try:
        while True:
            console.print("\n" + "="*50)
            console.print("[bold yellow]Listening...[/bold yellow]")

            # Record
            if not record_audio_on_robot(duration=duration):
                time.sleep(1)
                continue

            # Download
            local_audio = download_audio_from_robot()
            if not local_audio:
                time.sleep(1)
                continue

            # Transcribe
            text = transcribe_audio_local(local_audio)
            if not text or len(text.strip()) < 2:
                console.print("[dim]No speech detected[/dim]")
                try:
                    os.remove(local_audio)
                except:
                    pass
                time.sleep(0.5)
                continue

            # Check wake word if specified
            if wake_word and wake_word.lower() not in text.lower():
                console.print(f"[dim]Wake word '{wake_word}' not detected[/dim]")
                try:
                    os.remove(local_audio)
                except:
                    pass
                continue

            # Generate and execute command
            command = generate_robot_command(text)
            if command:
                execute_robot_command(command)
                response = command.get("response", "Done, sir.")
                speak_response(response)

            # Cleanup
            try:
                os.remove(local_audio)
            except:
                pass

    except KeyboardInterrupt:
        console.print("\n[yellow]Stopping voice control...[/yellow]")
        speak_response("Voice control disabled, sir. Standing by.")


@app.command("test-tts")
def test_tts(
    text: str = typer.Argument("Hello sir, TARS online and ready for your commands."),
):
    """Test ElevenLabs TTS."""
    console.print(Panel("TTS Test", style="bold blue"))
    speak_response(text)


@app.command("test-llm")
def test_llm(
    text: str = typer.Argument("move forward for 2 seconds"),
):
    """Test LLM command generation."""
    console.print(Panel("LLM Test", style="bold blue"))

    command = generate_robot_command(text)
    if command:
        console.print(Panel(json.dumps(command, indent=2), title="Generated Command"))

        # Ask to execute
        if typer.confirm("Execute this command?"):
            execute_robot_command(command)
            speak_response(command.get("response", "Done."))


@app.command("test-record")
def test_record(
    duration: float = typer.Option(3.0, "--duration", "-d", help="Recording duration"),
):
    """Test audio recording on robot."""
    console.print(Panel("Recording Test", style="bold blue"))

    if record_audio_on_robot(duration=duration):
        local_audio = download_audio_from_robot()
        if local_audio:
            console.print(f"[green]Audio saved to: {local_audio}[/green]")

            # Try to transcribe
            text = transcribe_audio_local(local_audio)
            if text:
                console.print(f"[green]Transcription: {text}[/green]")


if __name__ == "__main__":
    app()
