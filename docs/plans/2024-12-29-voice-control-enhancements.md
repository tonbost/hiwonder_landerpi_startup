# Voice Control Enhancements Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add working ROS2 motor control, wake word detection ("Hey TARS"), expanded commands, and systemd auto-start to the LanderPi voice controller.

**Architecture:** Replace direct SDK calls with ROS2 topic publishing via `docker exec`. Add continuous wake word detection using faster-whisper short samples. Create systemd service for boot persistence.

**Tech Stack:** Python 3, faster-whisper, AWS Bedrock, ElevenLabs, ROS2 (Docker), systemd

---

## Task 1: Create Systemd Directory and Activation Sound

**Files:**
- Create: `systemd/tars-voice.service`
- Create: `audio/beep.wav` (generate programmatically)

**Step 1: Create systemd directory**

```bash
mkdir -p systemd audio
```

**Step 2: Create beep sound file**

```bash
# Generate a simple beep using sox (or download a short beep)
# On robot: sudo apt install sox
# For now, create placeholder - will generate on robot
touch audio/beep.wav
```

**Step 3: Create systemd service file**

Create `systemd/tars-voice.service`:

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

**Step 4: Commit**

```bash
git add systemd/ audio/
git commit -m "feat: add systemd service and audio directory"
```

---

## Task 2: Update robot_voicecontroller.py - ROS2 Movement Function

**Files:**
- Modify: `robot_voicecontroller.py`

**Step 1: Add ROS2 command execution function**

Add after the configuration section (~line 90):

```python
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
```

**Step 2: Test ROS2 function manually on robot**

```bash
# SSH to robot and test
python3 -c "
import subprocess
cmd = '''docker exec landerpi-ros2 ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist \"'{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'\"'''
print(subprocess.run(cmd, shell=True, capture_output=True))
"
```

Expected: Robot moves forward briefly

**Step 3: Commit**

```bash
git add robot_voicecontroller.py
git commit -m "feat: add ROS2 cmd_vel execution function"
```

---

## Task 3: Update Movement Mappings to Use ROS2

**Files:**
- Modify: `robot_voicecontroller.py`

**Step 1: Replace execute_robot_command function**

Replace the existing `execute_robot_command` function (~line 200) with:

```python
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

    console.print(f"[yellow]Unknown action: {action}/{cmd}[/yellow]")
    return True
```

**Step 2: Test movement via voice controller**

```bash
python3 robot_voicecontroller.py test-llm "go forward"
# Should show: action=movement, command=forward
```

**Step 3: Commit**

```bash
git add robot_voicecontroller.py
git commit -m "feat: replace SDK with ROS2 movement commands"
```

---

## Task 4: Add Compound Commands (look_around, patrol, come_here)

**Files:**
- Modify: `robot_voicecontroller.py`

**Step 1: Add compound command functions**

Add after `execute_ros2_cmd_vel` function:

```python
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
            if not execute_ros2_cmd_vel(0.0, 0.0, -0.5, 3.14):  # Turn right 90°
                return False
            time.sleep(0.5)
        return True

    elif cmd == "come_here":
        # Simple: move forward toward user (future: use sound angle)
        return execute_ros2_cmd_vel(0.3, 0.0, 0.0, 3.0)

    console.print(f"[yellow]Unknown compound command: {cmd}[/yellow]")
    return False
```

**Step 2: Update execute_robot_command to handle compound**

Add to `execute_robot_command` function, after the movement handling:

```python
    elif action == "compound":
        return execute_compound_command(cmd, params)
```

**Step 3: Commit**

```bash
git add robot_voicecontroller.py
git commit -m "feat: add compound commands (look_around, patrol, come_here)"
```

---

## Task 5: Add Follow Mode (Continuous Tracking)

**Files:**
- Modify: `robot_voicecontroller.py`

**Step 1: Add follow mode state and function**

Add at module level (after MOVEMENT_COMMANDS):

```python
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
```

**Step 2: Update execute_robot_command for mode actions**

Add to `execute_robot_command` function:

```python
    elif action == "mode":
        if cmd == "follow_me":
            enable = params.get("enable", True)
            return toggle_follow_mode(enable)
```

**Step 3: Commit**

```bash
git add robot_voicecontroller.py
git commit -m "feat: add follow_me mode (continuous tracking)"
```

---

## Task 6: Update TARS Prompt

**Files:**
- Modify: `robot_voicecontroller.py`

**Step 1: Replace ROBOT_CONTROL_PROMPT**

Replace the existing prompt (~line 92) with:

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
3. Unknown commands: use "chat" action
"""
```

**Step 2: Test new prompt**

```bash
python3 robot_voicecontroller.py test-llm "look around and then patrol"
# Should return compound command
```

**Step 3: Commit**

```bash
git add robot_voicecontroller.py
git commit -m "feat: update TARS prompt - professional + new commands"
```

---

## Task 7: Add Wake Word Detection

**Files:**
- Modify: `robot_voicecontroller.py`

**Step 1: Add activation sound function**

Add after audio recording section:

```python
ACTIVATION_SOUND = Path.home() / "audio" / "beep.wav"


def play_activation_sound():
    """Play short beep to confirm wake word detected."""
    if ACTIVATION_SOUND.exists():
        try:
            subprocess.run(["aplay", "-q", str(ACTIVATION_SOUND)], timeout=2)
        except:
            pass
    else:
        # Fallback: print message
        console.print("[bold green]* ACTIVATED *[/bold green]")
```

**Step 2: Add wake word detection function**

Add after `play_activation_sound`:

```python
def wait_for_wake_word(wake_phrase: str = "hey tars", timeout: Optional[float] = None) -> bool:
    """
    Listen continuously for wake phrase.

    Args:
        wake_phrase: Phrase to listen for (default: "hey tars")
        timeout: Optional timeout in seconds (None = forever)

    Returns:
        True if wake word detected, False if timeout
    """
    console.print(f"[dim]Listening for '{wake_phrase}'...[/dim]")
    start_time = time.time()

    while True:
        # Check timeout
        if timeout and (time.time() - start_time) > timeout:
            return False

        # Record short clip
        audio_path = record_audio(duration=1.5)
        if not audio_path:
            time.sleep(0.1)
            continue

        try:
            # Quick transcription
            text = transcribe_audio(audio_path)

            if text and wake_phrase in text.lower():
                play_activation_sound()
                console.print("[bold green]Wake word detected![/bold green]")
                return True

        finally:
            # Cleanup temp file
            try:
                os.remove(audio_path)
            except:
                pass

        # Small pause
        time.sleep(0.05)
```

**Step 3: Commit**

```bash
git add robot_voicecontroller.py
git commit -m "feat: add wake word detection function"
```

---

## Task 8: Update Loop Command with Wake Word Mode

**Files:**
- Modify: `robot_voicecontroller.py`

**Step 1: Update loop command**

Replace the existing `loop` command function:

```python
@app.command()
def loop(
    duration: float = typer.Option(4.0, "--duration", "-d", help="Recording duration per command"),
    pause: float = typer.Option(0.5, "--pause", "-p", help="Pause between recordings"),
    wake_word: bool = typer.Option(False, "--wake-word", "-w", help="Require 'Hey TARS' to activate"),
):
    """Continuous voice command loop."""
    console.print(Panel("TARS Voice Control", style="bold green"))

    if wake_word:
        console.print("[cyan]Wake word mode: Say 'Hey TARS' to activate[/cyan]")
    else:
        console.print("[cyan]Continuous mode: Always listening[/cyan]")

    console.print("[dim]Say 'stop listening' or 'goodbye' to exit[/dim]\n")

    # Pre-load Whisper model
    get_whisper_model()

    try:
        while True:
            # Wake word mode: wait for "Hey TARS"
            if wake_word:
                if not wait_for_wake_word("hey tars"):
                    continue

            console.print("\n" + "─" * 40)

            # Record command
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
                text_lower = text.lower()
                if "stop listening" in text_lower or "goodbye" in text_lower or "exit" in text_lower:
                    speak_response("Voice control offline. Goodbye, sir.")
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
        console.print("\n[yellow]Interrupted[/yellow]")
        # Stop any active follow mode
        toggle_follow_mode(False)
        speak_response("Voice control offline.")
```

**Step 2: Commit**

```bash
git add robot_voicecontroller.py
git commit -m "feat: add --wake-word flag to loop command"
```

---

## Task 9: Update Deployment Script for Service Installation

**Files:**
- Modify: `deploy_voicecontroller.py`

**Step 1: Add service installation command**

Add new command to `deploy_voicecontroller.py`:

```python
@app.command("install-service")
def install_service():
    """Install and enable systemd service on robot."""
    console.print("[yellow]Installing systemd service...[/yellow]")

    conn = get_connection()
    config = load_config()

    # Upload service file
    service_file = Path(__file__).parent / "systemd" / "tars-voice.service"
    if not service_file.exists():
        console.print("[red]Service file not found: systemd/tars-voice.service[/red]")
        raise typer.Exit(1)

    conn.put(str(service_file), "/tmp/tars-voice.service")

    # Install service
    conn.sudo("cp /tmp/tars-voice.service /etc/systemd/system/", hide=True)
    conn.sudo("systemctl daemon-reload", hide=True)
    conn.sudo("systemctl enable tars-voice", hide=True)

    console.print("[green]Service installed and enabled[/green]")
    console.print("\nManagement commands:")
    console.print("  [cyan]sudo systemctl start tars-voice[/cyan]   # Start now")
    console.print("  [cyan]sudo systemctl stop tars-voice[/cyan]    # Stop")
    console.print("  [cyan]sudo systemctl status tars-voice[/cyan]  # Check status")
    console.print("  [cyan]journalctl -u tars-voice -f[/cyan]       # View logs")
```

**Step 2: Update deploy command to include service**

Update the `deploy` command to call `install_service`:

```python
@app.command()
def deploy():
    """Full deployment: upload script and install dependencies."""
    console.print(Panel("Deploying Voice Controller to Robot", style="bold blue"))

    upload()
    install()
    configure()
    install_service()

    console.print("\n[bold green]Deployment complete![/bold green]")
    console.print("\nTo start voice control:")
    console.print("  [cyan]sudo systemctl start tars-voice[/cyan]")
    console.print("\nOr run manually:")
    console.print("  [cyan]python3 ~/robot_voicecontroller.py loop --wake-word[/cyan]")
```

**Step 3: Commit**

```bash
git add deploy_voicecontroller.py
git commit -m "feat: add systemd service installation to deployment"
```

---

## Task 10: Create Beep Sound on Robot

**Files:**
- Robot: `~/audio/beep.wav`

**Step 1: Add beep generation to deployment**

Add to `deploy_voicecontroller.py` configure command:

```python
    # Create audio directory and beep sound
    conn.run("mkdir -p ~/audio", hide=True)

    # Generate beep using sox if available, otherwise use simple approach
    result = conn.run("which sox", hide=True, warn=True)
    if result.return_code == 0:
        conn.run("sox -n ~/audio/beep.wav synth 0.1 sine 880 vol 0.5", hide=True)
        console.print("[green]Generated activation beep sound[/green]")
    else:
        console.print("[yellow]sox not installed - beep sound skipped[/yellow]")
        console.print("  Install with: sudo apt install sox")
```

**Step 2: Commit**

```bash
git add deploy_voicecontroller.py
git commit -m "feat: add beep sound generation to deployment"
```

---

## Task 11: Final Integration Test

**Step 1: Deploy to robot**

```bash
uv run python deploy_voicecontroller.py deploy
```

**Step 2: SSH to robot and test components**

```bash
ssh tonbost@192.168.50.169

# Check components
python3 ~/robot_voicecontroller.py check

# Test TTS
python3 ~/robot_voicecontroller.py test-tts "TARS online."

# Test movement
python3 ~/robot_voicecontroller.py test-llm "go forward"
# Then actually execute:
python3 ~/robot_voicecontroller.py listen
# Say "go forward"

# Test wake word mode
python3 ~/robot_voicecontroller.py loop --wake-word
# Say "Hey TARS" then "look around"
```

**Step 3: Test systemd service**

```bash
# Start service
sudo systemctl start tars-voice

# Check status
sudo systemctl status tars-voice

# View logs
journalctl -u tars-voice -f

# Test: say "Hey TARS" followed by "go forward"

# Stop service
sudo systemctl stop tars-voice
```

**Step 4: Test boot persistence**

```bash
# Reboot robot
sudo reboot

# After reboot, check service started
sudo systemctl status tars-voice

# Test voice command
# Say "Hey TARS" then "patrol"
```

---

## Summary

| Task | Description | Files |
|------|-------------|-------|
| 1 | Create systemd directory and activation sound | `systemd/`, `audio/` |
| 2 | Add ROS2 cmd_vel execution function | `robot_voicecontroller.py` |
| 3 | Update movement mappings to use ROS2 | `robot_voicecontroller.py` |
| 4 | Add compound commands | `robot_voicecontroller.py` |
| 5 | Add follow mode | `robot_voicecontroller.py` |
| 6 | Update TARS prompt | `robot_voicecontroller.py` |
| 7 | Add wake word detection | `robot_voicecontroller.py` |
| 8 | Update loop with wake word mode | `robot_voicecontroller.py` |
| 9 | Update deployment for service | `deploy_voicecontroller.py` |
| 10 | Create beep sound on robot | `deploy_voicecontroller.py` |
| 11 | Final integration test | - |

**Total estimated time**: 45-60 minutes
