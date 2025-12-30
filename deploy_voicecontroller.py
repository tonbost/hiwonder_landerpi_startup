#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "typer",
#     "rich",
#     "fabric",
# ]
# ///
"""
Deploy Voice Controller to LanderPi Robot

Uploads the voice controller script and sets up dependencies on the robot.

Usage:
    uv run python deploy_voicecontroller.py deploy     # Full deployment
    uv run python deploy_voicecontroller.py install    # Install deps only
    uv run python deploy_voicecontroller.py upload     # Upload script only
    uv run python deploy_voicecontroller.py configure  # Configure AWS/ElevenLabs
"""

import json
import os
from pathlib import Path

import typer
from fabric import Connection
from rich.console import Console
from rich.panel import Panel

app = typer.Typer(help="Deploy Voice Controller to LanderPi Robot")
console = Console()

CONFIG_FILE = Path(__file__).parent / "config.json"
VOICE_SCRIPT = Path(__file__).parent / "robot_voicecontroller.py"


def load_config() -> dict:
    """Load robot connection config."""
    if CONFIG_FILE.exists():
        with open(CONFIG_FILE) as f:
            return json.load(f)
    raise ValueError("config.json not found")


def get_connection() -> Connection:
    """Get SSH connection to robot."""
    config = load_config()
    return Connection(
        host=config["host"],
        user=config["user"],
        connect_kwargs={"password": config["password"]}
    )


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


@app.command()
def upload():
    """Upload voice controller script to robot."""
    console.print("[yellow]Uploading script...[/yellow]")

    conn = get_connection()
    config = load_config()

    # Upload main script
    remote_path = f"/home/{config['user']}/robot_voicecontroller.py"
    conn.put(str(VOICE_SCRIPT), remote_path)
    conn.run(f"chmod +x {remote_path}")

    console.print(f"[green]Uploaded to {remote_path}[/green]")


@app.command()
def install():
    """Install dependencies on robot."""
    console.print("[yellow]Installing dependencies on robot...[/yellow]")

    conn = get_connection()

    # Install system packages
    console.print("  Installing system packages...")
    conn.sudo("apt-get update -qq", hide=True, warn=True)
    conn.sudo("apt-get install -y -qq ffmpeg python3-pip", hide=True, warn=True)

    # Install Python packages
    packages = [
        "typer",
        "rich",
        "boto3",
        "amazon-transcribe",  # AWS Transcribe Streaming SDK
        "faster-whisper",     # Fallback local ASR
        "elevenlabs",
    ]

    console.print("  Installing Python packages...")
    for pkg in packages:
        console.print(f"    Installing {pkg}...")
        result = conn.run(
            f"pip install --break-system-packages {pkg}",
            hide=True,
            warn=True
        )
        if result.return_code != 0:
            console.print(f"    [yellow]Warning: {pkg} may have issues[/yellow]")

    console.print("[green]Dependencies installed[/green]")


@app.command()
def configure():
    """Configure AWS and ElevenLabs credentials on robot."""
    console.print("[yellow]Configuring credentials...[/yellow]")

    conn = get_connection()
    config = load_config()

    # Check if AWS is already configured
    result = conn.run("aws sts get-caller-identity 2>/dev/null", hide=True, warn=True)
    if result.return_code == 0:
        console.print("[green]AWS credentials already configured[/green]")
    else:
        console.print("\n[bold]AWS Configuration Required[/bold]")
        console.print("Run on the robot:")
        console.print("  [cyan]aws configure[/cyan]")
        console.print("\nUse the IAM user credentials with BedrockInvokeOnly policy.")
        console.print("Policy file: aws/landerpi-bedrock-policy.json")

    # Setup .env file for ElevenLabs
    env_file = f"/home/{config['user']}/.env"

    # Get current values if they exist
    result = conn.run(f"cat {env_file} 2>/dev/null || echo ''", hide=True)
    current_env = result.stdout

    if "ELEVENLABS_API_KEY" not in current_env:
        console.print("\n[bold]ElevenLabs Configuration[/bold]")

        # Try to get from local .env
        local_env = Path(__file__).parent / ".env"
        if local_env.exists():
            with open(local_env) as f:
                for line in f:
                    if line.startswith("ELEVENLABS_"):
                        key, value = line.strip().split("=", 1)
                        conn.run(f"echo '{key}={value}' >> {env_file}", hide=True)
                        console.print(f"  Added {key}")

            console.print("[green]ElevenLabs credentials copied from local .env[/green]")
        else:
            console.print("Set on robot:")
            console.print(f"  [cyan]echo 'ELEVENLABS_API_KEY=your_key' >> {env_file}[/cyan]")
            console.print(f"  [cyan]echo 'ELEVENLABS_VOICE_ID=your_voice_id' >> {env_file}[/cyan]")
    else:
        console.print("[green]ElevenLabs credentials already configured[/green]")

    # Add .env sourcing to .bashrc if not present
    result = conn.run("grep 'source.*\\.env' ~/.bashrc 2>/dev/null || echo ''", hide=True)
    if not result.stdout.strip():
        conn.run(f"echo 'set -a; source {env_file} 2>/dev/null; set +a' >> ~/.bashrc", hide=True)
        console.print("[green]Added .env sourcing to .bashrc[/green]")

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


@app.command("install-service")
def install_service():
    """Install systemd service on robot (does NOT auto-enable)."""
    console.print("[yellow]Installing systemd service...[/yellow]")

    conn = get_connection()
    config = load_config()

    # Upload service file
    service_file = Path(__file__).parent / "systemd" / "tars-voice.service"
    if not service_file.exists():
        console.print("[red]Service file not found: systemd/tars-voice.service[/red]")
        raise typer.Exit(1)

    conn.put(str(service_file), "/tmp/tars-voice.service")

    # Install service (but don't enable - user must explicitly enable for boot)
    conn.sudo("cp /tmp/tars-voice.service /etc/systemd/system/", hide=True)
    conn.sudo("systemctl daemon-reload", hide=True)

    console.print("[green]Service installed (not enabled for boot)[/green]")
    console.print("\nManagement commands:")
    console.print("  [cyan]sudo systemctl start tars-voice[/cyan]   # Start now")
    console.print("  [cyan]sudo systemctl stop tars-voice[/cyan]    # Stop")
    console.print("  [cyan]sudo systemctl enable tars-voice[/cyan]  # Enable on boot")
    console.print("  [cyan]sudo systemctl status tars-voice[/cyan]  # Check status")
    console.print("  [cyan]journalctl -u tars-voice -f[/cyan]       # View logs")


@app.command()
def test():
    """Run component check on robot."""
    console.print("[yellow]Running check on robot...[/yellow]")

    conn = get_connection()
    config = load_config()

    result = conn.run(
        f"cd /home/{config['user']} && source ~/.env 2>/dev/null; python3 robot_voicecontroller.py check",
        warn=True
    )


@app.command("run")
def run_voice(
    mode: str = typer.Argument("listen", help="Mode: listen, loop, check"),
):
    """Run voice controller on robot."""
    conn = get_connection()
    config = load_config()

    console.print(f"[yellow]Starting voice controller ({mode})...[/yellow]")

    conn.run(
        f"cd /home/{config['user']} && source ~/.env 2>/dev/null; python3 robot_voicecontroller.py {mode}",
        pty=True  # Interactive mode
    )


if __name__ == "__main__":
    app()
