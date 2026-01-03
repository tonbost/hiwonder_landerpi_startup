#!/usr/bin/env python3
"""
Deploy and control robot exploration from local machine.

This script uploads robot_explorer.py and the exploration module to the robot
and provides commands to start, stop, and monitor exploration remotely.

The exploration module (validation/exploration/) is deployed to ~/landerpi/exploration/
on the robot, enabling modular code reuse.

Usage:
    uv run python deploy_explorer.py deploy          # Upload script + module to robot
    uv run python deploy_explorer.py start --duration 10  # Start exploration
    uv run python deploy_explorer.py stop            # Stop exploration
    uv run python deploy_explorer.py logs            # View logs
    uv run python deploy_explorer.py status          # Check status
"""

import json
import sys
import time
from pathlib import Path
from typing import Optional

import typer
from fabric import Connection
from rich.console import Console
from rich.panel import Panel

app = typer.Typer(help="Deploy and control robot exploration")
console = Console()

# Paths
BASE_DIR = Path(__file__).parent.resolve()
LOCAL_SCRIPT = BASE_DIR / "robot_explorer.py"
LOCAL_MODULE = BASE_DIR / "validation" / "exploration"
REMOTE_DIR = "landerpi"  # Relative to home
REMOTE_SCRIPT = f"{REMOTE_DIR}/robot_explorer.py"
REMOTE_MODULE = f"{REMOTE_DIR}/exploration"


def load_config() -> dict:
    """Load connection config from config.json."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
    console.print("[yellow]Warning: config.json not found[/yellow]")
    return {}


def get_connection(host: str, user: str, password: Optional[str]) -> Connection:
    """Create SSH connection."""
    connect_kwargs = {}
    if password:
        connect_kwargs['password'] = password
    return Connection(host=host, user=user, connect_kwargs=connect_kwargs)


def upload_directory(conn: Connection, local_path: Path, remote_path: str, user: str):
    """Upload a directory recursively, skipping __pycache__."""
    home = f"/home/{user}"
    full_remote = f"{home}/{remote_path}"

    # Create remote directory
    conn.run(f"mkdir -p {full_remote}", hide=True)

    # Upload each file
    for item in local_path.rglob("*"):
        if item.is_file() and "__pycache__" not in str(item):
            rel_path = item.relative_to(local_path)
            remote_file = f"{full_remote}/{rel_path}"
            remote_dir = str(Path(remote_file).parent)
            conn.run(f"mkdir -p {remote_dir}", hide=True)
            conn.put(str(item), remote_file)
            console.print(f"  [dim]{rel_path}[/dim]")


@app.command()
def deploy(
    host: Optional[str] = typer.Option(None, help="Robot IP"),
    user: Optional[str] = typer.Option(None, help="SSH user"),
    password: Optional[str] = typer.Option(None, help="SSH password"),
):
    """Upload robot_explorer.py and exploration module to the robot."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]Deploy Robot Explorer[/bold]\n\n"
        f"Host: {host}\n"
        f"Script: {LOCAL_SCRIPT.name}\n"
        f"Module: {LOCAL_MODULE.name}/",
        title="Deploy",
        border_style="blue"
    ))

    if not LOCAL_SCRIPT.exists():
        console.print(f"[red]Error: {LOCAL_SCRIPT} not found[/red]")
        sys.exit(1)

    if not LOCAL_MODULE.exists():
        console.print(f"[red]Error: {LOCAL_MODULE} not found[/red]")
        sys.exit(1)

    conn = get_connection(host, user, password)

    try:
        console.print(f"[blue]Connecting to {host}...[/blue]")
        conn.run("echo 'Connected'", hide=True)
        console.print("[green]Connected[/green]")

        # Create directories
        console.print(f"\n[blue]Creating ~/{REMOTE_DIR}/...[/blue]")
        conn.run(f"mkdir -p ~/{REMOTE_DIR}", hide=True)

        # Upload exploration module
        console.print(f"\n[blue]Uploading exploration module...[/blue]")
        # Remove old module first to ensure clean state
        conn.run(f"rm -rf ~/{REMOTE_MODULE}", hide=True, warn=True)
        upload_directory(conn, LOCAL_MODULE, REMOTE_MODULE, user)

        # Upload script
        remote_path = f"/home/{user}/{REMOTE_SCRIPT}"
        console.print(f"\n[blue]Uploading {LOCAL_SCRIPT.name}...[/blue]")
        conn.put(str(LOCAL_SCRIPT), remote_path)

        # Make executable
        conn.run(f"chmod +x {remote_path}", hide=True)

        console.print("\n[bold green]Deploy complete![/bold green]")
        console.print(f"\n[dim]To run on robot:[/dim]")
        console.print(f"  python3 ~/{REMOTE_SCRIPT} explore --duration 5")

    except Exception as e:
        console.print(f"[red]Deploy failed: {e}[/red]")
        sys.exit(1)


@app.command()
def start(
    host: Optional[str] = typer.Option(None, help="Robot IP"),
    user: Optional[str] = typer.Option(None, help="SSH user"),
    password: Optional[str] = typer.Option(None, help="SSH password"),
    duration: float = typer.Option(5.0, help="Duration in minutes"),
    speed: float = typer.Option(0.35, help="Forward speed (m/s)"),
    turn_multiplier: float = typer.Option(2.5, "--turn-multiplier", help="Turn time multiplier for carpet friction (default: 2.5)"),
    rosbag: bool = typer.Option(False, "--rosbag", help="Enable ROS2 bag recording"),
    yolo: bool = typer.Option(False, "--yolo", help="Enable YOLO object detection (CPU, 2-5 FPS)"),
    yolo_hailo: bool = typer.Option(False, "--yolo-hailo", help="Enable YOLO with Hailo-8 acceleration (25-40 FPS)"),
    yolo_logging: bool = typer.Option(False, "--yolo-logging", help="Enable YOLO debug logging (saves images to ~/yolo_logs)"),
):
    """Start exploration on robot."""
    # --yolo-hailo implies --yolo
    if yolo_hailo:
        yolo = True
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    rosbag_str = "[green]Yes[/green]" if rosbag else "No"
    yolo_str = "[green]Hailo (25-40 FPS)[/green]" if yolo_hailo else ("[green]CPU (2-5 FPS)[/green]" if yolo else "No")
    yolo_logging_str = "[green]Yes[/green]" if yolo_logging else "No"
    console.print(Panel.fit(
        f"[bold]Start Exploration[/bold]\n\n"
        f"Host: {host}\n"
        f"Duration: {duration} min\n"
        f"Speed: {speed} m/s\n"
        f"Turn multiplier: {turn_multiplier}x (carpet friction)\n"
        f"ROS2 bag: {rosbag_str}\n"
        f"YOLO: {yolo_str}\n"
        f"YOLO logging: {yolo_logging_str}\n\n"
        f"[bold red]Robot will MOVE![/bold red]",
        title="Start",
        border_style="yellow"
    ))

    conn = get_connection(host, user, password)

    try:
        console.print(f"[blue]Connecting to {host}...[/blue]")
        conn.run("echo 'Connected'", hide=True)

        # Check if script and module exist
        remote_path = f"/home/{user}/{REMOTE_SCRIPT}"
        remote_module_path = f"/home/{user}/{REMOTE_MODULE}"
        script_exists = conn.run(f"test -f {remote_path}", warn=True, hide=True).ok
        module_exists = conn.run(f"test -d {remote_module_path}", warn=True, hide=True).ok

        if not script_exists or not module_exists:
            console.print("[yellow]Explorer not fully deployed, deploying now...[/yellow]")
            conn.run(f"mkdir -p ~/{REMOTE_DIR}", hide=True)

            # Upload module
            if not module_exists:
                console.print("[dim]Uploading exploration module...[/dim]")
                conn.run(f"rm -rf ~/{REMOTE_MODULE}", hide=True, warn=True)
                upload_directory(conn, LOCAL_MODULE, REMOTE_MODULE, user)

            # Upload script
            conn.put(str(LOCAL_SCRIPT), remote_path)
            conn.run(f"chmod +x {remote_path}", hide=True)
            console.print("[green]Deployment complete[/green]")

        # Start Hailo inference server if requested
        if yolo_hailo:
            console.print("[blue]Starting Hailo inference server...[/blue]")
            # Check if already running
            result = conn.run("systemctl is-active hailo-server", hide=True, warn=True)
            if result.stdout.strip() != "active":
                conn.sudo("systemctl start hailo-server", pty=True, hide=True, warn=True)
                time.sleep(2)
                # Verify it started
                result = conn.run("systemctl is-active hailo-server", hide=True, warn=True)
                if result.stdout.strip() == "active":
                    console.print("[green]Hailo server started[/green]")
                else:
                    console.print("[red]Failed to start Hailo server![/red]")
                    console.print("Run: uv run python deploy_hailo8.py deploy")
                    sys.exit(1)
            else:
                console.print("[dim]Hailo server already running[/dim]")

        # Restart ROS2 stack if YOLO logging or Hailo mode requested
        if yolo_logging or yolo_hailo:
            features = []
            if yolo_hailo:
                features.append("Hailo bridge")
            if yolo_logging:
                features.append("YOLO logging")
            console.print(f"[blue]Restarting ROS2 stack with {' + '.join(features)}...[/blue]")

            # Stop current stack completely
            conn.run("cd ~/landerpi/docker && docker compose down", hide=True, warn=True)
            time.sleep(2)

            # Build environment variables for restart
            env_vars = []
            if yolo_logging:
                conn.run("echo 'true' > ~/landerpi/.yolo_logging_enabled", hide=True)
                env_vars.append("YOLO_LOGGING=true")
            if yolo_hailo:
                env_vars.append("USE_HAILO=true")

            env_prefix = " ".join(env_vars)
            restart_cmd = f'''cd ~/landerpi/docker && {env_prefix} docker compose up -d'''
            result = conn.run(restart_cmd, warn=True, hide=True)

            if result.ok:
                console.print(f"[green]ROS2 stack restarted[/green]")
                if yolo_hailo:
                    console.print("[dim]Using Hailo-8 via ZeroMQ bridge (25-40 FPS)[/dim]")
                if yolo_logging:
                    console.print("[dim]YOLO images will be saved to ~/yolo_logs/[/dim]")
                # Wait for stack to fully initialize (build + launch)
                console.print("[dim]Waiting for ROS2 stack to initialize...[/dim]")
                time.sleep(15)
            else:
                console.print("[yellow]Warning: Failed to restart stack[/yellow]")

        console.print("[green]Starting exploration...[/green]")
        console.print("[dim]Press Ctrl+C to stop[/dim]\n")

        # Run exploration - stream output
        cmd = f"python3 ~/{REMOTE_SCRIPT} explore --duration {duration} --speed {speed} --turn-multiplier {turn_multiplier}"
        if rosbag:
            cmd += " --rosbag"
        if yolo:
            cmd += " --yolo"
        conn.run(cmd, pty=True)

    except KeyboardInterrupt:
        console.print("\n[yellow]Stopping...[/yellow]")
        try:
            conn.run(f"python3 ~/{REMOTE_SCRIPT} stop", hide=True, warn=True)
        except:
            pass
    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")
        sys.exit(1)


@app.command()
def stop(
    host: Optional[str] = typer.Option(None, help="Robot IP"),
    user: Optional[str] = typer.Option(None, help="SSH user"),
    password: Optional[str] = typer.Option(None, help="SSH password"),
):
    """Stop exploration and motors."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    conn = get_connection(host, user, password)

    try:
        console.print("[bold red]Sending STOP command...[/bold red]")
        conn.run(f"python3 ~/{REMOTE_SCRIPT} stop", hide=False, warn=True)
        console.print("[green]Stop sent[/green]")
    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")


@app.command()
def status(
    host: Optional[str] = typer.Option(None, help="Robot IP"),
    user: Optional[str] = typer.Option(None, help="SSH user"),
    password: Optional[str] = typer.Option(None, help="SSH password"),
):
    """Check robot status."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    conn = get_connection(host, user, password)

    try:
        console.print(f"[blue]Checking status on {host}...[/blue]\n")
        conn.run(f"python3 ~/{REMOTE_SCRIPT} status", hide=False, warn=True)
    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")


@app.command()
def logs(
    host: Optional[str] = typer.Option(None, help="Robot IP"),
    user: Optional[str] = typer.Option(None, help="SSH user"),
    password: Optional[str] = typer.Option(None, help="SSH password"),
    follow: bool = typer.Option(False, "-f", "--follow", help="Follow logs"),
    lines: int = typer.Option(50, "-n", "--lines", help="Number of lines"),
):
    """View exploration logs."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    conn = get_connection(host, user, password)

    try:
        # Find latest log
        result = conn.run(
            f"ls -td ~/{REMOTE_DIR}/exploration_logs/session_* 2>/dev/null | head -1",
            hide=True, warn=True
        )

        if not result.ok or not result.stdout.strip():
            console.print("[yellow]No exploration logs found[/yellow]")
            return

        latest = result.stdout.strip()
        console.print(f"[blue]Latest session: {latest}[/blue]\n")

        # Show metadata
        conn.run(f"cat {latest}/metadata.json 2>/dev/null || echo 'No metadata'", hide=False)

        # Show events
        console.print("\n[bold]Events:[/bold]")
        if follow:
            conn.run(f"tail -f {latest}/events.jsonl 2>/dev/null", pty=True)
        else:
            conn.run(f"tail -n {lines} {latest}/events.jsonl 2>/dev/null || echo 'No events'", hide=False)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")


if __name__ == "__main__":
    app()
