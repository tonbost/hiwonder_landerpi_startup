#!/usr/bin/env python3
"""
Deploy autonomous exploration to LanderPi robot.

Uploads robot_explorer.py and manages exploration sessions.

Usage:
    uv run python deploy_exploration.py deploy          # Upload script
    uv run python deploy_exploration.py start           # Start exploration (5 min)
    uv run python deploy_exploration.py start --duration 30  # Start 30 min
    uv run python deploy_exploration.py stop            # Stop exploration
    uv run python deploy_exploration.py status          # Check status
    uv run python deploy_exploration.py logs            # View recent logs
"""

import json
import sys
from pathlib import Path
from typing import Optional

import typer
from fabric import Connection
from rich.console import Console
from rich.panel import Panel

app = typer.Typer(help="LanderPi Exploration Deployment")
console = Console()

# Paths
LOCAL_SCRIPT = Path(__file__).parent / "robot_explorer.py"
REMOTE_SCRIPT = "robot_explorer.py"  # Will be in home dir
REMOTE_LOGS = "landerpi/exploration_logs"


def load_config() -> dict:
    """Load connection config from config.json."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
    console.print("[red]config.json not found[/red]")
    return {}


def get_connection(host: str, user: str, password: Optional[str]) -> Connection:
    """Create SSH connection."""
    connect_kwargs = {}
    if password:
        connect_kwargs["password"] = password
    return Connection(host=host, user=user, connect_kwargs=connect_kwargs)


@app.command()
def deploy(
    host: Optional[str] = typer.Option(None, help="Robot IP"),
    user: Optional[str] = typer.Option(None, help="SSH user"),
    password: Optional[str] = typer.Option(None, help="SSH password"),
):
    """Upload exploration script to robot."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]Deploy Exploration[/bold]\n\n"
        f"Host: {host}\n"
        f"Script: {LOCAL_SCRIPT.name}",
        title="Deployment",
        border_style="blue"
    ))

    conn = get_connection(host, user, password)

    try:
        console.print(f"[blue]Connecting to {user}@{host}...[/blue]")
        conn.run("echo connected", hide=True)
        console.print("[green]Connected[/green]")

        # Upload script
        console.print(f"[blue]Uploading {LOCAL_SCRIPT.name}...[/blue]")
        remote_home = conn.run("echo $HOME", hide=True).stdout.strip()
        remote_path = f"{remote_home}/{REMOTE_SCRIPT}"
        conn.put(str(LOCAL_SCRIPT), remote_path)
        conn.run(f"chmod +x {remote_path}", hide=True)
        console.print("[green]Script uploaded[/green]")

        # Create log directory
        conn.run(f"mkdir -p {remote_home}/{REMOTE_LOGS}", hide=True)

        # Verify
        result = conn.run(f"python3 {REMOTE_SCRIPT} --help", hide=True, warn=True)
        if result.ok:
            console.print("[green]Deployment successful![/green]")
        else:
            console.print("[yellow]Script uploaded but verification failed[/yellow]")

    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")
        sys.exit(1)


@app.command()
def start(
    host: Optional[str] = typer.Option(None, help="Robot IP"),
    user: Optional[str] = typer.Option(None, help="SSH user"),
    password: Optional[str] = typer.Option(None, help="SSH password"),
    duration: float = typer.Option(5.0, help="Duration in minutes"),
    speed: float = typer.Option(0.2, help="Forward speed (m/s)"),
):
    """Start exploration on robot."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]Start Exploration[/bold]\n\n"
        f"Host: {host}\n"
        f"Duration: {duration} min\n"
        f"Speed: {speed} m/s\n\n"
        f"[bold red]Robot will MOVE![/bold red]",
        title="Exploration",
        border_style="yellow"
    ))

    conn = get_connection(host, user, password)

    try:
        console.print(f"[blue]Connecting to {user}@{host}...[/blue]")
        conn.run("echo connected", hide=True)

        # Check if script exists
        result = conn.run(f"test -f {REMOTE_SCRIPT}", hide=True, warn=True)
        if not result.ok:
            console.print("[yellow]Script not found, deploying...[/yellow]")
            conn.put(str(LOCAL_SCRIPT), REMOTE_SCRIPT)
            conn.run(f"chmod +x {REMOTE_SCRIPT}", hide=True)

        # Check ROS2 stack
        result = conn.run("docker ps --filter name=landerpi-ros2 -q", hide=True, warn=True)
        if not result.stdout.strip():
            console.print("[red]ROS2 stack not running![/red]")
            console.print("[dim]Run: uv run python deploy_ros2_stack.py deploy[/dim]")
            sys.exit(1)

        console.print("[green]Starting exploration...[/green]\n")

        # Run exploration (this will stream output)
        cmd = f"python3 {REMOTE_SCRIPT} explore --duration {duration} --speed {speed}"
        conn.run(cmd, pty=True)

    except KeyboardInterrupt:
        console.print("\n[yellow]Interrupted - sending stop...[/yellow]")
        try:
            conn.run(f"python3 {REMOTE_SCRIPT} stop", hide=True, warn=True)
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
        console.print("[red]Sending STOP...[/red]")
        conn.run(f"python3 {REMOTE_SCRIPT} stop", hide=False, warn=True)
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
        console.print(f"[blue]Connecting to {user}@{host}...[/blue]")
        conn.run("echo connected", hide=True)

        # Check script exists
        result = conn.run(f"test -f {REMOTE_SCRIPT}", hide=True, warn=True)
        script_ok = result.ok

        # Run status
        if script_ok:
            conn.run(f"python3 {REMOTE_SCRIPT} status", hide=False, warn=True)
        else:
            console.print("[yellow]Explorer script not deployed[/yellow]")

        # List recent sessions
        console.print("\n[bold]Recent sessions:[/bold]")
        result = conn.run(
            f"ls -1t {REMOTE_LOGS} 2>/dev/null | head -5",
            hide=True, warn=True
        )
        if result.ok and result.stdout.strip():
            for line in result.stdout.strip().split('\n'):
                console.print(f"  {line}")
        else:
            console.print("  [dim]No sessions yet[/dim]")

    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")


@app.command()
def logs(
    host: Optional[str] = typer.Option(None, help="Robot IP"),
    user: Optional[str] = typer.Option(None, help="SSH user"),
    password: Optional[str] = typer.Option(None, help="SSH password"),
    session: Optional[str] = typer.Option(None, help="Session name (default: latest)"),
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
        # Get session dir
        if session:
            session_dir = f"{REMOTE_LOGS}/{session}"
        else:
            result = conn.run(f"ls -1t {REMOTE_LOGS} | head -1", hide=True, warn=True)
            if not result.ok or not result.stdout.strip():
                console.print("[yellow]No sessions found[/yellow]")
                return
            session_dir = f"{REMOTE_LOGS}/{result.stdout.strip()}"

        console.print(f"[bold]Session:[/bold] {session_dir}\n")

        # Show metadata
        console.print("[bold]Metadata:[/bold]")
        conn.run(f"cat {session_dir}/metadata.json 2>/dev/null || echo 'No metadata'", hide=False)

        # Show events
        console.print("\n[bold]Events:[/bold]")
        conn.run(f"cat {session_dir}/events.jsonl 2>/dev/null | head -20 || echo 'No events'", hide=False)

        # Show lidar sample count
        result = conn.run(f"wc -l < {session_dir}/lidar_scans.jsonl 2>/dev/null", hide=True, warn=True)
        if result.ok and result.stdout.strip():
            console.print(f"\n[bold]Lidar samples:[/bold] {result.stdout.strip()}")

    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")


if __name__ == "__main__":
    app()
