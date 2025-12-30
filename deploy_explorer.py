#!/usr/bin/env python3
"""
Deploy and control robot exploration from local machine.

This script uploads robot_explorer.py to the robot and provides
commands to start, stop, and monitor exploration remotely.

Usage:
    uv run python deploy_explorer.py deploy          # Upload script to robot
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
LOCAL_SCRIPT = Path(__file__).parent / "robot_explorer.py"
REMOTE_DIR = "~/landerpi"
REMOTE_SCRIPT = f"{REMOTE_DIR}/robot_explorer.py"


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


@app.command()
def deploy(
    host: Optional[str] = typer.Option(None, help="Robot IP"),
    user: Optional[str] = typer.Option(None, help="SSH user"),
    password: Optional[str] = typer.Option(None, help="SSH password"),
):
    """Upload robot_explorer.py to the robot."""
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
        f"Script: {LOCAL_SCRIPT.name}",
        title="Deploy",
        border_style="blue"
    ))

    if not LOCAL_SCRIPT.exists():
        console.print(f"[red]Error: {LOCAL_SCRIPT} not found[/red]")
        sys.exit(1)

    conn = get_connection(host, user, password)

    try:
        console.print(f"[blue]Connecting to {host}...[/blue]")
        conn.run("echo 'Connected'", hide=True)
        console.print("[green]Connected[/green]")

        # Create directory
        console.print(f"[blue]Creating {REMOTE_DIR}...[/blue]")
        conn.run(f"mkdir -p {REMOTE_DIR}", hide=True)

        # Upload script
        console.print(f"[blue]Uploading {LOCAL_SCRIPT.name}...[/blue]")
        conn.put(str(LOCAL_SCRIPT), REMOTE_SCRIPT)

        # Make executable
        conn.run(f"chmod +x {REMOTE_SCRIPT}", hide=True)

        console.print("[bold green]Deploy complete![/bold green]")
        console.print(f"\n[dim]To run on robot:[/dim]")
        console.print(f"  python3 {REMOTE_SCRIPT} explore --duration 5")

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
        title="Start",
        border_style="yellow"
    ))

    conn = get_connection(host, user, password)

    try:
        console.print(f"[blue]Connecting to {host}...[/blue]")
        conn.run("echo 'Connected'", hide=True)

        # Check if script exists
        result = conn.run(f"test -f {REMOTE_SCRIPT}", warn=True, hide=True)
        if not result.ok:
            console.print("[yellow]Script not found on robot, deploying...[/yellow]")
            conn.run(f"mkdir -p {REMOTE_DIR}", hide=True)
            conn.put(str(LOCAL_SCRIPT), REMOTE_SCRIPT)
            conn.run(f"chmod +x {REMOTE_SCRIPT}", hide=True)

        console.print("[green]Starting exploration...[/green]")
        console.print("[dim]Press Ctrl+C to stop[/dim]\n")

        # Run exploration - stream output
        cmd = f"python3 {REMOTE_SCRIPT} explore --duration {duration} --speed {speed}"
        conn.run(cmd, pty=True)

    except KeyboardInterrupt:
        console.print("\n[yellow]Stopping...[/yellow]")
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
        console.print("[bold red]Sending STOP command...[/bold red]")
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
        console.print(f"[blue]Checking status on {host}...[/blue]\n")
        conn.run(f"python3 {REMOTE_SCRIPT} status", hide=False, warn=True)
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
            f"ls -td {REMOTE_DIR}/exploration_logs/session_* 2>/dev/null | head -1",
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
