#!/usr/bin/env python3
"""
ROS2-based depth camera test for LanderPi robot.
Requires deployed ROS2 stack: `deploy_ros2_stack.py deploy`

Tests camera via /aurora/* topics.
"""
import json
import sys
from pathlib import Path
from typing import Optional

import typer
from fabric import Connection
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

app = typer.Typer(help="LanderPi Depth Camera Test (ROS2 Stack)")
console = Console()

# ROS2 topics
CAMERA_NAMESPACE = "aurora"
TOPICS = {
    "rgb": f"/{CAMERA_NAMESPACE}/rgb/image_raw",
    "depth": f"/{CAMERA_NAMESPACE}/depth/image_raw",
    "ir": f"/{CAMERA_NAMESPACE}/ir/image_raw",
    "points": f"/{CAMERA_NAMESPACE}/points2",
}


def load_config() -> dict:
    """Load connection config from config.json if exists."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
    return {}


class CameraROS2Test:
    """Test camera via ROS2 stack."""

    def __init__(self, host: str, user: str, password: Optional[str] = None):
        self.host = host
        self.user = user
        self.console = console

        connect_kwargs = {}
        if password:
            connect_kwargs['password'] = password

        self.conn = Connection(host=host, user=user, connect_kwargs=connect_kwargs)

    def verify_connection(self) -> bool:
        """Check SSH connection."""
        try:
            self.console.print(f"[bold blue]Connecting to {self.user}@{self.host}...[/bold blue]")
            self.conn.run("echo 'Connection successful'", hide=True)
            self.console.print("[green]Connection successful[/green]")
            return True
        except Exception as e:
            self.console.print(f"[red]Connection failed:[/red] {e}")
            return False

    def check_ros2_stack(self) -> bool:
        """Check if ROS2 stack is running."""
        try:
            result = self.conn.run(
                "docker ps --filter name=landerpi-ros2 -q",
                hide=True, warn=True
            )
            if result.ok and result.stdout.strip():
                self.console.print("[green]ROS2 stack is running[/green]")
                return True
            self.console.print("[red]ROS2 stack not running[/red]")
            self.console.print("[dim]Run: uv run python deploy_ros2_stack.py deploy[/dim]")
            return False
        except Exception as e:
            self.console.print(f"[red]Stack check failed:[/red] {e}")
            return False

    def check_camera_topics(self) -> dict:
        """Check which camera topics exist."""
        results = {}
        try:
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash 2>/dev/null; "
                f"source /deptrum_ws/install/local_setup.bash 2>/dev/null; "
                f"ros2 topic list | grep {CAMERA_NAMESPACE}'",
                hide=True, warn=True
            )
            if result.ok:
                found = result.stdout.strip().split('\n')
                for name, topic in TOPICS.items():
                    results[name] = topic in found
            else:
                for name in TOPICS:
                    results[name] = False
        except Exception:
            for name in TOPICS:
                results[name] = False
        return results

    def get_topic_info(self, topic: str) -> Optional[dict]:
        """Get topic type and publisher count."""
        try:
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash 2>/dev/null; "
                f"source /deptrum_ws/install/local_setup.bash 2>/dev/null; "
                f"ros2 topic info {topic}'",
                hide=True, warn=True
            )
            if result.ok:
                info = {}
                for line in result.stdout.split('\n'):
                    if 'Type:' in line:
                        info['type'] = line.split(':')[1].strip()
                    if 'Publisher count:' in line:
                        info['publishers'] = int(line.split(':')[1].strip())
                return info
            return None
        except Exception:
            return None

    def read_stream_sample(self, stream: str) -> bool:
        """Read one sample from a stream."""
        topic = TOPICS.get(stream)
        if not topic:
            return False

        try:
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash 2>/dev/null; "
                f"source /deptrum_ws/install/local_setup.bash 2>/dev/null; "
                f"timeout 5 ros2 topic echo --once {topic} --field header'",
                hide=True, warn=True, timeout=10
            )
            return result.ok and 'stamp' in result.stdout
        except Exception:
            return False


@app.command()
def check(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Check camera status via ROS2 stack."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]LanderPi Camera Check (ROS2)[/bold]\n\n"
        f"Host: {host}\n"
        f"Namespace: {CAMERA_NAMESPACE}\n\n"
        f"[dim]Using deployed ROS2 stack[/dim]",
        title="Camera Check",
        border_style="blue"
    ))

    tester = CameraROS2Test(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    if not tester.check_ros2_stack():
        sys.exit(1)

    # Check camera topics
    console.print("\n[blue]Checking camera topics...[/blue]")
    topics = tester.check_camera_topics()

    table = Table(title="Camera Topics")
    table.add_column("Stream", style="cyan")
    table.add_column("Topic", style="dim")
    table.add_column("Status", justify="center")

    all_ok = True
    for name, topic in TOPICS.items():
        found = topics.get(name, False)
        status = "[green]FOUND[/green]" if found else "[red]MISSING[/red]"
        if not found:
            all_ok = False
        table.add_row(name.upper(), topic, status)

    console.print(table)

    if all_ok:
        console.print("\n[bold green]All camera topics available![/bold green]")
    else:
        console.print("\n[yellow]Some camera topics missing[/yellow]")
        console.print("[dim]Camera driver may need to be enabled in launch file[/dim]")

    sys.exit(0 if all_ok else 1)


@app.command()
def stream(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Read camera stream samples via ROS2 stack."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]LanderPi Camera Stream Test (ROS2)[/bold]\n\n"
        f"Host: {host}",
        title="Stream Test",
        border_style="blue"
    ))

    tester = CameraROS2Test(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    if not tester.check_ros2_stack():
        sys.exit(1)

    # Test each stream
    console.print("\n[blue]Reading stream samples...[/blue]\n")

    results = {}
    for name in ['rgb', 'depth', 'ir']:
        console.print(f"  Testing {name.upper()}...", end=" ")
        success = tester.read_stream_sample(name)
        results[name] = success
        if success:
            console.print("[green]OK[/green]")
        else:
            console.print("[red]FAIL[/red]")

    if all(results.values()):
        console.print("\n[bold green]Camera stream test complete![/bold green]")
    else:
        console.print("\n[yellow]Some streams not available[/yellow]")
        sys.exit(1)


@app.command()
def status(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Get detailed camera status."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    tester = CameraROS2Test(host, user, password)
    if not tester.verify_connection() or not tester.check_ros2_stack():
        sys.exit(1)

    console.print("\n[blue]Getting topic info...[/blue]\n")

    table = Table(title="Camera Topic Details")
    table.add_column("Topic", style="cyan")
    table.add_column("Type", style="dim")
    table.add_column("Publishers", justify="center")

    for name, topic in TOPICS.items():
        info = tester.get_topic_info(topic)
        if info:
            table.add_row(
                topic,
                info.get('type', 'unknown'),
                str(info.get('publishers', 0))
            )
        else:
            table.add_row(topic, "[red]not found[/red]", "-")

    console.print(table)


if __name__ == "__main__":
    app()
