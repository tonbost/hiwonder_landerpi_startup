#!/usr/bin/env python3
"""
ROS2-based lidar test for LanderPi robot.
Requires deployed ROS2 stack: `deploy_ros2_stack.py deploy`

Tests lidar via /scan topic.
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

app = typer.Typer(help="LanderPi Lidar Test (ROS2 Stack)")
console = Console()

# ROS2 topics
SCAN_TOPIC = "/scan"


def load_config() -> dict:
    """Load connection config from config.json if exists."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
    return {}


class LidarROS2Test:
    """Test lidar via ROS2 stack."""

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

    def check_scan_topic(self) -> bool:
        """Check if scan topic exists."""
        try:
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash && "
                f"ros2 topic list | grep {SCAN_TOPIC}'",
                hide=True, warn=True
            )
            if result.ok:
                self.console.print(f"[green]Scan topic found: {SCAN_TOPIC}[/green]")
                return True
            self.console.print(f"[yellow]Scan topic not found: {SCAN_TOPIC}[/yellow]")
            return False
        except Exception as e:
            self.console.print(f"[red]Topic check failed:[/red] {e}")
            return False

    def get_topic_hz(self) -> Optional[float]:
        """Get scan topic publish rate."""
        try:
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash && "
                f"timeout 3 ros2 topic hz {SCAN_TOPIC} --window 5' 2>&1 | tail -3",
                hide=True, warn=True, timeout=10
            )
            if result.ok:
                for line in result.stdout.split('\n'):
                    if 'average rate:' in line:
                        rate = float(line.split(':')[1].strip().split()[0])
                        return rate
            return None
        except Exception:
            return None

    def read_scan(self, samples: int = 1) -> dict:
        """Read scan data and return statistics."""
        try:
            # Use Python to analyze scan data
            script = f'''
import sys
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time

class ScanReader(Node):
    def __init__(self):
        super().__init__('scan_reader')
        self.samples = []
        self.max_samples = {samples}
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '{SCAN_TOPIC}', self.cb, qos)

    def cb(self, msg):
        if len(self.samples) >= self.max_samples:
            return
        ranges = [r for r in msg.ranges if 0.05 < r < 12.0 and not math.isinf(r)]
        if ranges:
            self.samples.append({{
                'min': min(ranges),
                'max': max(ranges),
                'avg': sum(ranges)/len(ranges),
                'count': len(ranges),
                'total': len(msg.ranges)
            }})
            print(f"Sample {{len(self.samples)}}: min={{min(ranges):.2f}}m max={{max(ranges):.2f}}m points={{len(ranges)}}")

rclpy.init()
node = ScanReader()
start = time.time()
while len(node.samples) < node.max_samples and time.time() - start < 10:
    rclpy.spin_once(node, timeout_sec=0.5)
if node.samples:
    print("SUCCESS")
else:
    print("NO_DATA")
rclpy.shutdown()
'''
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash && "
                f"python3 -c \"{script}\"'",
                hide=False, warn=True, timeout=20
            )
            return {'success': 'SUCCESS' in result.stdout}
        except Exception as e:
            self.console.print(f"[red]Scan read error:[/red] {e}")
            return {'success': False}


@app.command()
def check(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Check lidar status via ROS2 stack."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]LanderPi Lidar Check (ROS2)[/bold]\n\n"
        f"Host: {host}\n"
        f"Topic: {SCAN_TOPIC}\n\n"
        f"[dim]Using deployed ROS2 stack[/dim]",
        title="Lidar Check",
        border_style="blue"
    ))

    tester = LidarROS2Test(host, user, password)

    checks = []

    if not tester.verify_connection():
        sys.exit(1)

    stack_ok = tester.check_ros2_stack()
    checks.append(("ROS2 Stack", stack_ok))

    if stack_ok:
        topic_ok = tester.check_scan_topic()
        checks.append(("Scan Topic", topic_ok))

        if topic_ok:
            hz = tester.get_topic_hz()
            if hz:
                checks.append(("Publish Rate", True))
                console.print(f"[green]Publish rate: {hz:.1f} Hz[/green]")
            else:
                checks.append(("Publish Rate", False))

    # Summary
    console.print("\n[bold]Summary:[/bold]")
    all_ok = True
    for name, ok in checks:
        status = "[green]PASS[/green]" if ok else "[red]FAIL[/red]"
        if not ok:
            all_ok = False
        console.print(f"  {name}: {status}")

    sys.exit(0 if all_ok else 1)


@app.command()
def scan(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    samples: int = typer.Option(5, help="Number of samples to read"),
):
    """Read lidar scan data via ROS2 stack."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]LanderPi Lidar Scan (ROS2)[/bold]\n\n"
        f"Host: {host}\n"
        f"Samples: {samples}",
        title="Scan Test",
        border_style="blue"
    ))

    tester = LidarROS2Test(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    if not tester.check_ros2_stack():
        sys.exit(1)

    console.print(f"\n[blue]Reading {samples} scan samples...[/blue]\n")
    result = tester.read_scan(samples)

    if result.get('success'):
        console.print("\n[bold green]Lidar scan test complete![/bold green]")
    else:
        console.print("\n[red]Failed to read scan data[/red]")
        sys.exit(1)


@app.command()
def status(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Get lidar driver status."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    tester = LidarROS2Test(host, user, password)
    if not tester.verify_connection() or not tester.check_ros2_stack():
        sys.exit(1)

    # Check topic and rate
    topic_ok = tester.check_scan_topic()
    if topic_ok:
        hz = tester.get_topic_hz()
        if hz:
            console.print(f"[green]Lidar publishing at {hz:.1f} Hz[/green]")
        else:
            console.print("[yellow]Lidar topic exists but rate unknown[/yellow]")
    else:
        console.print("[red]Lidar not publishing[/red]")


if __name__ == "__main__":
    app()
