#!/usr/bin/env python3
"""
Autonomous exploration test for HiWonder LanderPi robot.

Uses sensor fusion (depth camera + lidar) with frontier-based exploration.
Robot will autonomously explore while avoiding obstacles.

Usage:
    uv run python validation/test_exploration.py start --yes
    uv run python validation/test_exploration.py start --duration 60 --yes
    uv run python validation/test_exploration.py stop
    uv run python validation/test_exploration.py status
"""

import json
import math
import signal
import sys
import tempfile
import time
from io import StringIO
from pathlib import Path
from typing import Optional

import typer
from fabric import Connection
from rich.console import Console
from rich.panel import Panel
from rich.prompt import Confirm
from rich.table import Table

from exploration import (
    ExplorationController,
    ExplorationConfig,
    SafetyConfig,
    SensorConfig,
    FrontierConfig,
    LogConfig,
)

app = typer.Typer(help="LanderPi Autonomous Exploration (Sensor Fusion)")
console = Console()

# Docker configuration
DOCKER_IMAGE = "landerpi-ros2:latest"
DOCKER_RUN_BASE = "docker run --rm --privileged --network host -v /dev:/dev"


def load_config() -> dict:
    """Load connection config from config.json."""
    config_path = Path(__file__).parent.parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
    # Also check parent directory
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
    return {}


class ExplorationRunner:
    """Runs autonomous exploration on LanderPi."""

    def __init__(self, host: str, user: str, password: Optional[str] = None):
        self.host = host
        self.user = user
        self.console = console

        connect_kwargs = {}
        if password:
            connect_kwargs["password"] = password

        self.conn = Connection(host=host, user=user, connect_kwargs=connect_kwargs)
        self.controller: Optional[ExplorationController] = None
        self.sdk_path = f"/home/{user}/ros_robot_controller"

    def verify_connection(self) -> bool:
        """Check SSH connection."""
        try:
            console.print(f"[blue]Connecting to {self.user}@{self.host}...[/blue]")
            self.conn.run("echo 'ok'", hide=True)
            console.print("[green]Connected[/green]")
            return True
        except Exception as e:
            console.print(f"[red]Connection failed: {e}[/red]")
            return False

    def check_prerequisites(self) -> bool:
        """Check all prerequisites are met."""
        checks = []

        # Check Docker
        result = self.conn.run(f"docker images -q {DOCKER_IMAGE}", hide=True, warn=True)
        checks.append(("Docker image", bool(result.ok and result.stdout.strip())))

        # Check SDK
        result = self.conn.run(f"test -f {self.sdk_path}/ros_robot_controller_sdk.py", hide=True, warn=True)
        checks.append(("Motor SDK", result.ok))

        # Check lidar device
        result = self.conn.run("ls /dev/ttyUSB* 2>/dev/null", hide=True, warn=True)
        checks.append(("Lidar device", result.ok))

        # Check camera
        result = self.conn.run("lsusb | grep 3251", hide=True, warn=True)
        checks.append(("Depth camera", result.ok))

        # Display results
        all_ok = True
        for name, ok in checks:
            status = "[green]OK[/green]" if ok else "[red]MISSING[/red]"
            console.print(f"  {name}: {status}")
            if not ok:
                all_ok = False

        return all_ok

    def get_battery(self) -> Optional[float]:
        """Get battery voltage in mV."""
        script = f"""
import sys
sys.path.insert(0, '{self.sdk_path}')
import time
from ros_robot_controller_sdk import Board

board = Board()
board.enable_reception()
time.sleep(0.1)
for _ in range(10):
    v = board.get_battery()
    if v is not None:
        print(v)
        break
    time.sleep(0.05)
"""
        try:
            result = self.conn.run(f"python3 -c '{script}'", hide=True, warn=True, timeout=5)
            if result.ok and result.stdout.strip():
                return float(result.stdout.strip())
        except:
            pass
        return None

    def move(self, vx: float, vy: float, wz: float) -> bool:
        """Send motion command via ROS2."""
        twist = f"'{{linear: {{x: {vx}, y: {vy}, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {wz}}}}}'"
        cmd = (
            f"docker exec landerpi-ros2 bash -c "
            f"\"source /opt/ros/humble/setup.bash && "
            f"ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist {twist}\""
        )
        try:
            self.conn.run(cmd, hide=True, warn=True, timeout=5)
            return True
        except:
            return False

    def stop_motors(self) -> None:
        """Stop all motors."""
        self.move(0, 0, 0)

    def read_lidar_scan(self) -> tuple:
        """Read lidar scan data. Returns (ranges, angle_min, angle_increment)."""
        # Script to read one scan
        scan_script = """
import sys
import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class ScanReader(Node):
    def __init__(self):
        super().__init__('scan_reader')
        self.data = None
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, qos)

    def cb(self, msg):
        if self.data is None:
            self.data = {
                'ranges': list(msg.ranges),
                'angle_min': msg.angle_min,
                'angle_increment': msg.angle_increment,
            }

rclpy.init()
node = ScanReader()
import time
start = time.time()
while node.data is None and (time.time() - start) < 2.0:
    rclpy.spin_once(node, timeout_sec=0.1)
if node.data:
    print(json.dumps(node.data))
node.destroy_node()
rclpy.shutdown()
"""
        try:
            # Upload and run script
            self.conn.put(StringIO(scan_script), "/tmp/read_scan.py")
            cmd = (
                f"{DOCKER_RUN_BASE} -v /tmp:/tmp {DOCKER_IMAGE} "
                f"bash -c 'source /opt/ros/humble/setup.bash && python3 /tmp/read_scan.py'"
            )
            result = self.conn.run(cmd, hide=True, warn=True, timeout=10)
            if result.ok and result.stdout.strip():
                data = json.loads(result.stdout.strip())
                return data['ranges'], data['angle_min'], data['angle_increment']
        except:
            pass
        return [], 0.0, 0.0

    def run_exploration(
        self,
        duration_minutes: float = 30.0,
        min_battery: float = 6.6,
        enable_rosbag: bool = False,
    ) -> None:
        """Run the exploration loop."""

        # Configure components
        safety_config = SafetyConfig(
            max_runtime_minutes=duration_minutes,
            battery_cutoff_voltage=min_battery,
            battery_warning_voltage=min_battery + 0.4,
        )
        log_config = LogConfig(rosbag_enabled=enable_rosbag)

        # Create controller
        self.controller = ExplorationController(
            move_func=self.move,
            stop_func=self.stop_motors,
            get_battery_func=self.get_battery,
            safety_config=safety_config,
            log_config=log_config,
        )

        # Start exploration
        self.controller.start()

        # Main loop
        loop_period = 1.0 / 10.0  # 10 Hz
        status_period = 60.0  # Report status every minute
        last_status_time = time.time()

        try:
            while self.controller.running:
                loop_start = time.time()

                # Read sensors
                ranges, angle_min, angle_inc = self.read_lidar_scan()
                if ranges:
                    self.controller.update_sensors(
                        ranges, angle_min, angle_inc,
                        depth_data=None, depth_width=0, depth_height=0
                    )

                # Step controller
                if not self.controller.step():
                    break

                # Periodic status
                if time.time() - last_status_time >= status_period:
                    last_status_time = time.time()
                    status = self.controller.get_status()
                    remaining = status['safety']['remaining_minutes']
                    console.print(f"[dim]Status: {remaining:.1f} min remaining, action: {status['action']}[/dim]")

                # Rate limiting
                elapsed = time.time() - loop_start
                if elapsed < loop_period:
                    time.sleep(loop_period - elapsed)

        except KeyboardInterrupt:
            console.print("\n[yellow]Interrupted[/yellow]")
        finally:
            self.controller.stop_exploration("Loop ended")


@app.command()
def start(
    host: Optional[str] = typer.Option(None, help="Robot IP"),
    user: Optional[str] = typer.Option(None, help="SSH user"),
    password: Optional[str] = typer.Option(None, help="SSH password"),
    duration: float = typer.Option(30.0, help="Max runtime in minutes"),
    min_battery: float = typer.Option(6.6, help="Battery cutoff voltage"),
    rosbag: bool = typer.Option(False, "--rosbag", help="Enable ROS2 bag recording"),
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval"),
):
    """Start autonomous exploration."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]Autonomous Exploration[/bold]\n\n"
        f"Host: {host}\n"
        f"Duration: {duration} min\n"
        f"Battery cutoff: {min_battery}V\n"
        f"ROS2 bag: {'Yes' if rosbag else 'No'}\n\n"
        f"[bold red]WARNING: Robot will MOVE autonomously![/bold red]",
        title="Exploration",
        border_style="yellow"
    ))

    runner = ExplorationRunner(host, user, password)

    if not runner.verify_connection():
        sys.exit(1)

    console.print("\n[bold]Checking prerequisites...[/bold]")
    if not runner.check_prerequisites():
        console.print("[red]Prerequisites not met[/red]")
        sys.exit(1)

    # Check battery
    battery = runner.get_battery()
    if battery:
        voltage = battery / 1000.0 if battery > 100 else battery
        console.print(f"[blue]Battery: {voltage:.2f}V[/blue]")
        if voltage < min_battery:
            console.print("[red]Battery too low![/red]")
            sys.exit(1)

    # Approval
    if not skip_approval:
        console.print("\n[bold red]The robot will move autonomously![/bold red]")
        if not Confirm.ask("Approve?", default=False):
            console.print("[blue]Cancelled[/blue]")
            sys.exit(0)

    console.print("\n[bold green]Starting exploration...[/bold green]\n")

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        console.print("\n[yellow]Stopping...[/yellow]")
        runner.stop_motors()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    runner.run_exploration(duration, min_battery, rosbag)


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

    console.print("[bold red]Sending STOP...[/bold red]")
    runner = ExplorationRunner(host, user, password)
    if runner.verify_connection():
        runner.stop_motors()
        console.print("[green]Stop sent[/green]")


@app.command()
def status(
    host: Optional[str] = typer.Option(None, help="Robot IP"),
    user: Optional[str] = typer.Option(None, help="SSH user"),
    password: Optional[str] = typer.Option(None, help="SSH password"),
):
    """Get robot status."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    runner = ExplorationRunner(host, user, password)
    if not runner.verify_connection():
        sys.exit(1)

    console.print("\n[bold]Robot Status[/bold]")

    # Battery
    battery = runner.get_battery()
    if battery:
        voltage = battery / 1000.0 if battery > 100 else battery
        console.print(f"Battery: {voltage:.2f}V")

    # Prerequisites
    console.print("\n[bold]Prerequisites:[/bold]")
    runner.check_prerequisites()


if __name__ == "__main__":
    app()
