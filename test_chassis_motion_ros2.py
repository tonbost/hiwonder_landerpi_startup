#!/usr/bin/env python3
"""
ROS2-based chassis motion test for LanderPi robot.
Requires deployed ROS2 stack: `deploy_ros2_stack.py deploy`

Moves the robot forward and backward no more than 20 cm.
SAFETY: This script REQUIRES user approval before moving the robot.
"""
import json
import sys
import time
from pathlib import Path
from typing import Optional

import typer
from rich.console import Console
from rich.prompt import Confirm
from rich.panel import Panel
from fabric import Connection

app = typer.Typer(help="LanderPi Chassis Motion Test (ROS2 Stack)")
console = Console()

# Motion parameters
MAX_DISTANCE_M = 0.20  # 20 cm max
LINEAR_SPEED = 0.15    # m/s (safe speed, max is 0.3)
MOVEMENT_DURATION = MAX_DISTANCE_M / LINEAR_SPEED  # ~1.33 seconds

# ROS2 topic for velocity commands (cmd_vel_bridge listens here)
CMD_VEL_TOPIC = "/cmd_vel"


def load_config() -> dict:
    """Load connection config from config.json if exists."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
    return {}


class ChassisMotionTest:
    """Test chassis motion via ROS2 stack."""

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

    def check_cmd_vel_topic(self) -> bool:
        """Check if cmd_vel topic exists."""
        try:
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash && "
                f"ros2 topic list | grep {CMD_VEL_TOPIC}'",
                hide=True, warn=True
            )
            if result.ok:
                self.console.print(f"[green]cmd_vel topic found: {CMD_VEL_TOPIC}[/green]")
                return True
            self.console.print(f"[yellow]cmd_vel topic not found: {CMD_VEL_TOPIC}[/yellow]")
            return False
        except Exception as e:
            self.console.print(f"[red]Topic check failed:[/red] {e}")
            return False

    def publish_twist(self, linear_x: float, duration: float) -> bool:
        """Publish Twist message using Python script to avoid escaping issues."""
        script = f'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

rclpy.init()
node = Node("motion_pub")
pub = node.create_publisher(Twist, "{CMD_VEL_TOPIC}", 10)

# Wait for subscriber
time.sleep(0.3)

msg = Twist()
msg.linear.x = {linear_x}
msg.linear.y = 0.0
msg.linear.z = 0.0
msg.angular.x = 0.0
msg.angular.y = 0.0
msg.angular.z = 0.0

# Publish at 10Hz for duration
start = time.time()
while time.time() - start < {duration}:
    pub.publish(msg)
    time.sleep(0.1)

# Send stop
msg.linear.x = 0.0
pub.publish(msg)
time.sleep(0.1)

node.destroy_node()
rclpy.shutdown()
print("OK")
'''
        try:
            self.conn.run(f"cat > /tmp/motion_pub.py << 'SCRIPT_EOF'\n{script}\nSCRIPT_EOF", hide=True)
            self.conn.run("docker cp /tmp/motion_pub.py landerpi-ros2:/tmp/motion_pub.py", hide=True)

            result = self.conn.run(
                "docker exec landerpi-ros2 bash -c '"
                "source /opt/ros/humble/setup.bash && "
                "source /ros2_ws/install/setup.bash && "
                "python3 /tmp/motion_pub.py'",
                hide=False, warn=True, timeout=duration + 5
            )
            return result.ok and "OK" in result.stdout
        except Exception as e:
            self.console.print(f"[red]Publish failed:[/red] {e}")
            return False

    def stop_robot(self):
        """Send stop command to the robot."""
        self.console.print("[red]Stopping robot...[/red]")
        self.publish_twist(0.0, 0.2)

    def move_forward(self, distance_m: float = MAX_DISTANCE_M) -> bool:
        """Move robot forward by specified distance."""
        duration = distance_m / LINEAR_SPEED

        self.console.print(f"[bold yellow]Moving FORWARD {distance_m*100:.1f} cm at {LINEAR_SPEED} m/s ({duration:.2f}s)[/bold yellow]")

        if self.publish_twist(LINEAR_SPEED, duration):
            self.console.print("[green]Forward movement complete[/green]")
            return True
        return False

    def move_backward(self, distance_m: float = MAX_DISTANCE_M) -> bool:
        """Move robot backward by specified distance."""
        duration = distance_m / LINEAR_SPEED

        self.console.print(f"[bold yellow]Moving BACKWARD {distance_m*100:.1f} cm at {LINEAR_SPEED} m/s ({duration:.2f}s)[/bold yellow]")

        if self.publish_twist(-LINEAR_SPEED, duration):
            self.console.print("[green]Backward movement complete[/green]")
            return True
        return False


@app.command()
def test(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    distance: float = typer.Option(0.10, help="Distance to move in meters (max 0.20)"),
    direction: str = typer.Option("both", help="Direction: forward, backward, or both"),
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval prompt"),
):
    """
    Test chassis motion via ROS2 stack.

    Requires: deploy_ros2_stack.py deploy
    SAFETY: Maximum movement is 20 cm. Robot will stop immediately after movement.
    """
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    # Validate distance
    if distance > MAX_DISTANCE_M:
        console.print(f"[red]Distance {distance*100:.0f}cm exceeds maximum {MAX_DISTANCE_M*100:.0f}cm[/red]")
        console.print(f"[yellow]Using maximum distance: {MAX_DISTANCE_M*100:.0f}cm[/yellow]")
        distance = MAX_DISTANCE_M

    if distance <= 0:
        console.print("[red]Distance must be positive[/red]")
        sys.exit(1)

    # Show test parameters
    console.print(Panel.fit(
        f"[bold]Chassis Motion Test (ROS2)[/bold]\n\n"
        f"Host: {host}\n"
        f"Distance: {distance*100:.1f} cm\n"
        f"Speed: {LINEAR_SPEED} m/s\n"
        f"Direction: {direction}\n"
        f"Duration: {distance/LINEAR_SPEED:.2f}s per direction\n\n"
        f"[dim]Using deployed ROS2 stack[/dim]",
        title="Test Parameters",
        border_style="blue"
    ))

    # Initialize tester
    tester = ChassisMotionTest(host, user, password)

    # Verify connection
    if not tester.verify_connection():
        sys.exit(1)

    # Check ROS2 stack
    if not tester.check_ros2_stack():
        sys.exit(1)

    # Check topic (optional, warn only)
    tester.check_cmd_vel_topic()

    # Request approval
    if not skip_approval:
        console.print("\n[bold red]WARNING: The robot is about to MOVE![/bold red]")
        console.print("[yellow]Ensure the area around the robot is clear.[/yellow]")
        console.print("[yellow]The robot will move up to 20cm in each direction.[/yellow]")

        if not Confirm.ask("\n[bold]Do you approve the robot to move?[/bold]", default=False):
            console.print("[blue]Motion test cancelled by user[/blue]")
            sys.exit(0)

    console.print("\n[bold green]Motion approved. Starting test...[/bold green]")
    console.print("[dim]Robot will stop after each movement[/dim]\n")

    # Execute movements
    try:
        if direction in ("forward", "both"):
            tester.move_forward(distance)
            time.sleep(1)  # Pause between movements

        if direction in ("backward", "both"):
            tester.move_backward(distance)

    except KeyboardInterrupt:
        console.print("\n[red]Test interrupted! Stopping robot...[/red]")
        tester.stop_robot()
        sys.exit(1)
    except Exception as e:
        console.print(f"\n[red]Error during test: {e}[/red]")
        tester.stop_robot()
        sys.exit(1)

    console.print("\n[bold green]Chassis motion test complete![/bold green]")


@app.command()
def stop(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Emergency stop - send zero velocity to robot."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print("[bold red]Sending STOP command...[/bold red]")

    tester = ChassisMotionTest(host, user, password)
    if tester.verify_connection():
        tester.stop_robot()
        console.print("[green]Stop command sent[/green]")


if __name__ == "__main__":
    app()
