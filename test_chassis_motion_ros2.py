#!/usr/bin/env python3
"""
Test script for LanderPi chassis motion control.
Moves the robot forward and backward no more than 20 cm.

SAFETY: This script REQUIRES user approval before moving the robot.
"""
import sys
import time
from typing import Optional

import typer
from rich.console import Console
from rich.prompt import Confirm
from rich.panel import Panel
from fabric import Connection
from invoke.exceptions import UnexpectedExit

app = typer.Typer(help="LanderPi Chassis Motion Test")
console = Console()

# Motion parameters
MAX_DISTANCE_M = 0.20  # 20 cm max
LINEAR_SPEED = 0.15    # m/s (safe speed, max is 0.3)
MOVEMENT_DURATION = MAX_DISTANCE_M / LINEAR_SPEED  # ~1.33 seconds

# ROS2 topic for velocity commands
CMD_VEL_TOPIC = "/ros_robot_controller/cmd_vel"


class ChassisMotionTest:
    def __init__(self, host: str, user: str, password: Optional[str] = None):
        self.host = host
        self.user = user
        self.console = console

        connect_kwargs = {}
        if password:
            connect_kwargs['password'] = password

        self.conn = Connection(host=host, user=user, connect_kwargs=connect_kwargs)

    def verify_connection(self) -> bool:
        """Check if we can connect to the robot."""
        try:
            self.console.print(f"[bold blue]Connecting to {self.user}@{self.host}...[/bold blue]")
            self.conn.run("echo 'Connection successful'", hide=True)
            self.console.print("[green]Connection successful[/green]")
            return True
        except Exception as e:
            self.console.print(f"[red]Connection failed:[/red] {e}")
            return False

    def check_ros2_available(self) -> bool:
        """Check if ROS2 is available on the robot."""
        try:
            result = self.conn.run("source /opt/ros/humble/setup.bash && ros2 topic list", hide=True, warn=True)
            if result.ok and CMD_VEL_TOPIC in result.stdout:
                self.console.print(f"[green]ROS2 available, {CMD_VEL_TOPIC} topic found[/green]")
                return True
            else:
                # Check if ros_robot_controller is running
                self.console.print("[yellow]cmd_vel topic not found. Checking robot controller...[/yellow]")
                return False
        except Exception as e:
            self.console.print(f"[red]ROS2 check failed:[/red] {e}")
            return False

    def stop_robot(self):
        """Send stop command to the robot."""
        stop_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"ros2 topic pub --once {CMD_VEL_TOPIC} geometry_msgs/msg/Twist "
            f"'{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}'"
        )
        self.conn.run(stop_cmd, hide=True, warn=True)

    def move_forward(self, distance_m: float = MAX_DISTANCE_M) -> bool:
        """Move robot forward by specified distance."""
        duration = distance_m / LINEAR_SPEED

        self.console.print(f"[bold yellow]Moving FORWARD {distance_m*100:.1f} cm at {LINEAR_SPEED} m/s ({duration:.2f}s)[/bold yellow]")

        # Publish velocity command
        move_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"timeout {duration + 0.5} ros2 topic pub --rate 10 {CMD_VEL_TOPIC} geometry_msgs/msg/Twist "
            f"'{{linear: {{x: {LINEAR_SPEED}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}'"
        )

        try:
            # Run command with timeout
            self.conn.run(move_cmd, hide=True, warn=True, timeout=duration + 2)
            time.sleep(duration)
        except Exception as e:
            self.console.print(f"[yellow]Move command ended: {e}[/yellow]")
        finally:
            self.stop_robot()

        self.console.print("[green]Forward movement complete[/green]")
        return True

    def move_backward(self, distance_m: float = MAX_DISTANCE_M) -> bool:
        """Move robot backward by specified distance."""
        duration = distance_m / LINEAR_SPEED

        self.console.print(f"[bold yellow]Moving BACKWARD {distance_m*100:.1f} cm at {LINEAR_SPEED} m/s ({duration:.2f}s)[/bold yellow]")

        # Publish velocity command (negative x for backward)
        move_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"timeout {duration + 0.5} ros2 topic pub --rate 10 {CMD_VEL_TOPIC} geometry_msgs/msg/Twist "
            f"'{{linear: {{x: {-LINEAR_SPEED}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}'"
        )

        try:
            self.conn.run(move_cmd, hide=True, warn=True, timeout=duration + 2)
            time.sleep(duration)
        except Exception as e:
            self.console.print(f"[yellow]Move command ended: {e}[/yellow]")
        finally:
            self.stop_robot()

        self.console.print("[green]Backward movement complete[/green]")
        return True


@app.command()
def test(
    host: str = typer.Option(..., help="Robot IP address"),
    user: str = typer.Option("pi", help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    distance: float = typer.Option(0.10, help="Distance to move in meters (max 0.20)"),
    direction: str = typer.Option("both", help="Direction: forward, backward, or both"),
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval prompt"),
):
    """
    Test chassis motion by moving forward and/or backward.

    SAFETY: Maximum movement is 20 cm. Robot will stop immediately after movement.
    """
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
        f"[bold]Chassis Motion Test[/bold]\n\n"
        f"Host: {host}\n"
        f"Distance: {distance*100:.1f} cm\n"
        f"Speed: {LINEAR_SPEED} m/s\n"
        f"Direction: {direction}\n"
        f"Duration: {distance/LINEAR_SPEED:.2f}s per direction",
        title="Test Parameters",
        border_style="blue"
    ))

    # Initialize tester
    tester = ChassisMotionTest(host, user, password)

    # Verify connection
    if not tester.verify_connection():
        sys.exit(1)

    # Check ROS2
    ros2_ok = tester.check_ros2_available()
    if not ros2_ok:
        console.print("[yellow]Warning: Could not verify ROS2 topic. The robot controller may need to be started.[/yellow]")
        console.print("[dim]Run on robot: ros2 launch ros_robot_controller ros_robot_controller.launch.py[/dim]")
        if not Confirm.ask("Continue anyway?", default=False):
            sys.exit(1)

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
    host: str = typer.Option(..., help="Robot IP address"),
    user: str = typer.Option("pi", help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Emergency stop - send zero velocity to robot."""
    console.print("[bold red]Sending STOP command...[/bold red]")

    tester = ChassisMotionTest(host, user, password)
    if tester.verify_connection():
        tester.stop_robot()
        console.print("[green]Stop command sent[/green]")


if __name__ == "__main__":
    app()
