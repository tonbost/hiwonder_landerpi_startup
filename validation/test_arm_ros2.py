#!/usr/bin/env python3
"""
ROS2-based arm test for LanderPi robot.
Requires deployed ROS2 stack: `deploy_ros2_stack.py deploy`

Tests arm via /arm/cmd and /arm/state topics.
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
from rich.prompt import Confirm

app = typer.Typer(help="LanderPi Arm Test (ROS2 Stack)")
console = Console()

# ROS2 topics
ARM_CMD_TOPIC = "/arm/cmd"
ARM_STATE_TOPIC = "/arm/state"

# Home positions for each servo (user-defined resting position)
HOME_POSITIONS = {
    1: 550,   # Base
    2: 785,   # Shoulder
    3: 0,     # Elbow (reads negative, clamped to 0)
    4: 257,   # Wrist
    5: 501,   # Rotate
    10: 497,  # Gripper
}


def load_config() -> dict:
    """Load connection config from config.json if exists."""
    # First check project root (parent of validation/)
    config_path = Path(__file__).parent.parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
    # Fallback to same directory
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
    # Config not found - print helpful message
    console.print("[yellow]Warning: config.json not found[/yellow]")
    console.print(f"  Checked: {Path(__file__).parent.parent / 'config.json'}")
    console.print(f"  Checked: {Path(__file__).parent / 'config.json'}")
    console.print("[dim]Create config.json with: {\"host\": \"<IP>\", \"user\": \"<USER>\", \"password\": \"<PASS>\"}[/dim]")
    return {}


class ArmROS2Test:
    """Test arm via ROS2 stack."""

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

    def check_arm_topic(self) -> bool:
        """Check if arm topic exists."""
        try:
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash && "
                f"ros2 topic list | grep {ARM_CMD_TOPIC}'",
                hide=True, warn=True
            )
            if result.ok:
                self.console.print(f"[green]Arm topic found: {ARM_CMD_TOPIC}[/green]")
                return True
            self.console.print(f"[red]Arm topic not found: {ARM_CMD_TOPIC}[/red]")
            return False
        except Exception as e:
            self.console.print(f"[red]Topic check failed:[/red] {e}")
            return False

    def publish_arm_cmd(self, cmd: dict) -> bool:
        """Publish command to arm topic using Python script to avoid escaping issues."""
        cmd_json = json.dumps(cmd)
        try:
            # Create a simple Python publisher script
            script = f'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

rclpy.init()
node = Node("arm_cmd_pub")
pub = node.create_publisher(String, "{ARM_CMD_TOPIC}", 10)

# Wait for subscriber
import time
time.sleep(0.5)

msg = String()
msg.data = {repr(cmd_json)}
pub.publish(msg)

# Spin briefly to ensure message is sent
rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
print("OK")
'''
            # Write script to host and copy to container
            self.conn.run(f"cat > /tmp/arm_pub.py << 'SCRIPT_EOF'\n{script}\nSCRIPT_EOF", hide=True)
            self.conn.run("docker cp /tmp/arm_pub.py landerpi-ros2:/tmp/arm_pub.py", hide=True)

            result = self.conn.run(
                "docker exec landerpi-ros2 bash -c '"
                "source /opt/ros/humble/setup.bash && "
                "source /ros2_ws/install/setup.bash && "
                "python3 /tmp/arm_pub.py'",
                hide=True, warn=True, timeout=10
            )
            return result.ok and "OK" in result.stdout
        except Exception as e:
            self.console.print(f"[red]Publish failed:[/red] {e}")
            return False

    def read_arm_state(self) -> Optional[dict]:
        """Read arm state."""
        try:
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash && "
                f"timeout 3 ros2 topic echo --once {ARM_STATE_TOPIC}'",
                hide=True, warn=True, timeout=10
            )
            if result.ok and result.stdout:
                # Parse YAML-like output
                for line in result.stdout.split('\n'):
                    if 'data:' in line:
                        data = line.split('data:', 1)[1].strip().strip('"\'')
                        return json.loads(data)
            return None
        except Exception:
            return None

    def go_home(self, duration: float = 2.0) -> bool:
        """Move arm to home position."""
        self.console.print("[yellow]Moving to home position...[/yellow]")
        cmd = {"action": "home", "duration": duration}
        if self.publish_arm_cmd(cmd):
            time.sleep(duration + 0.5)
            self.console.print("[green]Home command sent[/green]")
            return True
        return False

    def stop_arm(self) -> bool:
        """Stop arm servos."""
        self.console.print("[red]Stopping arm...[/red]")
        cmd = {"action": "stop"}
        return self.publish_arm_cmd(cmd)

    def control_gripper(self, position: str) -> bool:
        """Control gripper."""
        self.console.print(f"[yellow]Gripper: {position}[/yellow]")
        cmd = {"action": "gripper", "position": position}
        if self.publish_arm_cmd(cmd):
            time.sleep(1)
            return True
        return False

    def set_position(self, positions: list, duration: float = 2.0) -> bool:
        """Set specific servo positions."""
        cmd = {"action": "set_position", "duration": duration, "positions": positions}
        if self.publish_arm_cmd(cmd):
            time.sleep(duration + 0.5)
            return True
        return False

    def test_arm_positions(self, duration: float = 2.0) -> bool:
        """Test arm through various positions."""
        self.console.print("\n[yellow]Testing arm positions...[/yellow]")

        # Position 1: Reach forward (from home)
        self.console.print("[cyan]Position 1: Reaching forward...[/cyan]")
        if not self.set_position([[1, 546], [2, 600], [3, 200], [4, 400], [5, 501]], duration):
            return False

        # Position 2: Reach up
        self.console.print("[cyan]Position 2: Reaching up...[/cyan]")
        if not self.set_position([[1, 546], [2, 400], [3, 300], [4, 500], [5, 501]], duration):
            return False

        # Position 3: Return to home
        self.console.print("[cyan]Position 3: Returning to home...[/cyan]")
        home_pos = [[sid, HOME_POSITIONS[sid]] for sid in [1, 2, 3, 4, 5]]
        if not self.set_position(home_pos, duration):
            return False

        return True


@app.command()
def test(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    duration: float = typer.Option(2.0, help="Movement duration in seconds"),
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval prompt"),
):
    """
    Run arm test via ROS2 stack.

    Requires: deploy_ros2_stack.py deploy
    """
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]LanderPi Arm Test (ROS2)[/bold]\n\n"
        f"Host: {host}\n"
        f"Duration: {duration}s\n\n"
        f"[dim]Using deployed ROS2 stack[/dim]",
        title="Test Parameters",
        border_style="blue"
    ))

    tester = ArmROS2Test(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    if not tester.check_ros2_stack():
        sys.exit(1)

    if not tester.check_arm_topic():
        console.print("[yellow]Arm controller may not be running in the stack[/yellow]")

    if not skip_approval:
        console.print("\n[bold red]WARNING: The arm is about to MOVE![/bold red]")
        if not Confirm.ask("Continue?", default=False):
            console.print("[blue]Cancelled[/blue]")
            sys.exit(0)

    try:
        # Step 1: Move to home
        console.print("\n[bold]Step 1: Moving to home position[/bold]")
        tester.go_home(duration)
        time.sleep(1)

        # Step 2: Test gripper
        console.print("\n[bold]Step 2: Testing gripper[/bold]")
        tester.control_gripper("open")
        time.sleep(1)
        tester.control_gripper("close")
        time.sleep(1)
        tester.control_gripper("open")  # Leave open
        time.sleep(0.5)

        # Step 3: Full extension (all servos to 500)
        console.print("\n[bold]Step 3: Full extension (500 everywhere)[/bold]")
        tester.set_position([[1, 500], [2, 500], [3, 500], [4, 500], [5, 500]], duration)
        time.sleep(1)

        # Step 4: Test arm positions
        console.print("\n[bold]Step 4: Testing arm positions[/bold]")
        tester.test_arm_positions(duration)

        # Step 5: Return to home
        console.print("\n[bold]Step 5: Returning to home[/bold]")
        tester.go_home(duration)

        console.print("\n[bold green]Full arm test complete![/bold green]")
    except KeyboardInterrupt:
        console.print("\n[red]Interrupted![/red]")
        tester.stop_arm()
        sys.exit(1)


@app.command()
def home(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    duration: float = typer.Option(2.0, help="Duration in seconds"),
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval"),
):
    """Move arm to home position via ROS2."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    tester = ArmROS2Test(host, user, password)
    if not tester.verify_connection() or not tester.check_ros2_stack():
        sys.exit(1)

    if not skip_approval:
        if not Confirm.ask("Move arm to home?", default=False):
            sys.exit(0)

    tester.go_home(duration)


@app.command()
def stop(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Emergency stop - disable arm servos."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print("[bold red]Sending STOP command...[/bold red]")
    tester = ArmROS2Test(host, user, password)
    if tester.verify_connection():
        tester.stop_arm()
        console.print("[green]Stop command sent[/green]")


@app.command()
def status(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Get arm status via ROS2."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    tester = ArmROS2Test(host, user, password)
    if not tester.verify_connection() or not tester.check_ros2_stack():
        sys.exit(1)

    console.print("[blue]Reading arm state...[/blue]")
    state = tester.read_arm_state()
    if state:
        console.print(f"[green]Arm state:[/green] {json.dumps(state, indent=2)}")
    else:
        console.print("[yellow]Could not read arm state[/yellow]")


if __name__ == "__main__":
    app()
