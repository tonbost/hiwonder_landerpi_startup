#!/usr/bin/env python3
"""
Direct chassis motion test using ros_robot_controller_sdk.
No ROS2 required - communicates directly with STM32 via serial.

SAFETY: Maximum movement is 20 cm. Robot will stop immediately after movement.
"""
import sys
import time
import json
from pathlib import Path
from typing import Optional

import typer
from rich.console import Console
from rich.prompt import Confirm
from rich.panel import Panel
from fabric import Connection

app = typer.Typer(help="LanderPi Direct Chassis Motion Test (No ROS2)")
console = Console()

# Motion parameters
MAX_DISTANCE_M = 0.20  # 20 cm max
LINEAR_SPEED = 0.30    # m/s (max speed)
MOVEMENT_DURATION = MAX_DISTANCE_M / LINEAR_SPEED  # ~1.33 seconds

# SDK paths on remote - prefer permanent location, fall back to /tmp
REMOTE_SDK_DIR_PERMANENT = "/home/{user}/ros_robot_controller"
REMOTE_SDK_PATH_PERMANENT = REMOTE_SDK_DIR_PERMANENT + "/ros_robot_controller_sdk.py"
REMOTE_SDK_PATH_TMP = "/tmp/ros_robot_controller_sdk.py"
LOCAL_SDK_PATH = Path(__file__).parent / "drivers" / "ros_robot_controller-ros2" / "src" / "ros_robot_controller" / "ros_robot_controller" / "ros_robot_controller_sdk.py"


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


class DirectChassisTest:
    def __init__(self, host: str, user: str, password: Optional[str] = None):
        self.host = host
        self.user = user
        self.console = console
        self.sdk_path = None  # Will be set by upload_sdk()

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

    def check_serial_port(self) -> bool:
        """Check if serial port is available."""
        try:
            result = self.conn.run("ls /dev/ttyACM0", hide=True, warn=True)
            if result.ok:
                self.console.print("[green]/dev/ttyACM0 available[/green]")
                return True
            else:
                self.console.print("[red]/dev/ttyACM0 not found[/red]")
                return False
        except Exception as e:
            self.console.print(f"[red]Serial check failed:[/red] {e}")
            return False

    def upload_sdk(self) -> bool:
        """Upload SDK to robot if needed. Prefers permanent location."""
        try:
            # Check for permanent SDK location first (installed by setup_landerpi.py)
            permanent_path = REMOTE_SDK_PATH_PERMANENT.format(user=self.user)
            permanent_dir = REMOTE_SDK_DIR_PERMANENT.format(user=self.user)

            result = self.conn.run(f"test -f {permanent_path}", hide=True, warn=True)
            if result.ok:
                self.console.print(f"[dim]SDK found at {permanent_path}[/dim]")
                self.sdk_path = permanent_dir
                return True

            # Check if SDK exists in /tmp (fallback)
            result = self.conn.run(f"test -f {REMOTE_SDK_PATH_TMP}", hide=True, warn=True)
            if result.ok:
                self.console.print("[dim]SDK already on robot (/tmp)[/dim]")
                self.sdk_path = "/tmp"
                return True

            # Upload SDK to /tmp
            if LOCAL_SDK_PATH.exists():
                self.console.print("[blue]Uploading SDK to robot...[/blue]")
                self.conn.put(str(LOCAL_SDK_PATH), REMOTE_SDK_PATH_TMP)
                self.console.print("[green]SDK uploaded to /tmp[/green]")
                self.sdk_path = "/tmp"
                return True
            else:
                self.console.print(f"[red]Local SDK not found at {LOCAL_SDK_PATH}[/red]")
                return False
        except Exception as e:
            self.console.print(f"[red]SDK upload failed:[/red] {e}")
            return False

    def run_motion_test(self, distance_m: float, direction: str, duration_override: float = None) -> bool:
        """Run motion test on robot using SDK directly."""
        duration = duration_override if duration_override else distance_m / LINEAR_SPEED

        # Motor mapping: M1=front-left, M2=back-left, M3=front-right, M4=back-right
        # Left side: M1, M2 | Right side: M3, M4 (right side inverted)
        # For left side: negative = forward, positive = backward
        # For right side: positive = forward, negative = backward

        speed = LINEAR_SPEED

        if direction == "forward":
            # Forward: left side -, right side +
            m1_speed, m2_speed, m3_speed, m4_speed = -speed, -speed, speed, speed
            dir_label = "FORWARD"
        elif direction == "backward":
            # Backward: left side +, right side -
            m1_speed, m2_speed, m3_speed, m4_speed = speed, speed, -speed, -speed
            dir_label = "BACKWARD"
        elif direction == "turn_right":
            # Turn right (clockwise): left forward, right backward
            m1_speed, m2_speed, m3_speed, m4_speed = -speed, -speed, -speed, -speed
            dir_label = "TURN RIGHT"
        elif direction == "turn_left":
            # Turn left (counter-clockwise): left backward, right forward
            m1_speed, m2_speed, m3_speed, m4_speed = speed, speed, speed, speed
            dir_label = "TURN LEFT"
        elif direction == "strafe_right":
            # Strafe right: FL forward, BL backward, FR backward, BR forward
            m1_speed, m2_speed, m3_speed, m4_speed = -speed, speed, -speed, speed
            dir_label = "STRAFE RIGHT"
        elif direction == "strafe_left":
            # Strafe left: FL backward, BL forward, FR forward, BR backward
            m1_speed, m2_speed, m3_speed, m4_speed = speed, -speed, speed, -speed
            dir_label = "STRAFE LEFT"
        else:
            # Default to forward
            m1_speed, m2_speed, m3_speed, m4_speed = -speed, -speed, speed, speed
            dir_label = "FORWARD"

        self.console.print(f"[bold yellow]Moving {dir_label} {distance_m*100:.1f} cm at {LINEAR_SPEED} m/s ({duration:.2f}s)[/bold yellow]")

        # Use the SDK path determined during upload
        sdk_dir = self.sdk_path or "/tmp"

        # Write motion script to remote file
        motion_script = f"""import sys
sys.path.insert(0, '{sdk_dir}')
import time
from ros_robot_controller_sdk import Board

board = Board()
board.enable_reception()

# Beep to indicate start
board.set_buzzer(1000, 0.1, 0.1, 1)
time.sleep(0.3)

# Move - Mecanum wheel pattern: M1,M3 opposite to M2,M4
print('Starting motion...')
board.set_motor_speed([[1, {m1_speed}], [2, {m2_speed}], [3, {m3_speed}], [4, {m4_speed}]])
time.sleep({duration})

# Stop
board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
print('Motion complete')

# Beep to indicate stop
board.set_buzzer(500, 0.1, 0.1, 1)
"""

        try:
            # Write script to file on remote
            from io import StringIO
            self.conn.put(StringIO(motion_script), "/tmp/motion_test.py")

            # Execute the script
            result = self.conn.run(
                "python3 /tmp/motion_test.py",
                hide=False,
                warn=True,
                timeout=duration + 10
            )
            if result.ok:
                self.console.print(f"[green]{dir_label} movement complete[/green]")
                return True
            else:
                self.console.print(f"[red]Motion failed: {result.stderr}[/red]")
                return False
        except Exception as e:
            self.console.print(f"[red]Motion error: {e}[/red]")
            self.stop_robot()
            return False

    def stop_robot(self):
        """Emergency stop."""
        sdk_dir = self.sdk_path or "/tmp"
        stop_script = f'''
import sys
sys.path.insert(0, '{sdk_dir}')
from ros_robot_controller_sdk import Board

board = Board()
board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
print("STOPPED")
'''
        try:
            self.conn.run(f"python3 -c '{stop_script}'", hide=True, warn=True, timeout=3)
        except:
            pass

    def get_battery(self) -> Optional[int]:
        """Get battery voltage."""
        sdk_dir = self.sdk_path or "/tmp"
        script = f'''
import sys
sys.path.insert(0, '{sdk_dir}')
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
'''
        try:
            result = self.conn.run(f"python3 -c '{script}'", hide=True, warn=True, timeout=5)
            if result.ok and result.stdout.strip():
                return int(result.stdout.strip())
        except:
            pass
        return None


DIRECTIONS = ["forward", "backward", "turn_right", "turn_left", "strafe_right", "strafe_left"]


@app.command()
def test(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    distance: float = typer.Option(0.10, help="Distance to move in meters (max 0.20)"),
    direction: str = typer.Option("both", help="Direction: forward, backward, turn_right, turn_left, strafe_right, strafe_left, both, or all"),
    duration: float = typer.Option(None, help="Override duration in seconds (ignores distance calc)"),
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval prompt"),
):
    """
    Test chassis motion using direct serial communication (no ROS2 needed).
    """
    # Load config if options not provided
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required. Provide via options or config.json[/red]")
        sys.exit(1)

    # Validate distance
    if distance > MAX_DISTANCE_M:
        console.print(f"[yellow]Distance capped to {MAX_DISTANCE_M*100:.0f}cm max[/yellow]")
        distance = MAX_DISTANCE_M

    if distance <= 0:
        console.print("[red]Distance must be positive[/red]")
        sys.exit(1)

    # Calculate effective duration
    effective_duration = duration if duration else distance / LINEAR_SPEED

    # Determine directions for display
    if direction == "all":
        dir_display = "ALL (forward, backward, turn R/L, strafe R/L)"
    elif direction == "both":
        dir_display = "forward + backward"
    else:
        dir_display = direction

    # Show parameters
    console.print(Panel.fit(
        f"[bold]Direct Chassis Motion Test[/bold]\n\n"
        f"Host: {host}\n"
        f"Distance: {distance*100:.1f} cm\n"
        f"Speed: {LINEAR_SPEED} m/s\n"
        f"Direction: {dir_display}\n"
        f"Duration: {effective_duration:.2f}s per direction\n\n"
        f"[dim]Using direct serial (no ROS2)[/dim]",
        title="Test Parameters",
        border_style="blue"
    ))

    tester = DirectChassisTest(host, user, password)

    # Verify connection
    if not tester.verify_connection():
        sys.exit(1)

    # Check serial port
    if not tester.check_serial_port():
        console.print("[red]Serial port /dev/ttyACM0 not available[/red]")
        sys.exit(1)

    # Upload SDK
    if not tester.upload_sdk():
        console.print("[red]Could not upload SDK[/red]")
        sys.exit(1)

    # Get battery
    battery = tester.get_battery()
    if battery:
        voltage = battery / 1000.0
        console.print(f"[blue]Battery: {voltage:.2f}V[/blue]")
        if voltage < 10.0:
            console.print("[yellow]Warning: Low battery![/yellow]")

    # Request approval
    if not skip_approval:
        console.print("\n[bold red]WARNING: The robot is about to MOVE![/bold red]")
        console.print("[yellow]Ensure the area around the robot is clear.[/yellow]")

        if not Confirm.ask("\n[bold]Do you approve the robot to move?[/bold]", default=False):
            console.print("[blue]Motion test cancelled[/blue]")
            sys.exit(0)

    console.print("\n[bold green]Motion approved. Starting test...[/bold green]\n")

    # Determine which directions to test
    if direction == "all":
        directions_to_test = DIRECTIONS
    elif direction == "both":
        directions_to_test = ["forward", "backward"]
    else:
        directions_to_test = [direction]

    # Execute movements
    try:
        for i, dir_name in enumerate(directions_to_test):
            tester.run_motion_test(distance, dir_name, duration)
            if i < len(directions_to_test) - 1:
                time.sleep(1)

    except KeyboardInterrupt:
        console.print("\n[red]Interrupted! Stopping robot...[/red]")
        tester.stop_robot()
        sys.exit(1)

    console.print("\n[bold green]Chassis motion test complete![/bold green]")


@app.command()
def motor_test(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    motor: int = typer.Option(None, help="Test single motor (1-4), or all if not specified"),
    speed: float = typer.Option(0.3, help="Motor speed"),
    duration: float = typer.Option(2.0, help="Duration in seconds"),
):
    """Test individual motors to identify mapping."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    tester = DirectChassisTest(host, user, password)
    if not tester.verify_connection():
        sys.exit(1)
    tester.upload_sdk()

    if motor:
        motors_to_test = [motor]
    else:
        motors_to_test = [1, 2, 3, 4]

    sdk_dir = tester.sdk_path or "/tmp"

    for m in motors_to_test:
        console.print(f"\n[bold yellow]Testing Motor {m} at speed {speed} for {duration}s[/bold yellow]")
        console.print("[dim]Watch which wheel spins and which direction[/dim]")

        script = f"""import sys
sys.path.insert(0, '{sdk_dir}')
import time
from ros_robot_controller_sdk import Board

board = Board()
board.enable_reception()
board.set_buzzer(1000, 0.1, 0.1, 1)
time.sleep(0.3)

print('Motor {m} running...')
board.set_motor_speed([[{m}, {speed}]])
time.sleep({duration})
board.set_motor_speed([[{m}, 0]])
print('Motor {m} stopped')
board.set_buzzer(500, 0.1, 0.1, 1)
"""
        from io import StringIO
        tester.conn.put(StringIO(script), "/tmp/motor_test.py")
        tester.conn.run("python3 /tmp/motor_test.py", hide=False, warn=True, timeout=duration + 5)

        if len(motors_to_test) > 1:
            time.sleep(1)

    console.print("\n[green]Motor test complete![/green]")


@app.command()
def stop(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Emergency stop - send zero velocity to all motors."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print("[bold red]Sending STOP command...[/bold red]")
    tester = DirectChassisTest(host, user, password)
    if tester.verify_connection():
        tester.upload_sdk()
        tester.stop_robot()
        console.print("[green]Stop command sent[/green]")


@app.command()
def status(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Get robot status (battery, IMU)."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    tester = DirectChassisTest(host, user, password)
    if not tester.verify_connection():
        sys.exit(1)

    if not tester.check_serial_port():
        sys.exit(1)

    tester.upload_sdk()

    # Get battery
    battery = tester.get_battery()
    if battery:
        voltage = battery / 1000.0
        console.print(f"[green]Battery: {voltage:.2f}V ({battery}mV)[/green]")
    else:
        console.print("[yellow]Could not read battery[/yellow]")


if __name__ == "__main__":
    app()
