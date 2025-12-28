#!/usr/bin/env python3
"""
LanderPi robotic arm test using ros_robot_controller_sdk.
No ROS2 required - communicates directly with STM32 via serial.

Arm servos: IDs 1-5 (5-DOF arm) + ID 10 (gripper)
Position range: typically 0-1000, with 500 as center/home

SAFETY: Arm will move. Ensure workspace is clear before running tests.
"""
import sys
import time
import json
from pathlib import Path
from typing import Optional
from io import StringIO

import typer
from rich.console import Console
from rich.prompt import Confirm
from rich.panel import Panel
from rich.table import Table
from fabric import Connection

app = typer.Typer(help="LanderPi Robotic Arm Test (No ROS2)")
console = Console()

# Arm configuration
ARM_SERVO_IDS = [1, 2, 3, 4, 5]  # 5-DOF arm
GRIPPER_SERVO_ID = 10
ALL_SERVO_IDS = ARM_SERVO_IDS + [GRIPPER_SERVO_ID]

# Standard positions
HOME_POSITION = 500  # Center position
GRIPPER_OPEN = 200
GRIPPER_CLOSED = 700

# Movement duration (seconds)
DEFAULT_DURATION = 2.0

# SDK paths on remote - prefer permanent location, fall back to /tmp
REMOTE_SDK_DIR_PERMANENT = "/home/{user}/ros_robot_controller"
REMOTE_SDK_PATH_PERMANENT = REMOTE_SDK_DIR_PERMANENT + "/ros_robot_controller_sdk.py"
REMOTE_SDK_PATH_TMP = "/tmp/ros_robot_controller_sdk.py"
LOCAL_SDK_PATH = Path(__file__).parent / "drivers" / "ros_robot_controller-ros2" / "src" / "ros_robot_controller" / "ros_robot_controller" / "ros_robot_controller_sdk.py"


def load_config() -> dict:
    """Load connection config from config.json if exists."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
    return {}


class ArmTest:
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

            # Upload SDK to current directory
            if LOCAL_SDK_PATH.exists():
                self.console.print("[blue]Uploading SDK to robot...[/blue]")
                self.conn.put(str(LOCAL_SDK_PATH), REMOTE_SDK_PATH_TMP)
                self.console.print("[green]SDK uploaded[/green]")
                self.sdk_path = "/tmp"
                return True
            else:
                self.console.print(f"[red]Local SDK not found at {LOCAL_SDK_PATH}[/red]")
                return False
        except Exception as e:
            self.console.print(f"[red]SDK upload failed:[/red] {e}")
            return False

    def execute_script(self, script: str, timeout: int = 10, hide_output: bool = False) -> bool:
        """Execute a Python script on the robot."""
        try:
            self.conn.put(StringIO(script), "/tmp/arm_script.py")
            result = self.conn.run(
                "python3 /tmp/arm_script.py",
                hide=hide_output,
                warn=True,
                timeout=timeout
            )
            return result.ok
        except Exception as e:
            self.console.print(f"[red]Script execution error: {e}[/red]")
            return False

    def get_servo_status(self, servo_ids: list = None) -> dict:
        """Read status of all or specified servos."""
        if servo_ids is None:
            servo_ids = ALL_SERVO_IDS

        sdk_dir = self.sdk_path or "/tmp"

        script = f"""import sys
sys.path.insert(0, '{sdk_dir}')
import time
from ros_robot_controller_sdk import Board

board = Board()
board.enable_reception()
time.sleep(0.2)

servo_ids = {servo_ids}
for sid in servo_ids:
    try:
        pos = board.bus_servo_read_position(sid)
        vin = board.bus_servo_read_vin(sid)
        temp = board.bus_servo_read_temp(sid)

        if pos and vin and temp:
            print(f"{{sid}},{{pos[0]}},{{vin[0]/1000:.2f}},{{temp[0]}}")
        else:
            print(f"{{sid}},ERROR,0,0")
    except Exception as e:
        print(f"{{sid}},ERROR,0,0")
    time.sleep(0.05)
"""

        try:
            self.conn.put(StringIO(script), "/tmp/arm_status.py")
            result = self.conn.run("python3 /tmp/arm_status.py", hide=True, warn=True, timeout=10)

            status = {}
            if result.ok:
                for line in result.stdout.strip().split('\n'):
                    parts = line.split(',')
                    if len(parts) == 4:
                        sid = int(parts[0])
                        status[sid] = {
                            'position': parts[1] if parts[1] != 'ERROR' else None,
                            'voltage': parts[2],
                            'temperature': parts[3]
                        }
            return status
        except Exception as e:
            self.console.print(f"[red]Status read failed: {e}[/red]")
            return {}

    def move_to_home(self) -> bool:
        """Move arm to home position (all servos at 500)."""
        self.console.print("[yellow]Moving arm to HOME position...[/yellow]")

        sdk_dir = self.sdk_path or "/tmp"

        script = f"""import sys
sys.path.insert(0, '{sdk_dir}')
import time
from ros_robot_controller_sdk import Board

board = Board()
board.enable_reception()

# Beep to indicate start
board.set_buzzer(1000, 0.1, 0.1, 1)
time.sleep(0.3)

# Move all servos to center (500) position
positions = [[1, {HOME_POSITION}], [2, {HOME_POSITION}], [3, {HOME_POSITION}],
             [4, {HOME_POSITION}], [5, {HOME_POSITION}], [10, {HOME_POSITION}]]
board.bus_servo_set_position({DEFAULT_DURATION}, positions)
print("Moving to home position...")

time.sleep({DEFAULT_DURATION + 0.5})

# Beep to indicate completion
board.set_buzzer(500, 0.1, 0.1, 1)
print("Home position reached")
"""

        return self.execute_script(script, timeout=int(DEFAULT_DURATION + 5))

    def stop_arm(self) -> bool:
        """Stop all arm servos (disable torque)."""
        self.console.print("[red]Stopping arm...[/red]")

        sdk_dir = self.sdk_path or "/tmp"

        script = f"""import sys
sys.path.insert(0, '{sdk_dir}')
from ros_robot_controller_sdk import Board

board = Board()
board.enable_reception()

# Stop all servos
board.bus_servo_stop([1, 2, 3, 4, 5, 10])
print("All servos stopped")
"""

        return self.execute_script(script, timeout=5, hide_output=True)

    def test_gripper(self) -> bool:
        """Test gripper open/close."""
        self.console.print("[yellow]Testing gripper...[/yellow]")

        sdk_dir = self.sdk_path or "/tmp"

        script = f"""import sys
sys.path.insert(0, '{sdk_dir}')
import time
from ros_robot_controller_sdk import Board

board = Board()
board.enable_reception()

# Open gripper
board.set_buzzer(800, 0.1, 0.1, 1)
time.sleep(0.3)
print("Opening gripper...")
board.bus_servo_set_position(1.5, [[10, {GRIPPER_OPEN}]])
time.sleep(2)

# Close gripper
board.set_buzzer(800, 0.1, 0.1, 1)
time.sleep(0.3)
print("Closing gripper...")
board.bus_servo_set_position(1.5, [[10, {GRIPPER_CLOSED}]])
time.sleep(2)

# Return to center
print("Returning to center...")
board.bus_servo_set_position(1.5, [[10, {HOME_POSITION}]])
time.sleep(2)

board.set_buzzer(500, 0.1, 0.1, 1)
print("Gripper test complete")
"""

        return self.execute_script(script, timeout=15)

    def test_individual_servo(self, servo_id: int, duration: float = DEFAULT_DURATION) -> bool:
        """Test a single servo movement."""
        self.console.print(f"[yellow]Testing servo {servo_id}...[/yellow]")

        sdk_dir = self.sdk_path or "/tmp"

        # Move servo through range: home -> +200 -> -200 -> home
        script = f"""import sys
sys.path.insert(0, '{sdk_dir}')
import time
from ros_robot_controller_sdk import Board

board = Board()
board.enable_reception()

servo_id = {servo_id}
duration = {duration}
HOME_POSITION = {HOME_POSITION}

# Read current position
current_pos = board.bus_servo_read_position(servo_id)
if current_pos:
    print(f"Current position: {{current_pos[0]}}")
else:
    print("Could not read current position, using 500")
    current_pos = [500]

# Beep
board.set_buzzer(1000, 0.1, 0.1, 1)
time.sleep(0.3)

# Move to +200 from home
print(f"Moving servo {{servo_id}} to {{HOME_POSITION + 200}}...")
board.bus_servo_set_position(duration, [[servo_id, HOME_POSITION + 200]])
time.sleep(duration + 0.5)

# Move to -200 from home
print(f"Moving servo {{servo_id}} to {{HOME_POSITION - 200}}...")
board.bus_servo_set_position(duration, [[servo_id, HOME_POSITION - 200]])
time.sleep(duration + 0.5)

# Return to home
print(f"Returning servo {{servo_id}} to home...")
board.bus_servo_set_position(duration, [[servo_id, HOME_POSITION]])
time.sleep(duration + 0.5)

board.set_buzzer(500, 0.1, 0.1, 1)
print(f"Servo {{servo_id}} test complete")
"""

        return self.execute_script(script, timeout=int(duration * 3 + 10))

    def run_full_test(self, duration: float = DEFAULT_DURATION) -> bool:
        """Run comprehensive arm test sequence."""
        self.console.print("\n[bold yellow]Starting full arm test sequence...[/bold yellow]\n")

        # Test 1: Move to home
        if not self.move_to_home():
            self.console.print("[red]Home position test failed[/red]")
            return False
        time.sleep(1)

        # Test 2: Test gripper
        if not self.test_gripper():
            self.console.print("[red]Gripper test failed[/red]")
            return False
        time.sleep(1)

        # Test 3: Move arm through basic positions
        self.console.print("\n[yellow]Testing arm positions...[/yellow]")

        sdk_dir = self.sdk_path or "/tmp"

        script = f"""import sys
sys.path.insert(0, '{sdk_dir}')
import time
from ros_robot_controller_sdk import Board

board = Board()
board.enable_reception()

duration = {duration}

# Position 1: Reach forward
board.set_buzzer(1000, 0.1, 0.1, 1)
time.sleep(0.3)
print("Position 1: Reaching forward...")
board.bus_servo_set_position(duration, [
    [1, 500], [2, 700], [3, 300], [4, 500], [5, 500]
])
time.sleep(duration + 0.5)

# Position 2: Reach up
board.set_buzzer(1000, 0.1, 0.1, 1)
time.sleep(0.3)
print("Position 2: Reaching up...")
board.bus_servo_set_position(duration, [
    [1, 500], [2, 500], [3, 600], [4, 400], [5, 500]
])
time.sleep(duration + 0.5)

# Position 3: Return to home
board.set_buzzer(1000, 0.1, 0.1, 1)
time.sleep(0.3)
print("Position 3: Returning to home...")
board.bus_servo_set_position(duration, [
    [1, 500], [2, 500], [3, 500], [4, 500], [5, 500]
])
time.sleep(duration + 0.5)

board.set_buzzer(500, 0.1, 0.1, 2)
print("Full arm test complete!")
"""

        return self.execute_script(script, timeout=int(duration * 3 + 10))


@app.command()
def test(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    duration: float = typer.Option(DEFAULT_DURATION, help="Movement duration in seconds"),
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval prompt"),
):
    """
    Run comprehensive arm movement test (positions + gripper).

    WARNING: Arm will move through multiple positions. Ensure workspace is clear.
    """
    # Load config if options not provided
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required. Provide via options or config.json[/red]")
        sys.exit(1)

    # Show parameters
    console.print(Panel.fit(
        f"[bold]LanderPi Arm Test[/bold]\n\n"
        f"Host: {host}\n"
        f"Servos: 1-5 (arm) + 10 (gripper)\n"
        f"Duration: {duration}s per movement\n\n"
        f"[dim]Using direct serial (no ROS2)[/dim]",
        title="Test Parameters",
        border_style="blue"
    ))

    tester = ArmTest(host, user, password)

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

    # Request approval
    if not skip_approval:
        console.print("\n[bold red]WARNING: The robotic arm is about to MOVE![/bold red]")
        console.print("[yellow]Ensure the workspace around the arm is clear.[/yellow]")
        console.print("[yellow]The arm will move through multiple positions.[/yellow]")

        if not Confirm.ask("\n[bold]Do you approve the arm to move?[/bold]", default=False):
            console.print("[blue]Arm test cancelled[/blue]")
            sys.exit(0)

    console.print("\n[bold green]Movement approved. Starting test...[/bold green]\n")

    # Execute test
    try:
        if tester.run_full_test(duration):
            console.print("\n[bold green]Arm test complete![/bold green]")
        else:
            console.print("\n[bold red]Arm test failed[/bold red]")
            sys.exit(1)
    except KeyboardInterrupt:
        console.print("\n[red]Interrupted! Stopping arm...[/red]")
        tester.stop_arm()
        sys.exit(1)


@app.command()
def status(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Get arm servo status (positions, voltages, temperatures)."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    tester = ArmTest(host, user, password)
    if not tester.verify_connection():
        sys.exit(1)

    if not tester.check_serial_port():
        sys.exit(1)

    tester.upload_sdk()

    console.print("\n[bold blue]Reading servo status...[/bold blue]\n")

    status_data = tester.get_servo_status()

    if not status_data:
        console.print("[red]Failed to read servo status[/red]")
        sys.exit(1)

    # Create status table
    table = Table(title="Arm Servo Status")
    table.add_column("Servo ID", style="cyan", justify="center")
    table.add_column("Type", style="magenta")
    table.add_column("Position", style="green", justify="right")
    table.add_column("Voltage", style="yellow", justify="right")
    table.add_column("Temperature", style="red", justify="right")

    for sid in ALL_SERVO_IDS:
        if sid in status_data:
            data = status_data[sid]
            servo_type = "Gripper" if sid == 10 else f"Joint {sid}"

            pos = data['position'] if data['position'] else "ERROR"
            voltage = f"{data['voltage']}V" if data['voltage'] != '0' else "ERROR"
            temp = f"{data['temperature']}C" if data['temperature'] != '0' else "ERROR"

            table.add_row(str(sid), servo_type, pos, voltage, temp)
        else:
            table.add_row(str(sid), f"Joint {sid}" if sid != 10 else "Gripper", "N/A", "N/A", "N/A")

    console.print(table)
    console.print()


@app.command()
def home(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval prompt"),
):
    """Move arm to home position (all servos at 500)."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    tester = ArmTest(host, user, password)
    if not tester.verify_connection():
        sys.exit(1)

    tester.upload_sdk()

    # Request approval
    if not skip_approval:
        console.print("\n[bold yellow]WARNING: Arm will move to home position[/bold yellow]")
        if not Confirm.ask("Continue?", default=False):
            console.print("[blue]Cancelled[/blue]")
            sys.exit(0)

    try:
        if tester.move_to_home():
            console.print("[green]Arm moved to home position[/green]")
        else:
            console.print("[red]Failed to move to home[/red]")
            sys.exit(1)
    except KeyboardInterrupt:
        console.print("\n[red]Interrupted![/red]")
        sys.exit(1)


@app.command()
def stop(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Emergency stop - disable torque on all arm servos."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print("[bold red]Sending STOP command...[/bold red]")
    tester = ArmTest(host, user, password)
    if tester.verify_connection():
        tester.upload_sdk()
        if tester.stop_arm():
            console.print("[green]Stop command sent - all servos disabled[/green]")
        else:
            console.print("[yellow]Stop command may not have completed[/yellow]")


@app.command()
def servo_test(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    servo: int = typer.Option(None, help="Test single servo (1-5, 10), or all if not specified"),
    duration: float = typer.Option(DEFAULT_DURATION, help="Duration in seconds"),
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval prompt"),
):
    """Test individual servos to identify their function."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    tester = ArmTest(host, user, password)
    if not tester.verify_connection():
        sys.exit(1)

    tester.upload_sdk()

    if servo:
        servos_to_test = [servo]
    else:
        servos_to_test = ALL_SERVO_IDS

    # Request approval
    if not skip_approval:
        console.print("\n[bold yellow]WARNING: Servo(s) will move[/bold yellow]")
        console.print(f"Testing servos: {servos_to_test}")
        if not Confirm.ask("Continue?", default=False):
            console.print("[blue]Cancelled[/blue]")
            sys.exit(0)

    try:
        for sid in servos_to_test:
            if sid == 10:
                # Test gripper
                tester.test_gripper()
            else:
                # Test arm servo
                tester.test_individual_servo(sid, duration)

            if len(servos_to_test) > 1 and sid != servos_to_test[-1]:
                time.sleep(1)

        console.print("\n[green]Servo test complete![/green]")
    except KeyboardInterrupt:
        console.print("\n[red]Interrupted! Stopping arm...[/red]")
        tester.stop_arm()
        sys.exit(1)


if __name__ == "__main__":
    app()
