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
import signal
import sys
import time
from pathlib import Path
from typing import Optional

import typer
from fabric import Connection
from rich.console import Console
from rich.panel import Panel
from rich.prompt import Confirm

from exploration import (
    ExplorationController,
    SafetyConfig,
    RemoteDataLogger,
    RemoteLogConfig,
    ArmScanner,
    ArmScannerConfig,
)

app = typer.Typer(help="LanderPi Autonomous Exploration (Sensor Fusion)")
console = Console()

# Docker configuration
DOCKER_IMAGE = "landerpi-ros2:latest"


def load_config() -> dict:
    """Load connection config from config.json."""
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

    def run_remote_command(self, cmd: str) -> tuple:
        """Execute command on robot. Returns (success, output)."""
        try:
            result = self.conn.run(cmd, hide=True, warn=True, timeout=10)
            return result.ok, result.stdout.strip()
        except Exception as e:
            return False, str(e)

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

        # Check camera topic (if camera USB detected)
        if checks[-1][1]:  # If camera USB found
            result = self.conn.run(
                "docker exec landerpi-ros2 bash -c '"
                "source /opt/ros/humble/setup.bash && "
                "source /deptrum_ws/install/local_setup.bash 2>/dev/null; "
                "ros2 topic list 2>/dev/null | grep /aurora/depth'",
                hide=True, warn=True
            )
            checks.append(("Depth topic", result.ok))

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
        # Script to read one scan - outputs JSON
        scan_script = """
import sys
import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time

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
start = time.time()
while node.data is None and (time.time() - start) < 2.0:
    rclpy.spin_once(node, timeout_sec=0.1)
if node.data:
    print(json.dumps(node.data))
node.destroy_node()
rclpy.shutdown()
"""
        try:
            # Write script to temp file on robot and copy into container
            self.conn.run(f"cat > /tmp/read_scan.py << 'SCRIPT_EOF'\n{scan_script}\nSCRIPT_EOF", hide=True)
            self.conn.run("docker cp /tmp/read_scan.py landerpi-ros2:/tmp/read_scan.py", hide=True)

            # Run inside the already-running landerpi-ros2 container (not a new one)
            result = self.conn.run(
                "docker exec landerpi-ros2 bash -c '"
                "source /opt/ros/humble/setup.bash && "
                "source /ros2_ws/install/setup.bash 2>/dev/null; "
                "python3 /tmp/read_scan.py'",
                hide=True, warn=True, timeout=10
            )
            if result.ok and result.stdout.strip():
                data = json.loads(result.stdout.strip())
                return data['ranges'], data['angle_min'], data['angle_increment']
        except Exception:
            pass
        return [], 0.0, 0.0

    def read_depth_image(self) -> tuple:
        """Read depth image from camera. Returns (depth_data, width, height)."""
        # Script to read one depth image - outputs JSON with dimensions and data
        depth_script = """
import sys
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time

class DepthReader(Node):
    def __init__(self):
        super().__init__('depth_reader')
        self.data = None
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Image, '/aurora/depth/image_raw', self.cb, qos)

    def cb(self, msg):
        if self.data is None:
            # MONO16 = 2 bytes per pixel, little-endian
            width = msg.width
            height = msg.height
            # Extract depth values from raw data
            depths = []
            for i in range(0, len(msg.data), 2):
                if i + 1 < len(msg.data):
                    # Little-endian 16-bit
                    val = msg.data[i] | (msg.data[i + 1] << 8)
                    depths.append(val)
            self.data = {
                'width': width,
                'height': height,
                'depths': depths,
            }

rclpy.init()
node = DepthReader()
start = time.time()
while node.data is None and (time.time() - start) < 2.0:
    rclpy.spin_once(node, timeout_sec=0.1)
if node.data:
    # Only output dimensions and sample to reduce data size
    # For fusion, we only need center region anyway
    w, h = node.data['width'], node.data['height']
    depths = node.data['depths']
    # Extract center 200x160 region (larger than fusion needs)
    cx, cy = w // 2, h // 2
    hw, hh = 100, 80
    center_depths = []
    for y in range(max(0, cy - hh), min(h, cy + hh)):
        for x in range(max(0, cx - hw), min(w, cx + hw)):
            idx = y * w + x
            if idx < len(depths):
                center_depths.append(depths[idx])
    out = {
        'width': min(hw * 2, w),
        'height': min(hh * 2, h),
        'depths': center_depths,
    }
    print(json.dumps(out))
node.destroy_node()
rclpy.shutdown()
"""
        try:
            # Write script to temp file on robot and copy into container
            self.conn.run(f"cat > /tmp/read_depth.py << 'SCRIPT_EOF'\n{depth_script}\nSCRIPT_EOF", hide=True)
            self.conn.run("docker cp /tmp/read_depth.py landerpi-ros2:/tmp/read_depth.py", hide=True)

            # Run inside the already-running landerpi-ros2 container
            result = self.conn.run(
                "docker exec landerpi-ros2 bash -c '"
                "source /opt/ros/humble/setup.bash && "
                "source /ros2_ws/install/setup.bash 2>/dev/null; "
                "source /deptrum_ws/install/local_setup.bash 2>/dev/null; "
                "python3 /tmp/read_depth.py'",
                hide=True, warn=True, timeout=10
            )
            if result.ok and result.stdout.strip():
                data = json.loads(result.stdout.strip())
                return data['depths'], data['width'], data['height']
        except Exception:
            pass
        return [], 0, 0

    def move_arm_servo(self, servo_id: int, position: int, duration: float = 0.8) -> bool:
        """Move a single arm servo to a position."""
        cmd = json.dumps({
            "action": "set_position",
            "duration": duration,
            "positions": [[servo_id, position]]
        })
        script = f'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

rclpy.init()
node = Node("arm_cmd_pub")
pub = node.create_publisher(String, "/arm/cmd", 10)
time.sleep(0.3)
msg = String()
msg.data = {repr(cmd)}
pub.publish(msg)
rclpy.spin_once(node, timeout_sec=0.3)
node.destroy_node()
rclpy.shutdown()
print("OK")
'''
        try:
            self.conn.run(f"cat > /tmp/arm_move.py << 'SCRIPT_EOF'\n{script}\nSCRIPT_EOF", hide=True)
            self.conn.run("docker cp /tmp/arm_move.py landerpi-ros2:/tmp/arm_move.py", hide=True)
            result = self.conn.run(
                "docker exec landerpi-ros2 bash -c '"
                "source /opt/ros/humble/setup.bash && "
                "source /ros2_ws/install/setup.bash && "
                "python3 /tmp/arm_move.py'",
                hide=True, warn=True, timeout=5
            )
            return result.ok and "OK" in result.stdout
        except Exception:
            return False

    def get_depth_stats(self) -> Optional[dict]:
        """Get depth statistics from camera for arm scanner."""
        depths, width, height = self.read_depth_image()
        if not depths:
            return None

        # Filter valid depths (150mm - 3000mm)
        valid_depths = [d for d in depths if 150 <= d <= 3000]
        if not valid_depths:
            return None

        min_depth = min(valid_depths) / 1000.0  # Convert to meters
        avg_depth = sum(valid_depths) / len(valid_depths) / 1000.0
        valid_percent = len(valid_depths) / len(depths) * 100.0

        return {
            "min_depth": min_depth,
            "avg_depth": avg_depth,
            "valid_percent": valid_percent,
        }

    def create_arm_scanner(self) -> Optional[ArmScanner]:
        """Create an ArmScanner instance if arm topic is available."""
        # Check if arm topic exists
        try:
            result = self.conn.run(
                "docker exec landerpi-ros2 bash -c '"
                "source /opt/ros/humble/setup.bash && "
                "source /ros2_ws/install/setup.bash && "
                "ros2 topic list | grep /arm/cmd'",
                hide=True, warn=True
            )
            if not result.ok:
                console.print("[yellow]Arm topic not found - arm scanning disabled[/yellow]")
                return None
        except Exception:
            return None

        console.print("[green]Arm scanner enabled[/green]")
        return ArmScanner(
            arm_move_func=self.move_arm_servo,
            get_depth_func=self.get_depth_stats,
        )

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

        # Create remote logger (logs to robot, not local machine)
        remote_log_config = RemoteLogConfig(rosbag_enabled=enable_rosbag)
        remote_logger = RemoteDataLogger(
            config=remote_log_config,
            run_command=self.run_remote_command,
        )

        # Create arm scanner for escape maneuvers (optional)
        arm_scanner = self.create_arm_scanner()

        # Create controller with remote logger and arm scanner
        self.controller = ExplorationController(
            move_func=self.move,
            stop_func=self.stop_motors,
            get_battery_func=self.get_battery,
            safety_config=safety_config,
            logger=remote_logger,
            arm_scanner=arm_scanner,  # For progressive escape
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
                depth_data, depth_width, depth_height = self.read_depth_image()

                if ranges:
                    self.controller.update_sensors(
                        ranges, angle_min, angle_inc,
                        depth_data=depth_data if depth_data else None,
                        depth_width=depth_width,
                        depth_height=depth_height
                    )

                # Step controller
                if not self.controller.step():
                    break

                # Periodic status
                if time.time() - last_status_time >= status_period:
                    last_status_time = time.time()
                    status = self.controller.get_status()
                    remaining = status['safety']['remaining_minutes']
                    sensors = status.get('sensors', {})
                    depth_m = sensors.get('depth_m', 'N/A')
                    lidar_m = sensors.get('lidar_m', 'N/A')
                    console.print(
                        f"[dim]Status: {remaining:.1f} min remaining | "
                        f"depth: {depth_m}m, lidar: {lidar_m}m | "
                        f"action: {status['action']}[/dim]"
                    )

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
