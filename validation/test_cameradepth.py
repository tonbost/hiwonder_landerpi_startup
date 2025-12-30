#!/usr/bin/env python3
"""
Depth camera validation test for HiWonder LanderPi robot.
Tests Aurora 930 depth camera connectivity and functionality via ROS2 in Docker.

Key Information from HiWonder Documentation:
- Camera Model: Deptrum Aurora 930
- USB ID: VID:PID 3251:1930
- ROS2 Topics (namespace 'aurora'):
  - /aurora/rgb/image_raw (BGR8)
  - /aurora/ir/image_raw (MONO8)
  - /aurora/depth/image_raw (MONO16, millimeters)
  - /aurora/points2 (PointCloud2)
  - /aurora/rgb/camera_info, /aurora/ir/camera_info

Note: ROS2 runs inside Docker container on the robot.
"""
import sys
import time
import json
from pathlib import Path
from typing import Optional
from io import StringIO

import typer
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.prompt import Confirm
from fabric import Connection

app = typer.Typer(help="LanderPi Depth Camera Validation Test (Docker-based ROS2)")
console = Console()

# Camera configuration
CAMERA_USB_VID = "3251"
CAMERA_USB_PID = "1930"
CAMERA_NAMESPACE = "aurora"

# ROS2 Topics
TOPICS = {
    "rgb": f"/{CAMERA_NAMESPACE}/rgb/image_raw",
    "ir": f"/{CAMERA_NAMESPACE}/ir/image_raw",
    "depth": f"/{CAMERA_NAMESPACE}/depth/image_raw",
    "points": f"/{CAMERA_NAMESPACE}/points2",
    "rgb_info": f"/{CAMERA_NAMESPACE}/rgb/camera_info",
    "ir_info": f"/{CAMERA_NAMESPACE}/ir/camera_info",
}

# Docker configuration
DOCKER_IMAGE = "landerpi-ros2:latest"
DOCKER_RUN_BASE = "docker run --rm --privileged --network host -v /dev:/dev"


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


class DepthCameraTest:
    def __init__(self, host: str, user: str, password: Optional[str] = None):
        self.host = host
        self.user = user
        self.console = console
        self.docker_image = DOCKER_IMAGE

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

    def check_docker_available(self) -> bool:
        """Check if Docker is available on the robot."""
        try:
            result = self.conn.run("docker --version", hide=True, warn=True)
            if result.ok:
                self.console.print(f"[green]Docker available:[/green] [dim]{result.stdout.strip()}[/dim]")

                # Verify custom image exists
                result = self.conn.run(f"docker images -q {DOCKER_IMAGE}", hide=True, warn=True)
                if result.ok and result.stdout.strip():
                    self.console.print(f"[green]Using image:[/green] {DOCKER_IMAGE}")
                    return True
                else:
                    self.console.print(f"[red]Docker image not found:[/red] {DOCKER_IMAGE}")
                    self.console.print("[dim]Run 'uv run python setup_landerpi.py deploy' to build the image[/dim]")
                    return False

            self.console.print("[red]Docker not available[/red]")
            return False
        except Exception as e:
            self.console.print(f"[red]Docker check failed:[/red] {e}")
            return False

    def check_camera_device(self) -> bool:
        """Check if camera USB device exists."""
        try:
            result = self.conn.run(f"lsusb | grep {CAMERA_USB_VID}", hide=True, warn=True)
            if result.ok and result.stdout.strip():
                self.console.print(f"[green]Camera USB device found:[/green]")
                self.console.print(f"  [dim]{result.stdout.strip()}[/dim]")
                return True

            self.console.print(f"[red]Camera not detected (USB VID:{CAMERA_USB_VID})[/red]")
            self.console.print("[dim]Check USB connection and cable[/dim]")
            return False
        except Exception as e:
            self.console.print(f"[red]Camera check failed:[/red] {e}")
            return False

    def check_udev_rules(self) -> bool:
        """Check if udev rules are installed for the camera."""
        try:
            result = self.conn.run(
                "cat /etc/udev/rules.d/99-deptrum-libusb.rules 2>/dev/null",
                hide=True, warn=True
            )
            if result.ok and "3251" in result.stdout:
                self.console.print("[green]udev rules installed for Deptrum camera[/green]")
                return True

            self.console.print("[yellow]udev rules may not be installed[/yellow]")
            return False
        except Exception:
            return False

    def check_camera_workspace(self) -> bool:
        """Check if camera driver workspace exists."""
        try:
            result = self.conn.run(
                "test -d ~/deptrum_ws/install && echo 'exists'",
                hide=True, warn=True
            )
            if result.ok and "exists" in result.stdout:
                self.console.print("[green]Camera driver workspace found:[/green] ~/deptrum_ws")
                return True

            self.console.print("[yellow]Camera driver workspace not found[/yellow]")
            self.console.print("[dim]See DepthCameraHowTo.md for installation instructions[/dim]")
            return False
        except Exception:
            return False

    def run_ros2_command(self, cmd: str, timeout: int = 30) -> tuple[bool, str]:
        """Run a ROS2 command inside Docker container."""
        docker_cmd = (
            f"{DOCKER_RUN_BASE} -v ~/deptrum_ws:/deptrum_ws {self.docker_image} "
            f"bash -c 'source /opt/ros/humble/setup.bash && "
            f"source /deptrum_ws/install/local_setup.bash 2>/dev/null; {cmd}'"
        )
        try:
            result = self.conn.run(docker_cmd, hide=True, warn=True, timeout=timeout)
            return result.ok, result.stdout + result.stderr
        except Exception as e:
            return False, str(e)

    def check_ros2_available(self) -> bool:
        """Check if ROS2 is available via Docker."""
        try:
            self.console.print("[blue]Checking ROS2 in Docker...[/blue]")
            ok, output = self.run_ros2_command("ros2 topic list", timeout=30)
            if ok:
                self.console.print("[green]ROS2 available in Docker[/green]")
                return True

            self.console.print("[red]ROS2 not available in Docker[/red]")
            return False
        except Exception as e:
            self.console.print(f"[red]ROS2 check failed:[/red] {e}")
            return False

    def check_camera_topics(self) -> dict:
        """Check if camera topics exist (requires driver to be running)."""
        results = {}
        try:
            self.console.print("[blue]Checking camera topics...[/blue]")
            ok, output = self.run_ros2_command(
                f"ros2 topic list 2>/dev/null | grep -E '{CAMERA_NAMESPACE}'",
                timeout=15
            )

            if ok and output.strip():
                found_topics = output.strip().split('\n')
                self.console.print("[green]Camera topics found:[/green]")
                for topic in found_topics:
                    topic = topic.strip()
                    if topic:
                        self.console.print(f"  [dim]{topic}[/dim]")
                        # Determine which topic type this is
                        for name, expected in TOPICS.items():
                            if expected == topic:
                                results[name] = True

                return results
            else:
                self.console.print("[yellow]No camera topics found - driver may not be running[/yellow]")
                self.console.print("[dim]Use 'start-driver' command to start the camera driver[/dim]")
                return {}

        except Exception as e:
            self.console.print(f"[red]Topic check failed:[/red] {e}")
            return {}

    def start_camera_driver(self) -> bool:
        """Start the camera driver in a Docker container."""
        self.console.print("[blue]Starting camera driver in Docker...[/blue]")

        try:
            # Check if workspace exists
            if not self.check_camera_workspace():
                return False

            # Check if driver is already running
            result = self.conn.run(
                "docker ps --filter name=camera_driver -q",
                hide=True, warn=True
            )
            if result.ok and result.stdout.strip():
                self.console.print("[yellow]Camera driver already running[/yellow]")
                return True

            # Start the driver
            docker_cmd = (
                f"docker run -d --rm --privileged --network host "
                f"-v /dev:/dev -v ~/deptrum_ws:/deptrum_ws "
                f"--name camera_driver "
                f"{self.docker_image} "
                f"bash -c 'source /opt/ros/humble/setup.bash && "
                f"source /deptrum_ws/install/local_setup.bash && "
                f"ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py'"
            )

            result = self.conn.run(docker_cmd, hide=True, warn=True, timeout=30)

            if result.ok:
                container_id = result.stdout.strip()[:12]
                self.console.print(f"[green]Camera driver started[/green] (container: {container_id})")

                # Wait and check if it's running
                time.sleep(3)
                check_result = self.conn.run(
                    "docker ps --filter name=camera_driver -q",
                    hide=True, warn=True
                )
                if check_result.ok and check_result.stdout.strip():
                    self.console.print("[green]Driver is running[/green]")

                    # Show initial logs
                    log_result = self.conn.run(
                        "docker logs camera_driver 2>&1 | head -20",
                        hide=True, warn=True
                    )
                    if log_result.ok:
                        self.console.print("[dim]Initial output:[/dim]")
                        for line in log_result.stdout.split('\n')[:10]:
                            if line.strip():
                                self.console.print(f"  [dim]{line}[/dim]")
                    return True
                else:
                    # Driver crashed - show logs
                    log_result = self.conn.run(
                        "docker logs camera_driver 2>&1 | tail -10",
                        hide=True, warn=True
                    )
                    self.console.print("[red]Driver may have crashed:[/red]")
                    self.console.print(f"[dim]{log_result.stdout}[/dim]")
                    return False

            self.console.print("[red]Failed to start driver[/red]")
            return False

        except Exception as e:
            self.console.print(f"[red]Driver start error:[/red] {e}")
            return False

    def stop_camera_driver(self) -> bool:
        """Stop the camera driver container."""
        try:
            result = self.conn.run(
                "docker stop camera_driver 2>/dev/null",
                hide=True, warn=True, timeout=10
            )
            if result.ok:
                self.console.print("[green]Camera driver stopped[/green]")
                return True
            self.console.print("[yellow]Camera driver was not running[/yellow]")
            return True
        except Exception as e:
            self.console.print(f"[red]Stop error:[/red] {e}")
            return False

    def read_stream_data(self, samples: int = 3) -> dict:
        """Read camera stream data and validate."""
        self.console.print(f"[blue]Reading {samples} samples from camera streams...[/blue]")

        # Python script to read and analyze camera data
        validate_script = f'''
import sys
import time
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, PointCloud2, CameraInfo
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
except ImportError:
    print("ERROR: ROS2 Python packages not available")
    sys.exit(1)

class CameraValidator(Node):
    def __init__(self):
        super().__init__('camera_validator')
        self.results = {{}}
        self.sample_counts = {{'rgb': 0, 'ir': 0, 'depth': 0, 'points': 0, 'camera_info': 0}}
        self.max_samples = {samples}

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # RGB subscription
        self.rgb_sub = self.create_subscription(
            Image, '/{CAMERA_NAMESPACE}/rgb/image_raw', self.rgb_callback, qos)

        # IR subscription
        self.ir_sub = self.create_subscription(
            Image, '/{CAMERA_NAMESPACE}/ir/image_raw', self.ir_callback, qos)

        # Depth subscription
        self.depth_sub = self.create_subscription(
            Image, '/{CAMERA_NAMESPACE}/depth/image_raw', self.depth_callback, qos)

        # Point cloud subscription
        self.points_sub = self.create_subscription(
            PointCloud2, '/{CAMERA_NAMESPACE}/points2', self.points_callback, qos)

        # Camera info subscription
        self.info_sub = self.create_subscription(
            CameraInfo, '/{CAMERA_NAMESPACE}/rgb/camera_info', self.info_callback, qos)

    def rgb_callback(self, msg):
        if self.sample_counts['rgb'] >= self.max_samples:
            return
        self.sample_counts['rgb'] += 1
        self.results['rgb'] = {{
            'encoding': msg.encoding,
            'width': msg.width,
            'height': msg.height,
            'step': msg.step,
        }}
        print(f"RGB: {{msg.width}}x{{msg.height}} {{msg.encoding}}")

    def ir_callback(self, msg):
        if self.sample_counts['ir'] >= self.max_samples:
            return
        self.sample_counts['ir'] += 1
        self.results['ir'] = {{
            'encoding': msg.encoding,
            'width': msg.width,
            'height': msg.height,
        }}
        print(f"IR: {{msg.width}}x{{msg.height}} {{msg.encoding}}")

    def depth_callback(self, msg):
        if self.sample_counts['depth'] >= self.max_samples:
            return
        self.sample_counts['depth'] += 1

        # Analyze depth data
        data = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        valid_mask = (data > 150) & (data < 4000)
        valid_pct = np.sum(valid_mask) / data.size * 100

        if np.any(valid_mask):
            min_depth = np.min(data[valid_mask])
            max_depth = np.max(data[valid_mask])
            avg_depth = np.mean(data[valid_mask])
        else:
            min_depth = max_depth = avg_depth = 0

        self.results['depth'] = {{
            'encoding': msg.encoding,
            'width': msg.width,
            'height': msg.height,
            'valid_percentage': valid_pct,
            'min_depth_mm': int(min_depth),
            'max_depth_mm': int(max_depth),
            'avg_depth_mm': int(avg_depth),
        }}
        print(f"Depth: {{msg.width}}x{{msg.height}} valid={{valid_pct:.1f}}% range={{min_depth}}-{{max_depth}}mm")

    def points_callback(self, msg):
        if self.sample_counts['points'] >= self.max_samples:
            return
        self.sample_counts['points'] += 1
        self.results['points'] = {{
            'width': msg.width,
            'height': msg.height,
            'point_step': msg.point_step,
            'row_step': msg.row_step,
            'fields': [f.name for f in msg.fields],
        }}
        print(f"PointCloud: {{msg.width}}x{{msg.height}} step={{msg.point_step}}")

    def info_callback(self, msg):
        if self.sample_counts['camera_info'] >= self.max_samples:
            return
        self.sample_counts['camera_info'] += 1
        self.results['camera_info'] = {{
            'frame_id': msg.header.frame_id,
            'width': msg.width,
            'height': msg.height,
        }}
        print(f"CameraInfo: {{msg.width}}x{{msg.height}} frame={{msg.header.frame_id}}")

    def all_streams_received(self):
        return all(count > 0 for count in self.sample_counts.values())

def main():
    rclpy.init()
    node = CameraValidator()

    start = time.time()
    timeout = 15  # seconds

    while not node.all_streams_received() and (time.time() - start) < timeout:
        rclpy.spin_once(node, timeout_sec=0.5)

    # Print summary
    print("---SUMMARY---")
    for stream, data in node.results.items():
        if isinstance(data, dict):
            print(f"STREAM:{{stream}}")
            for key, val in data.items():
                print(f"  {{key}}:{{val}}")

    if node.all_streams_received():
        print("STATUS:SUCCESS")
    else:
        missing = [k for k, v in node.sample_counts.items() if v == 0]
        print(f"STATUS:PARTIAL")
        print(f"MISSING:{{','.join(missing)}}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

        try:
            # Upload the script to the robot
            import tempfile
            import os
            with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
                f.write(validate_script)
                temp_path = f.name
            try:
                self.conn.put(temp_path, "/tmp/camera_validate.py")
            finally:
                os.unlink(temp_path)

            # Run the script inside Docker container
            docker_cmd = (
                f"{DOCKER_RUN_BASE} -v /tmp:/tmp -v ~/deptrum_ws:/deptrum_ws {self.docker_image} "
                f"bash -c 'pip3 install -q numpy 2>/dev/null; "
                f"source /opt/ros/humble/setup.bash && "
                f"source /deptrum_ws/install/local_setup.bash 2>/dev/null; "
                f"python3 /tmp/camera_validate.py'"
            )

            result = self.conn.run(docker_cmd, hide=False, warn=True, timeout=30)

            if result.ok:
                # Parse results
                results = {}
                if "STATUS:SUCCESS" in result.stdout:
                    results['status'] = 'success'
                    self.console.print("\n[bold green]All camera streams validated![/bold green]")
                elif "STATUS:PARTIAL" in result.stdout:
                    results['status'] = 'partial'
                    # Extract missing streams
                    for line in result.stdout.split('\n'):
                        if line.startswith("MISSING:"):
                            missing = line.split(':')[1]
                            self.console.print(f"\n[yellow]Missing streams: {missing}[/yellow]")
                else:
                    results['status'] = 'failed'
                    self.console.print("\n[red]Camera validation failed[/red]")

                return results
            else:
                self.console.print("[red]Failed to run validation script[/red]")
                return {'status': 'error'}

        except Exception as e:
            self.console.print(f"[red]Validation error:[/red] {e}")
            return {'status': 'error', 'error': str(e)}

    def get_driver_logs(self, lines: int = 20) -> str:
        """Get recent logs from the camera driver container."""
        try:
            result = self.conn.run(
                f"docker logs camera_driver 2>&1 | tail -{lines}",
                hide=True, warn=True
            )
            return result.stdout if result.ok else ""
        except Exception:
            return ""


@app.command()
def check(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """
    Check camera connectivity, Docker, and driver availability.
    """
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required. Provide via options or config.json[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]Depth Camera System Check[/bold]\n\n"
        f"Host: {host}\n"
        f"Camera: Aurora 930 (USB {CAMERA_USB_VID}:{CAMERA_USB_PID})\n"
        f"ROS2 via: Docker",
        title="Camera Check",
        border_style="blue"
    ))

    tester = DepthCameraTest(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    checks = []

    # Check Docker (required for ROS2)
    docker_ok = tester.check_docker_available()
    checks.append(("Docker", docker_ok))

    # Check camera USB device
    device_ok = tester.check_camera_device()
    checks.append(("Camera USB", device_ok))

    # Check udev rules
    udev_ok = tester.check_udev_rules()
    checks.append(("udev Rules", udev_ok))

    # Check driver workspace
    workspace_ok = tester.check_camera_workspace()
    checks.append(("Driver Workspace", workspace_ok))

    # Check ROS2 via Docker
    if docker_ok:
        ros2_ok = tester.check_ros2_available()
        checks.append(("ROS2 in Docker", ros2_ok))

        # Check camera topics (only if driver might be running)
        if ros2_ok:
            topics = tester.check_camera_topics()
            topics_ok = len(topics) > 0
            checks.append(("Camera Topics", topics_ok))

    # Summary
    console.print("\n[bold]Summary:[/bold]")
    all_ok = True
    warnings = 0
    for name, ok in checks:
        if ok:
            status = "[green]PASS[/green]"
        elif name in ["Camera Topics", "udev Rules"]:
            status = "[yellow]WARN[/yellow]"
            warnings += 1
        else:
            status = "[red]FAIL[/red]"
            all_ok = False
        console.print(f"  {name}: {status}")

    if all_ok and warnings > 0:
        console.print("\n[yellow]Note: Start camera driver with 'start-driver' command[/yellow]")
        sys.exit(0)

    sys.exit(0 if all_ok else 1)


@app.command("start-driver")
def start_driver(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """
    Start the camera driver in a Docker container.
    This publishes camera data to ROS2 topics.
    """
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]Start Camera Driver[/bold]\n\n"
        f"Host: {host}\n"
        f"Camera: Aurora 930\n"
        f"Running in: Docker container",
        title="Start Driver",
        border_style="green"
    ))

    tester = DepthCameraTest(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    if not tester.check_docker_available():
        console.print("[red]Docker is required to run the camera driver[/red]")
        sys.exit(1)

    if not tester.check_camera_device():
        console.print("[red]Camera not detected[/red]")
        sys.exit(1)

    if tester.start_camera_driver():
        console.print("\n[bold green]Camera driver started![/bold green]")
        console.print("[dim]Use 'stream' command to read data, 'stop-driver' to stop[/dim]")
    else:
        console.print("\n[bold red]Failed to start camera driver[/bold red]")
        sys.exit(1)


@app.command("stop-driver")
def stop_driver(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """
    Stop the camera driver Docker container.
    """
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print("[bold yellow]Stopping camera driver...[/bold yellow]")
    tester = DepthCameraTest(host, user, password)

    if tester.verify_connection():
        tester.stop_camera_driver()


@app.command()
def stream(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    samples: int = typer.Option(3, help="Number of samples to read per stream"),
):
    """
    Read camera stream data and display information.
    """
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]Camera Stream Test[/bold]\n\n"
        f"Host: {host}\n"
        f"Samples: {samples}",
        title="Stream Test",
        border_style="blue"
    ))

    tester = DepthCameraTest(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    results = tester.read_stream_data(samples)

    if results.get('status') == 'success':
        console.print("\n[bold green]Camera stream test complete![/bold green]")
    elif results.get('status') == 'partial':
        console.print("\n[yellow]Partial data received - some streams may not be publishing[/yellow]")
        sys.exit(1)
    else:
        console.print("\n[red]Failed to read camera streams[/red]")
        console.print("[dim]Make sure the camera driver is running (start-driver)[/dim]")
        sys.exit(1)


@app.command()
def validate(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """
    Run full camera validation (check + stream test).
    """
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]Full Camera Validation[/bold]\n\n"
        f"Host: {host}\n"
        f"Camera: Aurora 930",
        title="Validation",
        border_style="cyan"
    ))

    tester = DepthCameraTest(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    # Run all checks
    console.print("\n[bold]Step 1: System Checks[/bold]\n")

    checks = []
    docker_ok = tester.check_docker_available()
    checks.append(("Docker", docker_ok))

    device_ok = tester.check_camera_device()
    checks.append(("Camera USB", device_ok))

    udev_ok = tester.check_udev_rules()
    checks.append(("udev Rules", udev_ok))

    workspace_ok = tester.check_camera_workspace()
    checks.append(("Driver Workspace", workspace_ok))

    if not all([docker_ok, device_ok, workspace_ok]):
        console.print("\n[red]Prerequisites not met. Fix issues above before continuing.[/red]")
        sys.exit(1)

    # Check if driver is running, start if not
    console.print("\n[bold]Step 2: Driver Status[/bold]\n")

    topics = tester.check_camera_topics()
    if not topics:
        console.print("[yellow]Driver not running, attempting to start...[/yellow]")
        if not tester.start_camera_driver():
            console.print("[red]Failed to start camera driver[/red]")
            sys.exit(1)
        time.sleep(3)  # Wait for driver to initialize

    # Read and validate streams
    console.print("\n[bold]Step 3: Stream Validation[/bold]\n")

    results = tester.read_stream_data(samples=3)

    # Final summary
    console.print("\n" + "=" * 50)
    console.print("[bold]Validation Summary[/bold]")
    console.print("=" * 50)

    table = Table(show_header=True, header_style="bold cyan")
    table.add_column("Component", style="dim")
    table.add_column("Status", justify="center")

    for name, ok in checks:
        status = "[green]PASS[/green]" if ok else "[red]FAIL[/red]"
        table.add_row(name, status)

    stream_status = "[green]PASS[/green]" if results.get('status') == 'success' else "[yellow]PARTIAL[/yellow]" if results.get('status') == 'partial' else "[red]FAIL[/red]"
    table.add_row("Stream Data", stream_status)

    console.print(table)

    if results.get('status') == 'success':
        console.print("\n[bold green]Camera validation PASSED![/bold green]")
        sys.exit(0)
    else:
        console.print("\n[bold yellow]Camera validation completed with warnings[/bold yellow]")
        sys.exit(1)


@app.command()
def logs(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    lines: int = typer.Option(30, help="Number of log lines to show"),
):
    """
    Show camera driver logs.
    """
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    tester = DepthCameraTest(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    logs_output = tester.get_driver_logs(lines)
    if logs_output:
        console.print("[bold]Camera Driver Logs:[/bold]\n")
        console.print(logs_output)
    else:
        console.print("[yellow]No logs available (driver may not be running)[/yellow]")


if __name__ == "__main__":
    app()
