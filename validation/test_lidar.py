#!/usr/bin/env python3
"""
Lidar validation test for HiWonder LanderPi robot.
Tests MS200/LD19 lidar connectivity and functionality via ROS2 in Docker.

Key Information from HiWonder Documentation:
- Default Lidar: MS200 (or LD19)
- Serial Port: /dev/ldlidar (baud: 230400)
- ROS2 Topics: /scan_raw (raw), /scan (filtered)
- Lidar Modes via /lidar_app/set_running:
  - 0: Stop
  - 1: Obstacle avoidance (turns to avoid obstacles)
  - 2: Tracking (follows objects at 35cm distance)
  - 3: Guard (rotates to face detected objects)

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

app = typer.Typer(help="LanderPi Lidar Validation Test (Docker-based ROS2)")
console = Console()

# Lidar configuration
LIDAR_DEVICE = "/dev/ldlidar"
LIDAR_DEVICE_ALT = "/dev/ttyUSB0"  # Fallback if symlink doesn't exist
LIDAR_BAUD = 230400
SCAN_TOPIC = "/scan"

# Docker configuration - always use custom image (has pip3, pyserial support)
DOCKER_IMAGE = "landerpi-ros2:latest"
DOCKER_IMAGE_FALLBACK = "ros:humble-ros-base"
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


class LidarTest:
    def __init__(self, host: str, user: str, password: Optional[str] = None):
        self.host = host
        self.user = user
        self.console = console
        self.docker_image = DOCKER_IMAGE  # Will be updated if custom image exists
        self.lidar_device = LIDAR_DEVICE  # Will be updated based on what exists

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

                # Verify custom image exists, fall back to base if not
                result = self.conn.run(f"docker images -q {DOCKER_IMAGE}", hide=True, warn=True)
                if result.ok and result.stdout.strip():
                    self.console.print(f"[green]Using image:[/green] {DOCKER_IMAGE}")
                else:
                    self.docker_image = DOCKER_IMAGE_FALLBACK
                    self.console.print(f"[yellow]Custom image not found, using:[/yellow] {DOCKER_IMAGE_FALLBACK}")
                return True

            self.console.print("[red]Docker not available[/red]")
            return False
        except Exception as e:
            self.console.print(f"[red]Docker check failed:[/red] {e}")
            return False

    def check_lidar_device(self) -> bool:
        """Check if lidar device exists."""
        try:
            # Check for /dev/ldlidar symlink
            result = self.conn.run(f"ls -la {LIDAR_DEVICE}", hide=True, warn=True)
            if result.ok:
                self.console.print(f"[green]Lidar device found: {LIDAR_DEVICE}[/green]")
                self.console.print(f"[dim]{result.stdout.strip()}[/dim]")
                self.lidar_device = LIDAR_DEVICE
                return True

            # Check for any ttyUSB devices
            result = self.conn.run("ls /dev/ttyUSB* 2>/dev/null", hide=True, warn=True)
            if result.ok:
                devices = result.stdout.strip().split('\n')
                self.console.print(f"[yellow]Lidar symlink not found, but USB serial devices exist:[/yellow]")
                for dev in devices:
                    self.console.print(f"  [dim]{dev}[/dim]")
                # Use first USB device as lidar
                self.lidar_device = devices[0]
                return True

            self.console.print(f"[red]No lidar device found at {LIDAR_DEVICE} or /dev/ttyUSB*[/red]")
            return False
        except Exception as e:
            self.console.print(f"[red]Device check failed:[/red] {e}")
            return False

    def get_lidar_type(self) -> Optional[str]:
        """Get configured lidar type from environment."""
        try:
            result = self.conn.run("echo $LIDAR_TYPE", hide=True, warn=True)
            if result.ok and result.stdout.strip():
                lidar_type = result.stdout.strip()
                self.console.print(f"[blue]Lidar type: {lidar_type}[/blue]")
                return lidar_type
            self.console.print("[dim]LIDAR_TYPE not set (default: MS200)[/dim]")
            return "MS200"
        except Exception:
            return None

    def run_ros2_command(self, cmd: str, timeout: int = 30) -> tuple[bool, str]:
        """Run a ROS2 command inside Docker container."""
        docker_cmd = (
            f"{DOCKER_RUN_BASE} {self.docker_image} "
            f"bash -c 'source /opt/ros/humble/setup.bash && {cmd}'"
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
            # Use 'ros2 topic list' as version check doesn't work in Humble
            ok, output = self.run_ros2_command("ros2 topic list", timeout=30)
            if ok:
                self.console.print(f"[green]ROS2 available in Docker[/green]")
                return True

            self.console.print("[red]ROS2 not available in Docker[/red]")
            self.console.print(f"[dim]{output}[/dim]")
            return False
        except Exception as e:
            self.console.print(f"[red]ROS2 check failed:[/red] {e}")
            return False

    def check_scan_topic(self) -> bool:
        """Check if lidar scan topic exists (requires lidar node to be running)."""
        try:
            self.console.print(f"[blue]Checking for scan topics...[/blue]")

            ok, output = self.run_ros2_command("ros2 topic list 2>/dev/null | grep -E 'scan'", timeout=15)

            if ok and output.strip():
                topics = output.strip().split('\n')
                self.console.print("[green]Lidar topics found:[/green]")
                for topic in topics:
                    if topic.strip():
                        self.console.print(f"  [dim]{topic.strip()}[/dim]")
                return True

            self.console.print("[yellow]No scan topics found - lidar node may not be running[/yellow]")
            self.console.print("[dim]Note: You may need to start the lidar driver first[/dim]")
            return False
        except Exception as e:
            self.console.print(f"[red]Topic check failed:[/red] {e}")
            return False

    def read_scan_data(self, samples: int = 5, topic: str = None) -> bool:
        """Read lidar scan data and display statistics using Docker."""
        scan_topic = topic or SCAN_TOPIC
        self.console.print(f"[blue]Reading {samples} scan samples from {scan_topic}...[/blue]")

        # Python script to read and analyze scan data
        scan_script = f'''
import sys
import time
import math

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
except ImportError:
    print("ERROR: ROS2 Python packages not available")
    sys.exit(1)

class ScanReader(Node):
    def __init__(self):
        super().__init__('scan_reader')
        self.samples = []
        self.max_samples = {samples}

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            LaserScan,
            '{scan_topic}',
            self.scan_callback,
            qos
        )

    def scan_callback(self, msg):
        if len(self.samples) >= self.max_samples:
            return

        ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max and not math.isinf(r) and not math.isnan(r)]

        if ranges:
            sample = {{
                'min_range': min(ranges),
                'max_range': max(ranges),
                'avg_range': sum(ranges) / len(ranges),
                'num_points': len(ranges),
                'total_points': len(msg.ranges),
                'angle_min': math.degrees(msg.angle_min),
                'angle_max': math.degrees(msg.angle_max),
                'angle_increment': math.degrees(msg.angle_increment),
            }}
            self.samples.append(sample)
            print(f"Sample {{len(self.samples)}}: min={{sample['min_range']:.2f}}m, max={{sample['max_range']:.2f}}m, points={{sample['num_points']}}/{{sample['total_points']}}")

def main():
    rclpy.init()
    node = ScanReader()

    start = time.time()
    timeout = 10  # seconds

    while len(node.samples) < node.max_samples and (time.time() - start) < timeout:
        rclpy.spin_once(node, timeout_sec=0.5)

    if node.samples:
        # Summary statistics
        min_dist = min(s['min_range'] for s in node.samples)
        max_dist = max(s['max_range'] for s in node.samples)
        avg_dist = sum(s['avg_range'] for s in node.samples) / len(node.samples)
        avg_points = sum(s['num_points'] for s in node.samples) / len(node.samples)

        print("---SUMMARY---")
        print(f"MIN_DIST:{{min_dist:.3f}}")
        print(f"MAX_DIST:{{max_dist:.3f}}")
        print(f"AVG_DIST:{{avg_dist:.3f}}")
        print(f"AVG_POINTS:{{avg_points:.0f}}")
        print(f"ANGLE_MIN:{{node.samples[0]['angle_min']:.1f}}")
        print(f"ANGLE_MAX:{{node.samples[0]['angle_max']:.1f}}")
        print(f"ANGLE_INC:{{node.samples[0]['angle_increment']:.3f}}")
        print("SUCCESS")
    else:
        print("ERROR: No scan data received")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

        try:
            # Upload the script to the robot
            self.conn.put(StringIO(scan_script), "/tmp/read_scan.py")

            # Run the script inside Docker container
            docker_cmd = (
                f"{DOCKER_RUN_BASE} -v /tmp:/tmp {self.docker_image} "
                f"bash -c 'source /opt/ros/humble/setup.bash && python3 /tmp/read_scan.py'"
            )

            result = self.conn.run(docker_cmd, hide=False, warn=True, timeout=30)

            if result.ok and "SUCCESS" in result.stdout:
                # Parse summary
                lines = result.stdout.split('\n')
                summary = {}
                for line in lines:
                    if ':' in line and line.startswith(('MIN_', 'MAX_', 'AVG_', 'ANGLE_')):
                        key, val = line.split(':')
                        summary[key] = float(val)

                if summary:
                    self.console.print("\n[bold green]Lidar Scan Summary:[/bold green]")
                    table = Table(show_header=True, header_style="bold cyan")
                    table.add_column("Metric", style="dim")
                    table.add_column("Value", justify="right")

                    table.add_row("Minimum Distance", f"{summary.get('MIN_DIST', 0):.3f} m")
                    table.add_row("Maximum Distance", f"{summary.get('MAX_DIST', 0):.3f} m")
                    table.add_row("Average Distance", f"{summary.get('AVG_DIST', 0):.3f} m")
                    table.add_row("Avg Points/Scan", f"{summary.get('AVG_POINTS', 0):.0f}")
                    table.add_row("Angle Range", f"{summary.get('ANGLE_MIN', 0):.1f}° to {summary.get('ANGLE_MAX', 0):.1f}°")
                    table.add_row("Angle Increment", f"{summary.get('ANGLE_INC', 0):.3f}°")

                    self.console.print(table)
                return True
            else:
                self.console.print("[red]Failed to read scan data[/red]")
                if "No scan data received" in result.stdout:
                    self.console.print("[yellow]Tip: Make sure the lidar driver is running first[/yellow]")
                return False

        except Exception as e:
            self.console.print(f"[red]Scan read error:[/red] {e}")
            return False

    def start_lidar_driver(self, lidar_type: str = "MS200") -> bool:
        """
        Start the lidar driver in a Docker container.
        Returns the container ID if successful.
        """
        self.console.print(f"[blue]Starting {lidar_type} lidar driver in Docker...[/blue]")

        # Determine the correct port
        port = self.lidar_device

        # Python script to run the lidar driver
        # Proper LD19/MS200 driver that accumulates full scans
        driver_script = f'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import struct
import math
import time
import threading

class LD19Driver(Node):
    """LD19/MS200 lidar driver - accumulates full 360° scans before publishing."""

    PACKET_HEADER = 0x54
    PACKET_VER_LEN = 0x2C  # 12-point packet
    PACKET_SIZE = 47

    def __init__(self):
        super().__init__('ld19_driver')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.port = "{port}"
        self.baudrate = {LIDAR_BAUD}
        self.running = True

        # Accumulate 360 degrees of data
        self.ranges = [float('inf')] * 360
        self.last_start_angle = None
        self.scan_complete = False

        self.get_logger().info(f"Opening {{self.port}} at {{self.baudrate}} baud")

        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info("Serial port opened successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {{e}}")
            raise

        # Start reading thread
        self.thread = threading.Thread(target=self.read_loop)
        self.thread.daemon = True
        self.thread.start()

        # Publish timer at 10Hz
        self.timer = self.create_timer(0.1, self.publish_scan)

    def read_loop(self):
        """Read and parse lidar data packets."""
        buffer = bytearray()
        while self.running:
            try:
                data = self.serial.read(512)
                if data:
                    buffer.extend(data)
                    self.process_buffer(buffer)
            except Exception as e:
                time.sleep(0.05)

    def process_buffer(self, buffer):
        """Process buffer for LD19 packets."""
        while len(buffer) >= self.PACKET_SIZE:
            # Find packet header
            try:
                idx = buffer.index(self.PACKET_HEADER)
                if idx > 0:
                    del buffer[:idx]
                    continue
            except ValueError:
                buffer.clear()
                return

            if len(buffer) < self.PACKET_SIZE:
                return

            # Validate packet type
            if buffer[1] != self.PACKET_VER_LEN:
                del buffer[:1]
                continue

            # Parse packet
            try:
                # Speed (unused for now)
                # speed = struct.unpack('<H', bytes(buffer[2:4]))[0]

                # Start and end angles (in 0.01 degrees)
                start_angle = struct.unpack('<H', bytes(buffer[4:6]))[0] / 100.0
                end_angle = struct.unpack('<H', bytes(buffer[42:44]))[0] / 100.0

                # Calculate angle step
                if end_angle < start_angle:
                    angle_span = (360.0 - start_angle) + end_angle
                else:
                    angle_span = end_angle - start_angle

                if angle_span <= 0 or angle_span > 60:
                    del buffer[:1]
                    continue

                angle_step = angle_span / 11.0  # 12 points = 11 steps

                # Extract 12 distance measurements
                for i in range(12):
                    offset = 6 + i * 3
                    dist_mm = struct.unpack('<H', bytes(buffer[offset:offset+2]))[0]
                    # intensity = buffer[offset + 2]  # unused

                    angle = (start_angle + i * angle_step) % 360.0
                    angle_idx = int(angle) % 360

                    if 50 < dist_mm < 12000:  # Valid range: 5cm to 12m
                        self.ranges[angle_idx] = dist_mm / 1000.0

                del buffer[:self.PACKET_SIZE]

            except Exception:
                del buffer[:1]

    def publish_scan(self):
        """Publish accumulated scan data."""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = 2.0 * math.pi / 360.0
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.05
        msg.range_max = 12.0
        msg.ranges = list(self.ranges)

        self.publisher.publish(msg)

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'serial'):
            self.serial.close()
        super().destroy_node()

def main():
    rclpy.init()
    try:
        node = LD19Driver()
        print("DRIVER_STARTED")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"ERROR: {{e}}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

        try:
            # Upload the driver script via temp file (avoids StringIO size mismatch)
            import tempfile
            import os
            with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
                f.write(driver_script)
                temp_path = f.name
            try:
                self.conn.put(temp_path, "/tmp/lidar_driver.py")
            finally:
                os.unlink(temp_path)

            # Install pyserial in the container and run driver
            docker_cmd = (
                f"docker run -d --rm --privileged --network host "
                f"-v /dev:/dev -v /tmp:/tmp --name lidar_driver "
                f"{self.docker_image} "
                f"bash -c 'pip3 install pyserial -q 2>/dev/null; source /opt/ros/humble/setup.bash && "
                f"python3 /tmp/lidar_driver.py'"
            )

            result = self.conn.run(docker_cmd, hide=True, warn=True, timeout=30)

            if result.ok:
                container_id = result.stdout.strip()[:12]
                self.console.print(f"[green]Lidar driver started[/green] (container: {container_id})")

                # Wait a moment and check if it's running
                time.sleep(2)
                check_result = self.conn.run("docker ps --filter name=lidar_driver -q", hide=True, warn=True)
                if check_result.ok and check_result.stdout.strip():
                    self.console.print("[green]Driver is running[/green]")
                    return True
                else:
                    # Check logs
                    log_result = self.conn.run("docker logs lidar_driver 2>&1 | tail -5", hide=True, warn=True)
                    self.console.print(f"[red]Driver may have crashed:[/red]")
                    self.console.print(f"[dim]{log_result.stdout}[/dim]")
                    return False

            self.console.print(f"[red]Failed to start driver[/red]")
            return False

        except Exception as e:
            self.console.print(f"[red]Driver start error:[/red] {e}")
            return False

    def stop_lidar_driver(self) -> bool:
        """Stop the lidar driver container."""
        try:
            result = self.conn.run("docker stop lidar_driver 2>/dev/null", hide=True, warn=True, timeout=10)
            if result.ok:
                self.console.print("[green]Lidar driver stopped[/green]")
                return True
            self.console.print("[yellow]Lidar driver was not running[/yellow]")
            return True
        except Exception as e:
            self.console.print(f"[red]Stop error:[/red] {e}")
            return False

    def test_lidar_mode(self, mode: int, duration: float = 5.0) -> bool:
        """
        Test lidar functionality mode using standalone Python (no HiWonder dependency).

        Modes:
            0: Stop
            1: Obstacle avoidance (turns to avoid obstacles)
            2: Tracking (follows objects at 35cm distance)
            3: Guard (rotates to face detected objects)
        """
        mode_names = {
            0: "Stop",
            1: "Obstacle Avoidance",
            2: "Tracking",
            3: "Guard"
        }

        mode_name = mode_names.get(mode, f"Unknown ({mode})")
        self.console.print(f"[bold yellow]Testing mode {mode}: {mode_name}[/bold yellow]")

        # Check for motor controller SDK
        result = self.conn.run("test -f ~/ros_robot_controller/ros_robot_controller_sdk.py", hide=True, warn=True)
        if not result.ok:
            self.console.print("[red]Motor controller SDK not found at ~/ros_robot_controller/[/red]")
            return False

        # ROS2-based lidar mode controller running in Docker
        # Based on HiWonder lidar_controller.py reference implementation
        mode_script = f'''
#!/usr/bin/env python3
"""
ROS2-based lidar mode controller.
Based on HiWonder lidar_controller.py reference implementation.
Subscribes to /scan topic and controls motors via SDK.
"""
import sys
import math
import time
import numpy as np
sys.path.insert(0, "/ros_robot_controller")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Import motor SDK
from ros_robot_controller_sdk import Board

# Control parameters (from HiWonder reference)
SPEED = 0.2              # m/s forward speed
THRESHOLD = 0.6          # meters - obstacle threshold
MAX_SCAN_ANGLE = 120     # degrees - scanning angle (60 each side)
TRACKING_DISTANCE = 0.35 # meters - following distance


class PID:
    """Simple PID controller (from HiWonder reference)."""

    def __init__(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0

    def update(self, feedback):
        error = self.SetPoint - feedback
        self.PTerm = self.Kp * error
        self.ITerm += error
        self.DTerm = self.Kd * (error - self.last_error)
        self.last_error = error
        self.output = self.PTerm + self.Ki * self.ITerm + self.DTerm
        return self.output

    def clear(self):
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0


class MecanumController:
    """Control mecanum wheels via SDK."""

    def __init__(self):
        self.board = Board()
        self.board.enable_reception()

    def move(self, vx, wz):
        """Move robot: vx=forward velocity, wz=angular velocity."""
        left_speed = -vx - wz * 0.15
        right_speed = vx - wz * 0.15
        max_speed = 0.3
        left_speed = max(-max_speed, min(max_speed, left_speed))
        right_speed = max(-max_speed, min(max_speed, right_speed))
        self.board.set_motor_speed([
            [1, left_speed], [2, left_speed],
            [3, right_speed], [4, right_speed]
        ])

    def stop(self):
        self.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])


def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))


class LidarModeController(Node):
    """ROS2 node for lidar-based robot control modes."""

    def __init__(self, mode, duration):
        super().__init__('lidar_mode_controller')
        self.mode = mode
        self.duration = duration
        self.start_time = time.time()

        # Motor controller
        self.motor = MecanumController()
        self.motor.board.set_buzzer(1000, 0.1, 0.1, 1)

        # State for obstacle avoidance
        self.last_act = 0
        self.timestamp = 0

        # PID controllers
        self.pid_yaw = PID(1.6, 0, 0.16)
        self.pid_dist = PID(1.7, 0, 0.16)

        # Subscribe to lidar with BEST_EFFORT QoS (standard for sensor data)
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos)

        self.get_logger().info(f'Starting mode {{mode}} for {{duration}}s')

    def get_left_right_ranges(self, ranges, angle_increment):
        """Extract left and right range arrays from LaserScan data."""
        # Convert to numpy, handle inf/nan
        ranges = np.array(ranges)
        max_index = int(math.radians(MAX_SCAN_ANGLE / 2) / angle_increment)

        # Left side (counterclockwise from front)
        left_ranges = ranges[:max_index]
        # Right side (clockwise from front, reversed)
        right_ranges = ranges[::-1][:max_index]

        return left_ranges, right_ranges

    def lidar_callback(self, msg):
        """Process lidar data and control robot based on mode."""
        # Check if duration exceeded
        if time.time() - self.start_time > self.duration:
            self.motor.stop()
            self.get_logger().info('Duration complete, stopping')
            sys.exit(0)  # Force clean exit (rclpy.shutdown from callback hangs)

        left_range, right_range = self.get_left_right_ranges(
            msg.ranges, msg.angle_increment)

        if self.mode == 1:
            self.mode_obstacle_avoidance(left_range, right_range, msg.angle_increment)
        elif self.mode == 2:
            self.mode_tracking(left_range, right_range, msg.angle_increment)
        elif self.mode == 3:
            self.mode_guard(left_range, right_range, msg.angle_increment)

    def mode_obstacle_avoidance(self, left_range, right_range, angle_increment):
        """Mode 1: Obstacle avoidance."""
        if time.time() < self.timestamp:
            return

        # Filter valid readings
        left_valid = left_range[np.isfinite(left_range) & (left_range > 0)]
        right_valid = right_range[np.isfinite(right_range) & (right_range > 0)]

        if len(left_valid) < 2 or len(right_valid) < 2:
            self.motor.move(SPEED, 0)
            return

        min_dist_left = left_valid.min()
        min_dist_right = right_valid.min()

        if min_dist_left <= THRESHOLD and min_dist_right > THRESHOLD:
            # Obstacle on left - turn right
            vx = SPEED / 6
            wz = -SPEED * 6.0
            if self.last_act != 0 and self.last_act != 1:
                wz = SPEED * 6.0
            self.last_act = 1
            self.motor.move(vx, wz)
            self.timestamp = time.time() + (math.radians(90) / abs(wz) / 2)
            self.get_logger().info(f'L:{{min_dist_left:.2f}}m R:{{min_dist_right:.2f}}m -> Turn RIGHT')

        elif min_dist_left <= THRESHOLD and min_dist_right <= THRESHOLD:
            # Both sides blocked - turn around
            vx = SPEED / 6
            wz = SPEED * 6.0
            self.last_act = 3
            self.motor.move(vx, wz)
            self.timestamp = time.time() + (math.radians(180) / abs(wz) / 2)
            self.get_logger().info(f'L:{{min_dist_left:.2f}}m R:{{min_dist_right:.2f}}m -> Turn AROUND')

        elif min_dist_left > THRESHOLD and min_dist_right <= THRESHOLD:
            # Obstacle on right - turn left
            vx = SPEED / 6
            wz = SPEED * 6.0
            if self.last_act != 0 and self.last_act != 2:
                wz = -SPEED * 6.0
            self.last_act = 2
            self.motor.move(vx, wz)
            self.timestamp = time.time() + (math.radians(90) / abs(wz) / 2)
            self.get_logger().info(f'L:{{min_dist_left:.2f}}m R:{{min_dist_right:.2f}}m -> Turn LEFT')

        else:
            # Clear - go forward
            self.last_act = 0
            self.motor.move(SPEED, 0)
            self.get_logger().info(f'L:{{min_dist_left:.2f}}m R:{{min_dist_right:.2f}}m -> FORWARD')

    def mode_tracking(self, left_range, right_range, angle_increment):
        """Mode 2: Tracking - follow closest object."""
        # Merge ranges
        ranges = np.concatenate([right_range[::-1], left_range])
        valid_mask = np.isfinite(ranges) & (ranges > 0.1)

        if not np.any(valid_mask):
            self.motor.stop()
            self.get_logger().info('No target')
            return

        dist = ranges[valid_mask].min()
        min_index = np.argmin(np.where(valid_mask, ranges, float('inf')))
        scan_angle = math.radians(MAX_SCAN_ANGLE / 2)
        angle = -scan_angle + angle_increment * min_index

        vx = 0.0
        wz = 0.0

        # Distance control
        if dist < THRESHOLD and abs(TRACKING_DISTANCE - dist) > 0.02:
            self.pid_dist.SetPoint = TRACKING_DISTANCE
            self.pid_dist.update(dist)
            vx = clamp(-self.pid_dist.output, -SPEED, SPEED)
        else:
            self.pid_dist.clear()

        # Yaw control
        if dist < THRESHOLD and abs(math.degrees(angle)) > 5:
            self.pid_yaw.update(-angle)
            wz = clamp(self.pid_yaw.output, -SPEED * 6.0, SPEED * 6.0)
        else:
            self.pid_yaw.clear()

        if abs(wz) < 0.008:
            wz = 0.0
        if abs(vx) < 0.05:
            vx = 0.0

        self.motor.move(vx, wz)
        self.get_logger().info(f'Target: {{dist:.2f}}m at {{math.degrees(angle):.0f}}° -> vx={{vx:.2f}} wz={{wz:.2f}}')

    def mode_guard(self, left_range, right_range, angle_increment):
        """Mode 3: Guard - rotate to face closest object."""
        ranges = np.concatenate([right_range[::-1], left_range])
        valid_mask = np.isfinite(ranges) & (ranges > 0.1)

        if not np.any(valid_mask):
            self.motor.stop()
            self.get_logger().info('No object')
            return

        dist = ranges[valid_mask].min()
        min_index = np.argmin(np.where(valid_mask, ranges, float('inf')))
        scan_angle = math.radians(MAX_SCAN_ANGLE / 2)
        angle = -scan_angle + angle_increment * min_index

        wz = 0.0
        if dist < THRESHOLD and abs(math.degrees(angle)) > 5:
            self.pid_yaw.update(-angle)
            wz = clamp(self.pid_yaw.output, -SPEED * 6.0, SPEED * 6.0)
        else:
            self.pid_yaw.clear()

        if abs(wz) < 0.008:
            wz = 0.0

        self.motor.move(0, wz)
        if abs(wz) > 0.01:
            self.get_logger().info(f'Object: {{dist:.2f}}m at {{math.degrees(angle):.0f}}° -> wz={{wz:.2f}}')
        else:
            self.get_logger().info(f'Facing object at {{dist:.2f}}m')

    def destroy_node(self):
        self.motor.stop()
        super().destroy_node()


def main():
    mode = {mode}
    duration = {duration}

    print(f"ROS2 Lidar Mode Controller")
    print(f"Mode: {{mode}}, Duration: {{duration}}s")
    print(f"Parameters: speed={{SPEED}}, threshold={{THRESHOLD}}m")

    rclpy.init()
    node = LidarModeController(mode, duration)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\\nInterrupted")
    finally:
        node.motor.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("Stopped")

if __name__ == "__main__":
    main()
'''

        try:
            # Upload script to robot via echo (avoids StringIO size mismatch issues)
            # First write to local temp file, then upload
            import tempfile
            import os
            with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
                f.write(mode_script)
                temp_path = f.name

            try:
                self.conn.put(temp_path, "/tmp/lidar_mode_controller.py")
            finally:
                os.unlink(temp_path)

            # Step 1: Check lidar device exists (this updates self.lidar_device)
            if not self.check_lidar_device():
                self.console.print("[red]Lidar device not found[/red]")
                return False

            # Step 2: Ensure lidar driver is running
            self.console.print("[dim]Checking lidar driver...[/dim]")
            check_result = self.conn.run("docker ps --filter name=lidar_driver -q", hide=True, warn=True)

            if not check_result.stdout.strip():
                self.console.print("[dim]Starting lidar driver...[/dim]")
                if not self.start_lidar_driver():
                    self.console.print("[red]Failed to start lidar driver[/red]")
                    return False
                time.sleep(2)  # Wait for driver to initialize

            # Step 3: Run ROS2 mode controller in Docker
            self.console.print(f"[bold]Running ROS2 lidar mode controller (mode {mode})...[/bold]")

            # Docker command with:
            # - Network host for ROS2 communication
            # - Mount motor SDK
            # - Mount serial devices
            # - Mount the script
            docker_cmd = (
                f"docker run --rm --privileged --network host "
                f"-v /dev:/dev "
                f"-v /home/{self.user}/ros_robot_controller:/ros_robot_controller "
                f"-v /tmp/lidar_mode_controller.py:/tmp/lidar_mode_controller.py "
                f"--name lidar_mode_controller "
                f"{self.docker_image} "
                f"bash -c '"
                f"pip3 install -q pyserial numpy 2>/dev/null; "
                f"source /opt/ros/humble/setup.bash && "
                f"python3 /tmp/lidar_mode_controller.py"
                f"'"
            )

            result = self.conn.run(
                docker_cmd,
                hide=False, warn=True, timeout=int(duration) + 60
            )

            if result.ok:
                self.console.print(f"[green]Mode {mode} test complete[/green]")
                return True
            else:
                self.console.print(f"[red]Mode test failed[/red]")
                return False

        except Exception as e:
            self.console.print(f"[red]Mode test error:[/red] {e}")
            # Emergency stop - stop containers and motors
            try:
                self.conn.run("docker stop lidar_mode_controller 2>/dev/null", hide=True, warn=True, timeout=5)
            except Exception:
                pass
            return False

    def stop_lidar_app(self) -> bool:
        """Stop any running lidar mode controller."""
        try:
            # Stop the mode controller container
            self.conn.run("docker stop lidar_mode_controller 2>/dev/null", hide=True, warn=True, timeout=10)
            self.console.print("[green]Lidar mode controller stopped[/green]")
            return True
        except Exception:
            return False


@app.command()
def check(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """
    Check lidar connectivity, Docker, and ROS2 availability.
    """
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required. Provide via options or config.json[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]Lidar System Check[/bold]\n\n"
        f"Host: {host}\n"
        f"Expected Device: {LIDAR_DEVICE}\n"
        f"Baud Rate: {LIDAR_BAUD}\n"
        f"ROS2 via: Docker",
        title="Lidar Check",
        border_style="blue"
    ))

    tester = LidarTest(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    checks = []

    # Check Docker first (required for ROS2)
    docker_ok = tester.check_docker_available()
    checks.append(("Docker", docker_ok))

    # Check device
    device_ok = tester.check_lidar_device()
    checks.append(("Lidar Device", device_ok))

    # Get lidar type
    tester.get_lidar_type()

    # Check ROS2 via Docker
    if docker_ok:
        ros2_ok = tester.check_ros2_available()
        checks.append(("ROS2 in Docker", ros2_ok))

        # Check scan topic (only if a driver is running)
        if ros2_ok:
            topic_ok = tester.check_scan_topic()
            checks.append(("Scan Topic", topic_ok))

    # Summary
    console.print("\n[bold]Summary:[/bold]")
    all_ok = True
    warnings = 0
    for name, ok in checks:
        if ok:
            status = "[green]PASS[/green]"
        elif name == "Scan Topic":
            status = "[yellow]WARN[/yellow]"
            warnings += 1
        else:
            status = "[red]FAIL[/red]"
            all_ok = False
        console.print(f"  {name}: {status}")

    if all_ok and warnings > 0:
        console.print("\n[yellow]Note: Scan topic not found - start lidar driver with 'start-driver' command[/yellow]")
        sys.exit(0)

    sys.exit(0 if all_ok else 1)


@app.command()
def scan(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    samples: int = typer.Option(5, help="Number of scan samples to read"),
):
    """
    Read lidar scan data and display statistics.
    """
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required. Provide via options or config.json[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]Lidar Scan Test[/bold]\n\n"
        f"Host: {host}\n"
        f"Topic: {SCAN_TOPIC}\n"
        f"Samples: {samples}",
        title="Scan Test",
        border_style="blue"
    ))

    tester = LidarTest(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    if not tester.check_lidar_device():
        console.print("[red]Lidar device not found[/red]")
        sys.exit(1)

    if not tester.read_scan_data(samples):
        console.print("[red]Failed to read scan data[/red]")
        sys.exit(1)

    console.print("\n[bold green]Lidar scan test complete![/bold green]")


@app.command()
def test(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    mode: int = typer.Option(1, help="Mode: 1=obstacle avoidance, 2=tracking, 3=guard"),
    duration: float = typer.Option(10.0, help="Duration in seconds"),
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval prompt"),
):
    """
    Test lidar functionality mode (robot will MOVE!).

    Modes:
        1: Obstacle avoidance - robot avoids obstacles in front
        2: Tracking - robot follows objects at 35cm distance
        3: Guard - robot rotates to face detected objects
    """
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required. Provide via options or config.json[/red]")
        sys.exit(1)

    if mode not in [1, 2, 3]:
        console.print("[red]Error: mode must be 1, 2, or 3[/red]")
        sys.exit(1)

    mode_names = {1: "Obstacle Avoidance", 2: "Tracking", 3: "Guard"}

    console.print(Panel.fit(
        f"[bold]Lidar Mode Test[/bold]\n\n"
        f"Host: {host}\n"
        f"Mode: {mode} ({mode_names[mode]})\n"
        f"Duration: {duration}s\n\n"
        f"[bold red]WARNING: Robot will MOVE![/bold red]",
        title="Mode Test",
        border_style="yellow"
    ))

    tester = LidarTest(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    # Check Docker and select correct image (landerpi-ros2 has pip3, ros:humble-ros-base doesn't)
    if not tester.check_docker_available():
        console.print("[red]Docker not available on robot[/red]")
        sys.exit(1)

    # Request approval
    if not skip_approval:
        console.print("\n[bold red]WARNING: The robot is about to MOVE![/bold red]")
        console.print("[yellow]Ensure the area around the robot is clear.[/yellow]")

        if not Confirm.ask("\n[bold]Do you approve the robot to move?[/bold]", default=False):
            console.print("[blue]Test cancelled[/blue]")
            sys.exit(0)

    console.print("\n[bold green]Motion approved. Starting test...[/bold green]\n")

    try:
        if tester.test_lidar_mode(mode, duration):
            console.print("\n[bold green]Lidar mode test complete![/bold green]")
        else:
            console.print("\n[bold red]Lidar mode test failed[/bold red]")
            sys.exit(1)
    except KeyboardInterrupt:
        console.print("\n[red]Interrupted! Stopping...[/red]")
        tester.stop_lidar_app()
        sys.exit(1)


@app.command()
def stop(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """
    Stop any running lidar functionality.
    """
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print("[bold red]Sending STOP command...[/bold red]")
    tester = LidarTest(host, user, password)

    if tester.verify_connection():
        tester.check_docker_available()
        # Stop both driver and app
        tester.stop_lidar_driver()
        tester.stop_lidar_app()
        console.print("[green]Stop commands sent[/green]")


@app.command("start-driver")
def start_driver(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    lidar_type: str = typer.Option("MS200", help="Lidar type: MS200 or LD19"),
):
    """
    Start the lidar driver in a Docker container.
    This publishes scan data to the /scan topic.
    """
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]Start Lidar Driver[/bold]\n\n"
        f"Host: {host}\n"
        f"Lidar Type: {lidar_type}\n"
        f"Running in: Docker container",
        title="Start Driver",
        border_style="green"
    ))

    tester = LidarTest(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    if not tester.check_docker_available():
        console.print("[red]Docker is required to run the lidar driver[/red]")
        sys.exit(1)

    if not tester.check_lidar_device():
        console.print("[red]Lidar device not found[/red]")
        sys.exit(1)

    if tester.start_lidar_driver(lidar_type):
        console.print("\n[bold green]Lidar driver started![/bold green]")
        console.print("[dim]Use 'scan' command to read data, 'stop-driver' to stop[/dim]")
    else:
        console.print("\n[bold red]Failed to start lidar driver[/bold red]")
        sys.exit(1)


@app.command("stop-driver")
def stop_driver(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """
    Stop the lidar driver Docker container.
    """
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print("[bold yellow]Stopping lidar driver...[/bold yellow]")
    tester = LidarTest(host, user, password)

    if tester.verify_connection():
        tester.stop_lidar_driver()


if __name__ == "__main__":
    app()
