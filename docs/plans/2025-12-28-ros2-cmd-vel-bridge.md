# ROS2 cmd_vel Bridge Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Create a persistent ROS2 cmd_vel bridge that translates standard Twist messages to motor commands, deployed via Docker Compose with auto-restart on reboot.

**Architecture:** Separate `cmd_vel_bridge` node subscribes to `/cmd_vel`, performs mecanum kinematics, publishes to `/ros_robot_controller/set_motor`. Both nodes run in a single Docker container managed by Docker Compose with `restart: unless-stopped`.

**Tech Stack:** ROS2 Humble, Python, Docker Compose, rclpy, geometry_msgs, ros_robot_controller_msgs

---

## Task 1: Create Directory Structure

**Files:**
- Create: `ros2_nodes/cmd_vel_bridge/`
- Create: `docker/`
- Create: `config/`

**Step 1: Create local directories**

```bash
mkdir -p ros2_nodes/cmd_vel_bridge/cmd_vel_bridge
mkdir -p docker
mkdir -p config
```

**Step 2: Verify structure**

Run: `ls -la ros2_nodes/ docker/ config/`
Expected: Empty directories exist

**Step 3: Commit**

```bash
git add ros2_nodes/.gitkeep docker/.gitkeep config/.gitkeep 2>/dev/null || true
```

Note: We'll commit with actual files in next tasks.

---

## Task 2: Create Robot Parameters Config

**Files:**
- Create: `config/robot_params.yaml`

**Step 1: Write config file**

```yaml
# LanderPi Robot Parameters
# Mecanum wheel geometry - adjust after calibration

cmd_vel_bridge:
  ros__parameters:
    # Wheel geometry (meters)
    wheel_radius: 0.05        # 50mm - measure actual wheel
    wheel_base: 0.15          # 150mm - front to back axle distance
    track_width: 0.15         # 150mm - left to right wheel distance

    # Motor configuration
    motor_ids:
      front_left: 1
      back_left: 2
      front_right: 3
      back_right: 4

    # Safety limits
    max_wheel_rps: 3.0        # Max rotations per second
    cmd_vel_timeout: 0.5      # Stop if no command for 500ms

    # Topic names
    cmd_vel_topic: "/cmd_vel"
    motor_topic: "/ros_robot_controller/set_motor"
```

**Step 2: Verify YAML syntax**

Run: `python3 -c "import yaml; yaml.safe_load(open('config/robot_params.yaml'))"`
Expected: No output (valid YAML)

**Step 3: Commit**

```bash
git add config/robot_params.yaml
git commit -m "feat: add robot parameters config for cmd_vel bridge"
```

---

## Task 3: Create ROS2 Package Structure

**Files:**
- Create: `ros2_nodes/cmd_vel_bridge/package.xml`
- Create: `ros2_nodes/cmd_vel_bridge/setup.py`
- Create: `ros2_nodes/cmd_vel_bridge/setup.cfg`
- Create: `ros2_nodes/cmd_vel_bridge/resource/cmd_vel_bridge`
- Create: `ros2_nodes/cmd_vel_bridge/cmd_vel_bridge/__init__.py`

**Step 1: Write package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>cmd_vel_bridge</name>
  <version>0.1.0</version>
  <description>Bridge from cmd_vel (Twist) to ros_robot_controller motor commands for LanderPi mecanum robot</description>
  <maintainer email="user@example.com">LanderPi</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>ros_robot_controller_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Step 2: Write setup.py**

```python
from setuptools import setup

package_name = 'cmd_vel_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LanderPi',
    maintainer_email='user@example.com',
    description='cmd_vel to motor bridge for LanderPi mecanum robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = cmd_vel_bridge.cmd_vel_bridge_node:main',
        ],
    },
)
```

**Step 3: Write setup.cfg**

```ini
[develop]
script_dir=$base/lib/cmd_vel_bridge
[install]
install_scripts=$base/lib/cmd_vel_bridge
```

**Step 4: Create resource file and __init__.py**

```bash
mkdir -p ros2_nodes/cmd_vel_bridge/resource
touch ros2_nodes/cmd_vel_bridge/resource/cmd_vel_bridge
touch ros2_nodes/cmd_vel_bridge/cmd_vel_bridge/__init__.py
```

**Step 5: Commit**

```bash
git add ros2_nodes/cmd_vel_bridge/
git commit -m "feat: add cmd_vel_bridge ROS2 package structure"
```

---

## Task 4: Implement Mecanum Kinematics Module

**Files:**
- Create: `ros2_nodes/cmd_vel_bridge/cmd_vel_bridge/mecanum_kinematics.py`

**Step 1: Write kinematics module**

```python
"""Mecanum wheel kinematics for 4-wheel robot."""

from dataclasses import dataclass
from typing import Tuple


@dataclass
class WheelSpeeds:
    """Wheel speeds in rotations per second (RPS)."""
    front_left: float
    back_left: float
    front_right: float
    back_right: float


@dataclass
class RobotGeometry:
    """Robot physical dimensions."""
    wheel_radius: float  # meters
    wheel_base: float    # front-to-back distance (meters)
    track_width: float   # left-to-right distance (meters)

    @property
    def lx(self) -> float:
        """Half of wheel base."""
        return self.wheel_base / 2.0

    @property
    def ly(self) -> float:
        """Half of track width."""
        return self.track_width / 2.0


def twist_to_wheel_speeds(
    vx: float,
    vy: float,
    wz: float,
    geometry: RobotGeometry,
    max_rps: float = 3.0
) -> WheelSpeeds:
    """
    Convert Twist velocities to individual wheel speeds.

    Args:
        vx: Linear velocity X (forward/backward, m/s)
        vy: Linear velocity Y (strafe left/right, m/s)
        wz: Angular velocity Z (rotation, rad/s)
        geometry: Robot physical dimensions
        max_rps: Maximum wheel rotations per second (safety limit)

    Returns:
        WheelSpeeds with RPS for each wheel

    Mecanum kinematics (standard configuration with rollers at 45 degrees):
        FL = (vx - vy - (lx + ly) * wz) / r
        BL = (vx + vy - (lx + ly) * wz) / r
        FR = (vx + vy + (lx + ly) * wz) / r
        BR = (vx - vy + (lx + ly) * wz) / r
    """
    r = geometry.wheel_radius
    k = geometry.lx + geometry.ly  # Combined geometry factor

    # Calculate wheel angular velocities (rad/s)
    fl_rad = (vx - vy - k * wz) / r
    bl_rad = (vx + vy - k * wz) / r
    fr_rad = (vx + vy + k * wz) / r
    br_rad = (vx - vy + k * wz) / r

    # Convert to RPS (rad/s -> rev/s)
    fl_rps = fl_rad / (2.0 * 3.14159265359)
    bl_rps = bl_rad / (2.0 * 3.14159265359)
    fr_rps = fr_rad / (2.0 * 3.14159265359)
    br_rps = br_rad / (2.0 * 3.14159265359)

    # Clamp to max RPS
    def clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    return WheelSpeeds(
        front_left=clamp(fl_rps, max_rps),
        back_left=clamp(bl_rps, max_rps),
        front_right=clamp(fr_rps, max_rps),
        back_right=clamp(br_rps, max_rps),
    )
```

**Step 2: Test kinematics locally (quick sanity check)**

Run: `python3 -c "
from ros2_nodes.cmd_vel_bridge.cmd_vel_bridge.mecanum_kinematics import *
g = RobotGeometry(0.05, 0.15, 0.15)
# Forward motion: all wheels same direction
s = twist_to_wheel_speeds(0.1, 0, 0, g)
print(f'Forward: FL={s.front_left:.2f} BL={s.back_left:.2f} FR={s.front_right:.2f} BR={s.back_right:.2f}')
# All should be positive and equal for forward
assert abs(s.front_left - s.back_left) < 0.01
assert abs(s.front_right - s.back_right) < 0.01
print('Sanity check passed')
"`

Expected: Forward motion shows equal wheel speeds, no assertion errors

**Step 3: Commit**

```bash
git add ros2_nodes/cmd_vel_bridge/cmd_vel_bridge/mecanum_kinematics.py
git commit -m "feat: add mecanum kinematics module"
```

---

## Task 5: Implement cmd_vel Bridge Node

**Files:**
- Create: `ros2_nodes/cmd_vel_bridge/cmd_vel_bridge/cmd_vel_bridge_node.py`

**Step 1: Write the bridge node**

```python
#!/usr/bin/env python3
"""
ROS2 node that bridges /cmd_vel (Twist) to motor commands.

Subscribes to: /cmd_vel (geometry_msgs/Twist)
Publishes to: /ros_robot_controller/set_motor (ros_robot_controller_msgs/MotorsState)
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import MotorsState, MotorState

from .mecanum_kinematics import twist_to_wheel_speeds, RobotGeometry, WheelSpeeds


class CmdVelBridge(Node):
    """Bridge node from cmd_vel to motor commands."""

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # Declare parameters with defaults
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.15)
        self.declare_parameter('track_width', 0.15)
        self.declare_parameter('max_wheel_rps', 3.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('motor_ids.front_left', 1)
        self.declare_parameter('motor_ids.back_left', 2)
        self.declare_parameter('motor_ids.front_right', 3)
        self.declare_parameter('motor_ids.back_right', 4)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('motor_topic', '/ros_robot_controller/set_motor')

        # Get parameters
        self.geometry = RobotGeometry(
            wheel_radius=self.get_parameter('wheel_radius').value,
            wheel_base=self.get_parameter('wheel_base').value,
            track_width=self.get_parameter('track_width').value,
        )
        self.max_rps = self.get_parameter('max_wheel_rps').value
        self.timeout = self.get_parameter('cmd_vel_timeout').value

        self.motor_ids = {
            'front_left': self.get_parameter('motor_ids.front_left').value,
            'back_left': self.get_parameter('motor_ids.back_left').value,
            'front_right': self.get_parameter('motor_ids.front_right').value,
            'back_right': self.get_parameter('motor_ids.back_right').value,
        }

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        motor_topic = self.get_parameter('motor_topic').value

        # Last command timestamp for watchdog
        self.last_cmd_time = self.get_clock().now()

        # Publisher
        self.motor_pub = self.create_publisher(MotorsState, motor_topic, 10)

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Watchdog timer (10Hz check)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info(
            f'cmd_vel_bridge started: {cmd_vel_topic} -> {motor_topic}'
        )
        self.get_logger().info(
            f'Geometry: radius={self.geometry.wheel_radius}m, '
            f'base={self.geometry.wheel_base}m, track={self.geometry.track_width}m'
        )

    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming cmd_vel message."""
        self.last_cmd_time = self.get_clock().now()

        # Calculate wheel speeds
        speeds = twist_to_wheel_speeds(
            vx=msg.linear.x,
            vy=msg.linear.y,
            wz=msg.angular.z,
            geometry=self.geometry,
            max_rps=self.max_rps
        )

        # Publish motor commands
        self.publish_motor_speeds(speeds)

    def publish_motor_speeds(self, speeds: WheelSpeeds):
        """Publish wheel speeds to motor controller."""
        msg = MotorsState()
        msg.data = [
            self._motor_state(self.motor_ids['front_left'], speeds.front_left),
            self._motor_state(self.motor_ids['back_left'], speeds.back_left),
            self._motor_state(self.motor_ids['front_right'], speeds.front_right),
            self._motor_state(self.motor_ids['back_right'], speeds.back_right),
        ]
        self.motor_pub.publish(msg)

    def _motor_state(self, motor_id: int, rps: float) -> MotorState:
        """Create a MotorState message."""
        state = MotorState()
        state.id = motor_id
        state.rps = rps
        return state

    def watchdog_callback(self):
        """Stop motors if no cmd_vel received within timeout."""
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > self.timeout:
            # Send zero velocities
            self.publish_motor_speeds(WheelSpeeds(0.0, 0.0, 0.0, 0.0))

    def stop_motors(self):
        """Emergency stop - zero all motors."""
        self.publish_motor_speeds(WheelSpeeds(0.0, 0.0, 0.0, 0.0))
        self.get_logger().info('Motors stopped')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Verify syntax**

Run: `python3 -m py_compile ros2_nodes/cmd_vel_bridge/cmd_vel_bridge/cmd_vel_bridge_node.py`
Expected: No output (valid syntax)

**Step 3: Commit**

```bash
git add ros2_nodes/cmd_vel_bridge/cmd_vel_bridge/cmd_vel_bridge_node.py
git commit -m "feat: add cmd_vel_bridge ROS2 node with watchdog"
```

---

## Task 6: Create Launch File

**Files:**
- Create: `ros2_nodes/cmd_vel_bridge/launch/`
- Create: `ros2_nodes/cmd_vel_bridge/launch/landerpi.launch.py`
- Modify: `ros2_nodes/cmd_vel_bridge/setup.py` (add launch files to data_files)

**Step 1: Create launch directory**

```bash
mkdir -p ros2_nodes/cmd_vel_bridge/launch
```

**Step 2: Write launch file**

```python
#!/usr/bin/env python3
"""Launch file for LanderPi ROS2 stack."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Config file path (mounted from host)
    config_file = '/ros2_ws/config/robot_params.yaml'

    return LaunchDescription([
        # Motor controller node (from ros_robot_controller package)
        Node(
            package='ros_robot_controller',
            executable='ros_robot_controller',
            name='ros_robot_controller',
            output='screen',
            parameters=[{'imu_frame': 'imu_link'}],
        ),

        # cmd_vel bridge node
        Node(
            package='cmd_vel_bridge',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[config_file],
        ),
    ])
```

**Step 3: Update setup.py to include launch files**

Replace setup.py with:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'cmd_vel_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LanderPi',
    maintainer_email='user@example.com',
    description='cmd_vel to motor bridge for LanderPi mecanum robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = cmd_vel_bridge.cmd_vel_bridge_node:main',
        ],
    },
)
```

**Step 4: Commit**

```bash
git add ros2_nodes/cmd_vel_bridge/launch/ ros2_nodes/cmd_vel_bridge/setup.py
git commit -m "feat: add landerpi launch file"
```

---

## Task 7: Create Docker Compose Configuration

**Files:**
- Create: `docker/docker-compose.yml`

**Step 1: Write docker-compose.yml**

```yaml
# LanderPi ROS2 Stack
# Persists across reboots with restart: unless-stopped

services:
  ros2-stack:
    image: landerpi-ros2:latest
    container_name: landerpi-ros2
    restart: unless-stopped
    network_mode: host
    privileged: true

    devices:
      - /dev/ttyAMA0:/dev/ttyAMA0

    volumes:
      # Custom nodes (read-only)
      - ../ros2_nodes/cmd_vel_bridge:/ros2_ws/src/cmd_vel_bridge:ro
      # Vendor drivers (read-only) - never modified
      - ../drivers/ros_robot_controller-ros2/src/ros_robot_controller:/ros2_ws/src/ros_robot_controller:ro
      - ../drivers/ros_robot_controller-ros2/src/ros_robot_controller_msgs:/ros2_ws/src/ros_robot_controller_msgs:ro
      # Config (read-only)
      - ../config:/ros2_ws/config:ro
      # Build artifacts (persistent)
      - ros2_build:/ros2_ws/build
      - ros2_install:/ros2_ws/install
      - ros2_log:/ros2_ws/log

    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp

    command: >
      bash -c "
        set -e
        source /opt/ros/humble/setup.bash
        cd /ros2_ws

        echo '=== Building ROS2 packages ==='
        colcon build --symlink-install --packages-select ros_robot_controller_msgs ros_robot_controller cmd_vel_bridge

        source install/setup.bash

        echo '=== Starting LanderPi ROS2 stack ==='
        ros2 launch cmd_vel_bridge landerpi.launch.py
      "

volumes:
  ros2_build:
  ros2_install:
  ros2_log:
```

**Step 2: Verify YAML syntax**

Run: `python3 -c "import yaml; yaml.safe_load(open('docker/docker-compose.yml'))"`
Expected: No output (valid YAML)

**Step 3: Commit**

```bash
git add docker/docker-compose.yml
git commit -m "feat: add Docker Compose for persistent ROS2 stack"
```

---

## Task 8: Create Deployment Script

**Files:**
- Create: `deploy_ros2_stack.py`

**Step 1: Write deployment script**

```python
#!/usr/bin/env python3
"""Deploy ROS2 stack to LanderPi robot."""

import json
import typer
from pathlib import Path
from fabric import Connection
from rich.console import Console
from rich.panel import Panel

app = typer.Typer()
console = Console()


def get_config():
    """Load connection config from config.json."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        with open(config_path) as f:
            return json.load(f)
    return {}


def upload_directory(conn: Connection, local_path: Path, remote_path: str):
    """Upload a directory recursively."""
    conn.run(f"mkdir -p {remote_path}")
    for item in local_path.rglob("*"):
        if item.is_file() and "__pycache__" not in str(item):
            rel_path = item.relative_to(local_path)
            remote_file = f"{remote_path}/{rel_path}"
            remote_dir = str(Path(remote_file).parent)
            conn.run(f"mkdir -p {remote_dir}")
            conn.put(str(item), remote_file)
            console.print(f"  [dim]{rel_path}[/dim]")


@app.command()
def deploy(
    host: str = typer.Option(None, help="Robot IP address"),
    user: str = typer.Option(None, help="SSH username"),
    password: str = typer.Option(None, help="SSH password"),
    start: bool = typer.Option(True, help="Start the stack after deployment"),
):
    """Deploy ROS2 stack to LanderPi robot."""

    # Load defaults from config.json
    config = get_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not all([host, user, password]):
        console.print("[red]Missing connection details. Use --host, --user, --password or config.json[/red]")
        raise typer.Exit(1)

    console.print(Panel(f"Deploying ROS2 stack to {user}@{host}", title="LanderPi Deploy"))

    base_path = Path(__file__).parent

    with Connection(host, user=user, connect_kwargs={"password": password}) as conn:
        # Create remote directories
        console.print("\n[bold]Creating directories...[/bold]")
        conn.run("mkdir -p ~/landerpi/{ros2_nodes,docker,config,drivers}")

        # Upload ros2_nodes
        console.print("\n[bold]Uploading ros2_nodes/...[/bold]")
        upload_directory(conn, base_path / "ros2_nodes", "~/landerpi/ros2_nodes")

        # Upload docker
        console.print("\n[bold]Uploading docker/...[/bold]")
        upload_directory(conn, base_path / "docker", "~/landerpi/docker")

        # Upload config
        console.print("\n[bold]Uploading config/...[/bold]")
        upload_directory(conn, base_path / "config", "~/landerpi/config")

        # Upload drivers (read-only reference)
        console.print("\n[bold]Uploading drivers/ros_robot_controller-ros2/...[/bold]")
        upload_directory(
            conn,
            base_path / "drivers" / "ros_robot_controller-ros2",
            "~/landerpi/drivers/ros_robot_controller-ros2"
        )

        if start:
            console.print("\n[bold]Starting ROS2 stack...[/bold]")
            result = conn.run(
                "cd ~/landerpi/docker && docker compose up -d --build",
                warn=True
            )
            if result.ok:
                console.print("[green]ROS2 stack started successfully![/green]")
            else:
                console.print("[red]Failed to start stack. Check logs with:[/red]")
                console.print("  docker logs landerpi-ros2")

        # Verify
        console.print("\n[bold]Verifying deployment...[/bold]")
        result = conn.run("docker ps --filter name=landerpi-ros2 --format '{{.Status}}'", warn=True)
        if "Up" in result.stdout:
            console.print("[green]Container is running![/green]")
            console.print("\nTest with:")
            console.print("  uv run python test_chassis_motion.py test")
        else:
            console.print("[yellow]Container not running yet. Check logs.[/yellow]")


@app.command()
def stop(
    host: str = typer.Option(None),
    user: str = typer.Option(None),
    password: str = typer.Option(None),
):
    """Stop the ROS2 stack on robot."""
    config = get_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    with Connection(host, user=user, connect_kwargs={"password": password}) as conn:
        conn.run("cd ~/landerpi/docker && docker compose down")
        console.print("[green]ROS2 stack stopped[/green]")


@app.command()
def logs(
    host: str = typer.Option(None),
    user: str = typer.Option(None),
    password: str = typer.Option(None),
    follow: bool = typer.Option(False, "-f", help="Follow log output"),
):
    """View ROS2 stack logs."""
    config = get_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    with Connection(host, user=user, connect_kwargs={"password": password}) as conn:
        cmd = "docker logs landerpi-ros2"
        if follow:
            cmd += " -f"
        conn.run(cmd)


if __name__ == "__main__":
    app()
```

**Step 2: Test syntax**

Run: `python3 -m py_compile deploy_ros2_stack.py`
Expected: No output (valid syntax)

**Step 3: Commit**

```bash
git add deploy_ros2_stack.py
git commit -m "feat: add deployment script for ROS2 stack"
```

---

## Task 9: Deploy to Robot

**Step 1: Run deployment**

Run: `uv run python deploy_ros2_stack.py deploy`
Expected: Files uploaded, container started

**Step 2: Verify container is running**

Run: `uv run python deploy_ros2_stack.py logs`
Expected: Shows ROS2 node startup logs

**Step 3: Check topics on robot (via SSH)**

Run SSH command:
```bash
docker exec landerpi-ros2 bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic list"
```

Expected output should include:
- `/cmd_vel`
- `/ros_robot_controller/set_motor`
- `/ros_robot_controller/imu_raw`

---

## Task 10: Test cmd_vel Motion

**Step 1: Run test_chassis_motion.py**

Run: `uv run python test_chassis_motion.py test --host 192.168.50.169 --user tonbost --password ".corleonne."`

Expected: Robot moves forward and backward

**Step 2: Test persistence - reboot robot**

Run SSH command: `sudo reboot`

Wait 60 seconds, then:

Run: `uv run python deploy_ros2_stack.py logs`
Expected: Container auto-started, nodes running

**Step 3: Test motion again after reboot**

Run: `uv run python test_chassis_motion.py test`
Expected: Robot moves (proves persistence works)

**Step 4: Final commit**

```bash
git add -A
git commit -m "feat: complete ROS2 cmd_vel bridge with persistence"
```

---

## Summary

After completing all tasks:

1. `/cmd_vel` topic available for standard ROS2 navigation
2. Mecanum kinematics translate Twist to wheel speeds
3. Docker Compose ensures stack survives reboots
4. `deploy_ros2_stack.py` for easy deployment
5. `test_chassis_motion.py` works with ROS2 stack
