# ROS2 Unified Stack Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Standardize all sensor testing to use the persistent ROS2 stack pattern, creating new `*_ros2.py` test scripts and integrating all drivers into the Docker Compose stack.

**Architecture:** All drivers (motion, arm, lidar, camera) run in a single Docker Compose service with `restart: unless-stopped`. Test scripts connect via SSH and use `docker exec landerpi-ros2` to publish/subscribe to ROS2 topics. This mirrors the existing `test_chassis_motion.py` pattern.

**Tech Stack:** Python 3.11+, Typer, Fabric, ROS2 Humble, Docker Compose

**Reference:** Design doc at `docs/plans/2025-12-28-ros2-unified-stack-design.md`

---

## Phase 1: Arm Controller ROS2 Node

### Task 1.1: Create arm_controller package structure

**Files:**
- Create: `ros2_nodes/arm_controller/package.xml`
- Create: `ros2_nodes/arm_controller/setup.py`
- Create: `ros2_nodes/arm_controller/setup.cfg`
- Create: `ros2_nodes/arm_controller/resource/arm_controller`
- Create: `ros2_nodes/arm_controller/arm_controller/__init__.py`

**Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>arm_controller</name>
  <version>0.1.0</version>
  <description>ROS2 arm controller for LanderPi 5-DOF arm</description>
  <maintainer email="robot@landerpi.local">LanderPi</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Step 2: Create setup.py**

```python
from setuptools import find_packages, setup

package_name = 'arm_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LanderPi',
    maintainer_email='robot@landerpi.local',
    description='ROS2 arm controller for LanderPi 5-DOF arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller_node = arm_controller.arm_controller_node:main',
        ],
    },
)
```

**Step 3: Create setup.cfg**

```ini
[develop]
script_dir=$base/lib/arm_controller
[install]
install_scripts=$base/lib/arm_controller
```

**Step 4: Create resource file and __init__.py**

```bash
mkdir -p ros2_nodes/arm_controller/resource
touch ros2_nodes/arm_controller/resource/arm_controller
mkdir -p ros2_nodes/arm_controller/arm_controller
touch ros2_nodes/arm_controller/arm_controller/__init__.py
```

**Step 5: Commit**

```bash
git add ros2_nodes/arm_controller/
git commit -m "feat(arm): create arm_controller ROS2 package structure"
```

---

### Task 1.2: Create arm_controller_node.py

**Files:**
- Create: `ros2_nodes/arm_controller/arm_controller/arm_controller_node.py`

**Step 1: Create the arm controller node**

```python
#!/usr/bin/env python3
"""
ROS2 node for controlling the LanderPi 5-DOF arm.

Subscribes to: /arm/cmd (std_msgs/String) - JSON commands
Publishes to: /arm/state (std_msgs/String) - JSON state

Command format:
  {"action": "set_position", "duration": 2.0, "positions": [[1, 500], [2, 500], ...]}
  {"action": "home"}
  {"action": "stop"}
  {"action": "gripper", "position": "open"|"close"|<pulse>}
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# SDK path - mounted from host
sys.path.insert(0, '/ros2_ws/src/ros_robot_controller/ros_robot_controller')


class ArmController(Node):
    """ROS2 node for arm control via SDK."""

    # Servo configuration
    ARM_SERVO_IDS = [1, 2, 3, 4, 5]
    GRIPPER_SERVO_ID = 10
    ALL_SERVO_IDS = ARM_SERVO_IDS + [GRIPPER_SERVO_ID]
    HOME_POSITION = 500
    GRIPPER_OPEN = 200
    GRIPPER_CLOSED = 700

    def __init__(self):
        super().__init__('arm_controller')

        # Try to import SDK
        try:
            from ros_robot_controller_sdk import Board
            self.board = Board()
            self.board.enable_reception()
            self.sdk_available = True
            self.get_logger().info('SDK initialized successfully')
        except Exception as e:
            self.board = None
            self.sdk_available = False
            self.get_logger().warn(f'SDK not available: {e}')

        # Publisher for arm state
        self.state_pub = self.create_publisher(String, '/arm/state', 10)

        # Subscriber for arm commands
        self.cmd_sub = self.create_subscription(
            String,
            '/arm/cmd',
            self.cmd_callback,
            10
        )

        # State timer (publish at 1Hz)
        self.state_timer = self.create_timer(1.0, self.publish_state)

        self.get_logger().info('Arm controller started')

    def cmd_callback(self, msg: String):
        """Handle incoming arm commands."""
        try:
            cmd = json.loads(msg.data)
            action = cmd.get('action', '')

            if action == 'set_position':
                self.set_position(cmd.get('duration', 2.0), cmd.get('positions', []))
            elif action == 'home':
                self.go_home(cmd.get('duration', 2.0))
            elif action == 'stop':
                self.stop()
            elif action == 'gripper':
                self.control_gripper(cmd.get('position', 'open'))
            else:
                self.get_logger().warn(f'Unknown action: {action}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Command error: {e}')

    def set_position(self, duration: float, positions: list):
        """Set servo positions."""
        if not self.sdk_available:
            self.get_logger().warn('SDK not available')
            return

        self.get_logger().info(f'Setting positions: {positions} over {duration}s')
        self.board.bus_servo_set_position(duration, positions)

    def go_home(self, duration: float = 2.0):
        """Move all servos to home position."""
        if not self.sdk_available:
            return

        positions = [[sid, self.HOME_POSITION] for sid in self.ALL_SERVO_IDS]
        self.get_logger().info(f'Moving to home position over {duration}s')
        self.board.bus_servo_set_position(duration, positions)

    def stop(self):
        """Stop all servos."""
        if not self.sdk_available:
            return

        self.get_logger().info('Stopping all servos')
        self.board.bus_servo_stop(self.ALL_SERVO_IDS)

    def control_gripper(self, position):
        """Control gripper."""
        if not self.sdk_available:
            return

        if position == 'open':
            pulse = self.GRIPPER_OPEN
        elif position == 'close':
            pulse = self.GRIPPER_CLOSED
        else:
            pulse = int(position)

        self.get_logger().info(f'Gripper to {pulse}')
        self.board.bus_servo_set_position(0.5, [[self.GRIPPER_SERVO_ID, pulse]])

    def publish_state(self):
        """Publish current arm state."""
        state = {
            'sdk_available': self.sdk_available,
            'timestamp': time.time(),
        }

        if self.sdk_available:
            try:
                positions = {}
                for sid in self.ALL_SERVO_IDS:
                    pos = self.board.bus_servo_read_position(sid)
                    positions[sid] = pos[0] if pos else None
                state['positions'] = positions
            except Exception:
                pass

        msg = String()
        msg.data = json.dumps(state)
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Commit**

```bash
git add ros2_nodes/arm_controller/arm_controller/arm_controller_node.py
git commit -m "feat(arm): add arm_controller ROS2 node"
```

---

## Phase 2: Lidar Driver ROS2 Node

### Task 2.1: Create lidar_driver package structure

**Files:**
- Create: `ros2_nodes/lidar_driver/package.xml`
- Create: `ros2_nodes/lidar_driver/setup.py`
- Create: `ros2_nodes/lidar_driver/setup.cfg`
- Create: `ros2_nodes/lidar_driver/resource/lidar_driver`
- Create: `ros2_nodes/lidar_driver/lidar_driver/__init__.py`

**Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>lidar_driver</name>
  <version>0.1.0</version>
  <description>ROS2 driver for LD19/MS200 lidar on LanderPi</description>
  <maintainer email="robot@landerpi.local">LanderPi</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Step 2: Create setup.py**

```python
from setuptools import find_packages, setup

package_name = 'lidar_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='LanderPi',
    maintainer_email='robot@landerpi.local',
    description='ROS2 driver for LD19/MS200 lidar on LanderPi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_driver_node = lidar_driver.ld19_driver_node:main',
        ],
    },
)
```

**Step 3: Create setup.cfg and resource files**

```bash
mkdir -p ros2_nodes/lidar_driver/resource
touch ros2_nodes/lidar_driver/resource/lidar_driver
mkdir -p ros2_nodes/lidar_driver/lidar_driver
touch ros2_nodes/lidar_driver/lidar_driver/__init__.py
```

Create `ros2_nodes/lidar_driver/setup.cfg`:
```ini
[develop]
script_dir=$base/lib/lidar_driver
[install]
install_scripts=$base/lib/lidar_driver
```

**Step 4: Commit**

```bash
git add ros2_nodes/lidar_driver/
git commit -m "feat(lidar): create lidar_driver ROS2 package structure"
```

---

### Task 2.2: Create ld19_driver_node.py

**Files:**
- Create: `ros2_nodes/lidar_driver/lidar_driver/ld19_driver_node.py`

**Step 1: Create the lidar driver node**

```python
#!/usr/bin/env python3
"""
ROS2 driver for LD19/MS200 lidar.

Publishes to: /scan (sensor_msgs/LaserScan)

Reads serial data from /dev/ttyUSB0 or /dev/ttyUSB1 at 230400 baud.
Accumulates 360 degrees of scan data before publishing.
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    import serial
except ImportError:
    serial = None


class LD19Driver(Node):
    """LD19/MS200 lidar driver - accumulates full 360 scans before publishing."""

    PACKET_HEADER = 0x54
    PACKET_VER_LEN = 0x2C  # 12-point packet
    PACKET_SIZE = 47

    def __init__(self):
        super().__init__('lidar_driver')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('port_fallback', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('frame_id', 'laser_frame')

        port = self.get_parameter('port').value
        port_fallback = self.get_parameter('port_fallback').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publisher
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        # Scan accumulator (360 degrees)
        self.ranges = [float('inf')] * 360
        self.running = True

        # Try to open serial port
        if serial is None:
            self.get_logger().error('pyserial not installed')
            self.serial = None
            return

        self.serial = None
        for p in [port, port_fallback]:
            try:
                self.serial = serial.Serial(p, self.baudrate, timeout=1)
                self.get_logger().info(f'Opened {p} at {self.baudrate} baud')
                break
            except Exception as e:
                self.get_logger().warn(f'Failed to open {p}: {e}')

        if self.serial is None:
            self.get_logger().error('Could not open any serial port')
            return

        # Start reading thread
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

        # Publish timer at 10Hz
        self.timer = self.create_timer(0.1, self.publish_scan)

        self.get_logger().info('Lidar driver started')

    def read_loop(self):
        """Read and parse lidar data packets."""
        buffer = bytearray()
        while self.running and self.serial:
            try:
                data = self.serial.read(512)
                if data:
                    buffer.extend(data)
                    self.process_buffer(buffer)
            except Exception:
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
                    dist_mm = struct.unpack('<H', bytes(buffer[offset:offset + 2]))[0]

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
        msg.header.frame_id = self.frame_id
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
        if self.serial:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LD19Driver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Commit**

```bash
git add ros2_nodes/lidar_driver/lidar_driver/ld19_driver_node.py
git commit -m "feat(lidar): add LD19 lidar driver ROS2 node"
```

---

## Phase 3: Update Docker Compose and Launch File

### Task 3.1: Update docker-compose.yml

**Files:**
- Modify: `docker/docker-compose.yml`

**Step 1: Update docker-compose.yml with new nodes and mounts**

Replace contents of `docker/docker-compose.yml`:

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
      - /dev/ttyAMA0:/dev/ttyAMA0    # Motor controller
      - /dev/ttyUSB0:/dev/ttyUSB0    # Lidar (primary)
      - /dev/ttyUSB1:/dev/ttyUSB1    # Lidar (fallback)

    volumes:
      # Custom nodes (read-only)
      - ../ros2_nodes/cmd_vel_bridge:/ros2_ws/src/cmd_vel_bridge:ro
      - ../ros2_nodes/arm_controller:/ros2_ws/src/arm_controller:ro
      - ../ros2_nodes/lidar_driver:/ros2_ws/src/lidar_driver:ro
      # Vendor drivers (read-only) - never modified
      - ../drivers/ros_robot_controller-ros2/src/ros_robot_controller:/ros2_ws/src/ros_robot_controller:ro
      - ../drivers/ros_robot_controller-ros2/src/ros_robot_controller_msgs:/ros2_ws/src/ros_robot_controller_msgs:ro
      # Camera driver workspace (built on robot, optional)
      - ~/deptrum_ws:/deptrum_ws:ro
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

        echo '=== Installing pyserial for lidar driver ==='
        pip3 install -q pyserial 2>/dev/null || true

        echo '=== Building ROS2 packages ==='
        colcon build --symlink-install --packages-select \
          ros_robot_controller_msgs \
          ros_robot_controller \
          cmd_vel_bridge \
          arm_controller \
          lidar_driver

        source install/setup.bash

        # Source camera driver if available
        if [ -f /deptrum_ws/install/local_setup.bash ]; then
          echo '=== Sourcing camera driver ==='
          source /deptrum_ws/install/local_setup.bash
        fi

        echo '=== Starting LanderPi ROS2 stack ==='
        ros2 launch cmd_vel_bridge landerpi.launch.py
      "

volumes:
  ros2_build:
  ros2_install:
  ros2_log:
```

**Step 2: Commit**

```bash
git add docker/docker-compose.yml
git commit -m "feat(docker): add arm_controller and lidar_driver to compose stack"
```

---

### Task 3.2: Update launch file to start all nodes

**Files:**
- Modify: `ros2_nodes/cmd_vel_bridge/launch/landerpi.launch.py`

**Step 1: Update launch file**

Replace contents of `ros2_nodes/cmd_vel_bridge/launch/landerpi.launch.py`:

```python
"""
LanderPi unified ROS2 launch file.

Launches all LanderPi nodes:
- ros_robot_controller (vendor) - STM32 communication
- cmd_vel_bridge - Twist to motor commands
- arm_controller - Arm servo control
- lidar_driver - LD19/MS200 lidar
- camera_driver - Aurora 930 (if workspace available)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable lidar driver'
    )

    enable_arm_arg = DeclareLaunchArgument(
        'enable_arm',
        default_value='true',
        description='Enable arm controller'
    )

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='Enable camera driver (requires deptrum_ws)'
    )

    # Vendor node: ros_robot_controller
    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        name='ros_robot_controller',
        output='screen',
        parameters=[{
            'port': '/dev/ttyAMA0',
            'baudrate': 1000000,
        }],
    )

    # cmd_vel_bridge node
    cmd_vel_bridge_node = Node(
        package='cmd_vel_bridge',
        executable='cmd_vel_bridge_node',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'wheel_radius': 0.05,
            'wheel_base': 0.15,
            'track_width': 0.15,
            'max_wheel_rps': 3.0,
            'cmd_vel_timeout': 0.5,
        }],
    )

    # arm_controller node
    arm_controller_node = Node(
        package='arm_controller',
        executable='arm_controller_node',
        name='arm_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_arm')),
    )

    # lidar_driver node
    lidar_driver_node = Node(
        package='lidar_driver',
        executable='lidar_driver_node',
        name='lidar_driver',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        parameters=[{
            'port': '/dev/ttyUSB0',
            'port_fallback': '/dev/ttyUSB1',
            'baudrate': 230400,
            'frame_id': 'laser_frame',
        }],
    )

    return LaunchDescription([
        # Arguments
        enable_lidar_arg,
        enable_arm_arg,
        enable_camera_arg,

        # Log startup
        LogInfo(msg='Starting LanderPi ROS2 stack...'),

        # Nodes
        ros_robot_controller_node,
        cmd_vel_bridge_node,
        arm_controller_node,
        lidar_driver_node,

        LogInfo(msg='LanderPi ROS2 stack launched'),
    ])
```

**Step 2: Commit**

```bash
git add ros2_nodes/cmd_vel_bridge/launch/landerpi.launch.py
git commit -m "feat(launch): update launch file to start all nodes"
```

---

## Phase 4: Create ROS2 Test Scripts

### Task 4.1: Create test_arm_ros2.py

**Files:**
- Create: `test_arm_ros2.py`

**Step 1: Create the test script**

```python
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


def load_config() -> dict:
    """Load connection config from config.json if exists."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
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
        """Publish command to arm topic."""
        cmd_json = json.dumps(cmd).replace('"', '\\"')
        try:
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash && "
                f"ros2 topic pub --once {ARM_CMD_TOPIC} std_msgs/msg/String "
                f"\"{{data: \\\"{cmd_json}\\\"}}\""
                f"'",
                hide=True, warn=True
            )
            return result.ok
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
        tester.go_home(duration)
        time.sleep(1)
        tester.control_gripper("open")
        time.sleep(1)
        tester.control_gripper("close")
        time.sleep(1)
        tester.go_home(duration)
        console.print("\n[bold green]Arm test complete![/bold green]")
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
```

**Step 2: Commit**

```bash
git add test_arm_ros2.py
git commit -m "feat: add test_arm_ros2.py for ROS2 stack testing"
```

---

### Task 4.2: Create test_lidar_ros2.py

**Files:**
- Create: `test_lidar_ros2.py`

**Step 1: Create the test script**

```python
#!/usr/bin/env python3
"""
ROS2-based lidar test for LanderPi robot.
Requires deployed ROS2 stack: `deploy_ros2_stack.py deploy`

Tests lidar via /scan topic.
"""
import json
import sys
from pathlib import Path
from typing import Optional

import typer
from fabric import Connection
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

app = typer.Typer(help="LanderPi Lidar Test (ROS2 Stack)")
console = Console()

# ROS2 topics
SCAN_TOPIC = "/scan"


def load_config() -> dict:
    """Load connection config from config.json if exists."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
    return {}


class LidarROS2Test:
    """Test lidar via ROS2 stack."""

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

    def check_scan_topic(self) -> bool:
        """Check if scan topic exists."""
        try:
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash && "
                f"ros2 topic list | grep {SCAN_TOPIC}'",
                hide=True, warn=True
            )
            if result.ok:
                self.console.print(f"[green]Scan topic found: {SCAN_TOPIC}[/green]")
                return True
            self.console.print(f"[yellow]Scan topic not found: {SCAN_TOPIC}[/yellow]")
            return False
        except Exception as e:
            self.console.print(f"[red]Topic check failed:[/red] {e}")
            return False

    def get_topic_hz(self) -> Optional[float]:
        """Get scan topic publish rate."""
        try:
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash && "
                f"timeout 3 ros2 topic hz {SCAN_TOPIC} --window 5' 2>&1 | tail -3",
                hide=True, warn=True, timeout=10
            )
            if result.ok:
                for line in result.stdout.split('\n'):
                    if 'average rate:' in line:
                        rate = float(line.split(':')[1].strip().split()[0])
                        return rate
            return None
        except Exception:
            return None

    def read_scan(self, samples: int = 1) -> dict:
        """Read scan data and return statistics."""
        try:
            # Use Python to analyze scan data
            script = f'''
import sys
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time

class ScanReader(Node):
    def __init__(self):
        super().__init__('scan_reader')
        self.samples = []
        self.max_samples = {samples}
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '{SCAN_TOPIC}', self.cb, qos)

    def cb(self, msg):
        if len(self.samples) >= self.max_samples:
            return
        ranges = [r for r in msg.ranges if 0.05 < r < 12.0 and not math.isinf(r)]
        if ranges:
            self.samples.append({{
                'min': min(ranges),
                'max': max(ranges),
                'avg': sum(ranges)/len(ranges),
                'count': len(ranges),
                'total': len(msg.ranges)
            }})
            print(f"Sample {{len(self.samples)}}: min={{min(ranges):.2f}}m max={{max(ranges):.2f}}m points={{len(ranges)}}")

rclpy.init()
node = ScanReader()
start = time.time()
while len(node.samples) < node.max_samples and time.time() - start < 10:
    rclpy.spin_once(node, timeout_sec=0.5)
if node.samples:
    print("SUCCESS")
else:
    print("NO_DATA")
rclpy.shutdown()
'''
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash && "
                f"python3 -c \"{script}\"'",
                hide=False, warn=True, timeout=20
            )
            return {'success': 'SUCCESS' in result.stdout}
        except Exception as e:
            self.console.print(f"[red]Scan read error:[/red] {e}")
            return {'success': False}


@app.command()
def check(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Check lidar status via ROS2 stack."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]LanderPi Lidar Check (ROS2)[/bold]\n\n"
        f"Host: {host}\n"
        f"Topic: {SCAN_TOPIC}\n\n"
        f"[dim]Using deployed ROS2 stack[/dim]",
        title="Lidar Check",
        border_style="blue"
    ))

    tester = LidarROS2Test(host, user, password)

    checks = []

    if not tester.verify_connection():
        sys.exit(1)

    stack_ok = tester.check_ros2_stack()
    checks.append(("ROS2 Stack", stack_ok))

    if stack_ok:
        topic_ok = tester.check_scan_topic()
        checks.append(("Scan Topic", topic_ok))

        if topic_ok:
            hz = tester.get_topic_hz()
            if hz:
                checks.append(("Publish Rate", True))
                console.print(f"[green]Publish rate: {hz:.1f} Hz[/green]")
            else:
                checks.append(("Publish Rate", False))

    # Summary
    console.print("\n[bold]Summary:[/bold]")
    all_ok = True
    for name, ok in checks:
        status = "[green]PASS[/green]" if ok else "[red]FAIL[/red]"
        if not ok:
            all_ok = False
        console.print(f"  {name}: {status}")

    sys.exit(0 if all_ok else 1)


@app.command()
def scan(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    samples: int = typer.Option(5, help="Number of samples to read"),
):
    """Read lidar scan data via ROS2 stack."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]LanderPi Lidar Scan (ROS2)[/bold]\n\n"
        f"Host: {host}\n"
        f"Samples: {samples}",
        title="Scan Test",
        border_style="blue"
    ))

    tester = LidarROS2Test(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    if not tester.check_ros2_stack():
        sys.exit(1)

    console.print(f"\n[blue]Reading {samples} scan samples...[/blue]\n")
    result = tester.read_scan(samples)

    if result.get('success'):
        console.print("\n[bold green]Lidar scan test complete![/bold green]")
    else:
        console.print("\n[red]Failed to read scan data[/red]")
        sys.exit(1)


@app.command()
def status(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Get lidar driver status."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    tester = LidarROS2Test(host, user, password)
    if not tester.verify_connection() or not tester.check_ros2_stack():
        sys.exit(1)

    # Check topic and rate
    topic_ok = tester.check_scan_topic()
    if topic_ok:
        hz = tester.get_topic_hz()
        if hz:
            console.print(f"[green]Lidar publishing at {hz:.1f} Hz[/green]")
        else:
            console.print("[yellow]Lidar topic exists but rate unknown[/yellow]")
    else:
        console.print("[red]Lidar not publishing[/red]")


if __name__ == "__main__":
    app()
```

**Step 2: Commit**

```bash
git add test_lidar_ros2.py
git commit -m "feat: add test_lidar_ros2.py for ROS2 stack testing"
```

---

### Task 4.3: Create test_cameradepth_ros2.py

**Files:**
- Create: `test_cameradepth_ros2.py`

**Step 1: Create the test script**

```python
#!/usr/bin/env python3
"""
ROS2-based depth camera test for LanderPi robot.
Requires deployed ROS2 stack: `deploy_ros2_stack.py deploy`

Tests camera via /aurora/* topics.
"""
import json
import sys
from pathlib import Path
from typing import Optional

import typer
from fabric import Connection
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

app = typer.Typer(help="LanderPi Depth Camera Test (ROS2 Stack)")
console = Console()

# ROS2 topics
CAMERA_NAMESPACE = "aurora"
TOPICS = {
    "rgb": f"/{CAMERA_NAMESPACE}/rgb/image_raw",
    "depth": f"/{CAMERA_NAMESPACE}/depth/image_raw",
    "ir": f"/{CAMERA_NAMESPACE}/ir/image_raw",
    "points": f"/{CAMERA_NAMESPACE}/points2",
}


def load_config() -> dict:
    """Load connection config from config.json if exists."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
    return {}


class CameraROS2Test:
    """Test camera via ROS2 stack."""

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

    def check_camera_topics(self) -> dict:
        """Check which camera topics exist."""
        results = {}
        try:
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash 2>/dev/null; "
                f"source /deptrum_ws/install/local_setup.bash 2>/dev/null; "
                f"ros2 topic list | grep {CAMERA_NAMESPACE}'",
                hide=True, warn=True
            )
            if result.ok:
                found = result.stdout.strip().split('\n')
                for name, topic in TOPICS.items():
                    results[name] = topic in found
            else:
                for name in TOPICS:
                    results[name] = False
        except Exception:
            for name in TOPICS:
                results[name] = False
        return results

    def get_topic_info(self, topic: str) -> Optional[dict]:
        """Get topic type and publisher count."""
        try:
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash 2>/dev/null; "
                f"source /deptrum_ws/install/local_setup.bash 2>/dev/null; "
                f"ros2 topic info {topic}'",
                hide=True, warn=True
            )
            if result.ok:
                info = {}
                for line in result.stdout.split('\n'):
                    if 'Type:' in line:
                        info['type'] = line.split(':')[1].strip()
                    if 'Publisher count:' in line:
                        info['publishers'] = int(line.split(':')[1].strip())
                return info
            return None
        except Exception:
            return None

    def read_stream_sample(self, stream: str) -> bool:
        """Read one sample from a stream."""
        topic = TOPICS.get(stream)
        if not topic:
            return False

        try:
            result = self.conn.run(
                f"docker exec landerpi-ros2 bash -c '"
                f"source /opt/ros/humble/setup.bash && "
                f"source /ros2_ws/install/setup.bash 2>/dev/null; "
                f"source /deptrum_ws/install/local_setup.bash 2>/dev/null; "
                f"timeout 5 ros2 topic echo --once {topic} --field header'",
                hide=True, warn=True, timeout=10
            )
            return result.ok and 'stamp' in result.stdout
        except Exception:
            return False


@app.command()
def check(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Check camera status via ROS2 stack."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]LanderPi Camera Check (ROS2)[/bold]\n\n"
        f"Host: {host}\n"
        f"Namespace: {CAMERA_NAMESPACE}\n\n"
        f"[dim]Using deployed ROS2 stack[/dim]",
        title="Camera Check",
        border_style="blue"
    ))

    tester = CameraROS2Test(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    if not tester.check_ros2_stack():
        sys.exit(1)

    # Check camera topics
    console.print("\n[blue]Checking camera topics...[/blue]")
    topics = tester.check_camera_topics()

    table = Table(title="Camera Topics")
    table.add_column("Stream", style="cyan")
    table.add_column("Topic", style="dim")
    table.add_column("Status", justify="center")

    all_ok = True
    for name, topic in TOPICS.items():
        found = topics.get(name, False)
        status = "[green]FOUND[/green]" if found else "[red]MISSING[/red]"
        if not found:
            all_ok = False
        table.add_row(name.upper(), topic, status)

    console.print(table)

    if all_ok:
        console.print("\n[bold green]All camera topics available![/bold green]")
    else:
        console.print("\n[yellow]Some camera topics missing[/yellow]")
        console.print("[dim]Camera driver may need to be enabled in launch file[/dim]")

    sys.exit(0 if all_ok else 1)


@app.command()
def stream(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Read camera stream samples via ROS2 stack."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel.fit(
        f"[bold]LanderPi Camera Stream Test (ROS2)[/bold]\n\n"
        f"Host: {host}",
        title="Stream Test",
        border_style="blue"
    ))

    tester = CameraROS2Test(host, user, password)

    if not tester.verify_connection():
        sys.exit(1)

    if not tester.check_ros2_stack():
        sys.exit(1)

    # Test each stream
    console.print("\n[blue]Reading stream samples...[/blue]\n")

    results = {}
    for name in ['rgb', 'depth', 'ir']:
        console.print(f"  Testing {name.upper()}...", end=" ")
        success = tester.read_stream_sample(name)
        results[name] = success
        if success:
            console.print("[green]OK[/green]")
        else:
            console.print("[red]FAIL[/red]")

    if all(results.values()):
        console.print("\n[bold green]Camera stream test complete![/bold green]")
    else:
        console.print("\n[yellow]Some streams not available[/yellow]")
        sys.exit(1)


@app.command()
def status(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Get detailed camera status."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    tester = CameraROS2Test(host, user, password)
    if not tester.verify_connection() or not tester.check_ros2_stack():
        sys.exit(1)

    console.print("\n[blue]Getting topic info...[/blue]\n")

    table = Table(title="Camera Topic Details")
    table.add_column("Topic", style="cyan")
    table.add_column("Type", style="dim")
    table.add_column("Publishers", justify="center")

    for name, topic in TOPICS.items():
        info = tester.get_topic_info(topic)
        if info:
            table.add_row(
                topic,
                info.get('type', 'unknown'),
                str(info.get('publishers', 0))
            )
        else:
            table.add_row(topic, "[red]not found[/red]", "-")

    console.print(table)


if __name__ == "__main__":
    app()
```

**Step 2: Commit**

```bash
git add test_cameradepth_ros2.py
git commit -m "feat: add test_cameradepth_ros2.py for ROS2 stack testing"
```

---

## Phase 5: Update Documentation

### Task 5.1: Update landerpi-robot agent

**Files:**
- Modify: `.claude/agents/landerpi-robot.md`

**Step 1: Add Testing Modes section after "Core Responsibilities"**

Add after line ~71 (after Core Responsibilities section):

```markdown
## Testing Modes

LanderPi supports two testing approaches:

### Direct SDK Tests (No ROS2)
- Scripts: `test_chassis_direct.py`, `test_arm.py`, `test_lidar.py`, `test_cameradepth.py`
- Use `ros_robot_controller_sdk.py` for direct serial communication
- Self-contained, start their own Docker containers if needed
- Good for: Quick testing, debugging, when ROS2 stack is not deployed

### ROS2 Stack Tests (Integrated)
- Scripts: `test_chassis_motion.py`, `test_arm_ros2.py`, `test_lidar_ros2.py`, `test_cameradepth_ros2.py`
- Require persistent ROS2 stack: `deploy_ros2_stack.py deploy`
- Use standard ROS2 topics via `docker exec landerpi-ros2`
- Good for: Production use, integration testing, nav2/SLAM workflows

### When to Use Which

| Scenario | Use Direct | Use ROS2 |
|----------|------------|----------|
| Quick hardware verification | Yes | |
| Debugging motor/servo issues | Yes | |
| Integration with nav2/SLAM | | Yes |
| Production deployment | | Yes |
| Robot offline/recovery | Yes | |
```

**Step 2: Update Quick Reference Commands to include ROS2 test scripts**

Add in the Quick Reference Commands section:

```markdown
# ROS2 Stack Tests (requires: deploy_ros2_stack.py deploy)
uv run python test_arm_ros2.py test --yes        # Arm via ROS2
uv run python test_lidar_ros2.py scan --samples 5  # Lidar via ROS2
uv run python test_cameradepth_ros2.py stream    # Camera via ROS2
```

**Step 3: Commit**

```bash
git add .claude/agents/landerpi-robot.md
git commit -m "docs(agent): add testing modes and ROS2 test scripts"
```

---

### Task 5.2: Update landerpi-core skill

**Files:**
- Modify: `.claude/skills/landerpi-core/SKILL.md`

**Step 1: Add ROS2 Stack Architecture section at the end**

```markdown
## ROS2 Stack Architecture

The persistent ROS2 stack runs all drivers in a single Docker Compose service.

### Stack Components

| Node | Topic | Purpose |
|------|-------|---------|
| `ros_robot_controller` | `/ros_robot_controller/*` | STM32 communication (vendor) |
| `cmd_vel_bridge` | `/cmd_vel` → motors | Motion control |
| `arm_controller` | `/arm/cmd`, `/arm/state` | Arm servo control |
| `lidar_driver` | `/scan` | Lidar scanning |
| `camera_driver` | `/aurora/*` | Depth camera (vendor) |

### Deployment

```bash
# Deploy and start persistent ROS2 stack
uv run python deploy_ros2_stack.py deploy

# Stop stack
uv run python deploy_ros2_stack.py stop

# View logs
uv run python deploy_ros2_stack.py logs
uv run python deploy_ros2_stack.py logs -f  # Follow
```

### Testing Pattern

All `*_ros2.py` test scripts follow this pattern:
1. SSH to robot
2. `docker exec landerpi-ros2` to run ROS2 commands
3. Use `ros2 topic pub` to send commands
4. Use `ros2 topic echo` to read data

### Direct vs ROS2 Testing

| Script Type | When to Use |
|-------------|-------------|
| `test_*.py` (direct) | Quick testing, debugging, offline recovery |
| `test_*_ros2.py` | Production, nav2/SLAM integration |
```

**Step 2: Commit**

```bash
git add .claude/skills/landerpi-core/SKILL.md
git commit -m "docs(skill): add ROS2 stack architecture to landerpi-core"
```

---

### Task 5.3: Update other landerpi skills with ROS2 references

**Files:**
- Modify: `.claude/skills/landerpi-motion/SKILL.md`
- Modify: `.claude/skills/landerpi-arm/SKILL.md`
- Modify: `.claude/skills/landerpi-lidar/SKILL.md`
- Modify: `.claude/skills/landerpi-camera/SKILL.md`

**Step 1: Add ROS2 test reference to each skill**

Add to **landerpi-motion** (after Motion Commands table):
```markdown
### ROS2 Stack Testing

For ROS2-based testing (requires deployed stack):
```bash
uv run python test_chassis_motion.py test  # Test via /cmd_vel topic
```
```

Add to **landerpi-arm** (after Arm Commands or similar section):
```markdown
### ROS2 Stack Testing

For ROS2-based testing (requires deployed stack):
```bash
uv run python test_arm_ros2.py test --yes   # Test via /arm/cmd topic
uv run python test_arm_ros2.py home --yes   # Home position
uv run python test_arm_ros2.py status       # Read arm state
```
```

Add to **landerpi-lidar** (after Lidar Commands):
```markdown
### ROS2 Stack Testing

For ROS2-based testing (requires deployed stack):
```bash
uv run python test_lidar_ros2.py check       # Check lidar status
uv run python test_lidar_ros2.py scan --samples 5  # Read scan data
```
```

Add to **landerpi-camera** (after Camera Commands):
```markdown
### ROS2 Stack Testing

For ROS2-based testing (requires deployed stack):
```bash
uv run python test_cameradepth_ros2.py check   # Check camera topics
uv run python test_cameradepth_ros2.py stream  # Read stream samples
```
```

**Step 2: Commit**

```bash
git add .claude/skills/landerpi-motion/SKILL.md
git add .claude/skills/landerpi-arm/SKILL.md
git add .claude/skills/landerpi-lidar/SKILL.md
git add .claude/skills/landerpi-camera/SKILL.md
git commit -m "docs(skills): add ROS2 test script references to all landerpi skills"
```

---

### Task 5.4: Update CLAUDE.md with new commands

**Files:**
- Modify: `CLAUDE.md`

**Step 1: Add ROS2 test commands to Commands section**

Add after the existing test commands:

```markdown
# Test sensors via ROS2 stack (requires deploy_ros2_stack.py deploy)
uv run python test_arm_ros2.py test --yes        # Arm via ROS2 topics
uv run python test_arm_ros2.py home --yes        # Home position
uv run python test_arm_ros2.py status            # Arm state
uv run python test_lidar_ros2.py check           # Lidar status
uv run python test_lidar_ros2.py scan --samples 5  # Read scan data
uv run python test_cameradepth_ros2.py check     # Camera topics
uv run python test_cameradepth_ros2.py stream    # Read streams
```

**Step 2: Update Architecture section**

Add new entries for arm_controller and lidar_driver in the Architecture section.

**Step 3: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: add ROS2 test commands to CLAUDE.md"
```

---

## Final Task: Merge to main

### Task 6.1: Final verification and merge

**Step 1: Run linting**

```bash
ruff check ros2_nodes/ test_arm_ros2.py test_lidar_ros2.py test_cameradepth_ros2.py
```

**Step 2: Review all changes**

```bash
git log --oneline main..HEAD
```

**Step 3: Merge to main**

```bash
git checkout main
git merge feature/ros2-unified-stack
git push origin main
```

---

## Summary

This plan creates:
- 2 new ROS2 nodes (`arm_controller`, `lidar_driver`)
- 3 new test scripts (`test_arm_ros2.py`, `test_lidar_ros2.py`, `test_cameradepth_ros2.py`)
- Updated Docker Compose with all drivers
- Updated launch file to start all nodes
- Updated agent and 5 skills with testing mode documentation

Total commits: ~12
Estimated implementation time: Can be done in parallel with subagents
