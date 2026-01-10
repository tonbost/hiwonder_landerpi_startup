---
name: landerpi-dev
description: Development guidelines and code architecture for HiWonder LanderPi robot project. This skill should be used when creating new features, adding validation tests, writing deployment scripts, or modifying existing code. Ensures consistency with established patterns for CLI tools, remote execution, module design, and ROS2 integration.
---

# LanderPi Development Guide

## Overview

Development skill for the LanderPi robot setup project. Provides architectural patterns, coding conventions, and best practices to ensure new code integrates seamlessly with existing infrastructure.

## Project Structure

```
hiwonderSetup/
├── setup_landerpi.py      # Main deployment with state tracking
├── checkLanderPi.py       # Health check diagnostics
├── deploy_*.py            # Component deployment scripts
├── robot_*.py             # On-robot execution scripts
├── config.json            # SSH connection credentials
├── pyproject.toml         # Dependencies (typer, rich, fabric)
├── validation/            # Test and validation scripts
│   ├── test_*.py          # Component tests (direct + ROS2)
│   └── exploration/       # Modular exploration package
├── ros2_nodes/            # ROS2 nodes for Docker deployment
├── docker/                # Docker Compose for ROS2 stack
├── config/                # Robot configuration files
├── drivers/               # SDK and driver files
├── whisplay/              # Whisplay HAT driver files
│   └── install_wm8960_ubuntu.sh  # Ubuntu-compatible WM8960 installer
└── .claude/skills/        # LanderPi-specific skills
```

## CLI Tool Patterns

All CLI tools follow a consistent pattern using Typer + Rich.

### Standard CLI Structure

```python
#!/usr/bin/env python3
"""Brief description of what this tool does."""

import json
import sys
from pathlib import Path
from typing import Optional

import typer
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from fabric import Connection

app = typer.Typer(help="Tool description")
console = Console()


def load_config() -> dict:
    """Load connection config from config.json."""
    config_path = Path(__file__).parent / "config.json"
    if not config_path.exists():
        config_path = Path(__file__).parent.parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
    return {}


@app.command()
def main_command(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval prompt"),
):
    """Command description."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    # Show parameters panel
    console.print(Panel.fit(
        f"[bold]Tool Name[/bold]\n\nHost: {host}\nOption: value",
        title="Parameters",
        border_style="blue"
    ))

    # Request approval for dangerous operations
    if not skip_approval:
        console.print("\n[bold yellow]WARNING: Description of what will happen[/bold yellow]")
        from rich.prompt import Confirm
        if not Confirm.ask("Continue?", default=False):
            console.print("[blue]Cancelled[/blue]")
            sys.exit(0)

    # Execute with try/except for interrupts
    try:
        # ... implementation
        console.print("[green]Success message[/green]")
    except KeyboardInterrupt:
        console.print("\n[red]Interrupted![/red]")
        sys.exit(1)


if __name__ == "__main__":
    app()
```

### Key CLI Conventions

| Pattern | Convention |
|---------|------------|
| Config loading | `load_config()` from `config.json`, fallback to parent dir |
| Option defaults | Always allow `--host`, `--user`, `--password` overrides |
| Approval prompts | Use `--yes`/`-y` flag, always warn before robot movement |
| Output formatting | Rich `Panel`, `Table`, color codes (`[green]`, `[red]`, `[yellow]`) |
| Error handling | `sys.exit(1)` for errors, catch `KeyboardInterrupt` |
| Dangerous commands | Require explicit approval, provide emergency stop |

## Remote Execution Patterns

### Fabric Connection Pattern

```python
from fabric import Connection
from io import StringIO

class ComponentTest:
    def __init__(self, host: str, user: str, password: Optional[str] = None):
        self.host = host
        self.user = user
        connect_kwargs = {}
        if password:
            connect_kwargs['password'] = password
        self.conn = Connection(host=host, user=user, connect_kwargs=connect_kwargs)
        self.sdk_path = None

    def verify_connection(self) -> bool:
        """Check SSH connectivity."""
        try:
            self.conn.run("echo 'Connected'", hide=True)
            return True
        except Exception as e:
            console.print(f"[red]Connection failed: {e}[/red]")
            return False

    def execute_script(self, script: str, timeout: int = 10) -> bool:
        """Execute Python script on robot."""
        try:
            self.conn.put(StringIO(script), "/tmp/script.py")
            result = self.conn.run("python3 /tmp/script.py", warn=True, timeout=timeout)
            return result.ok
        except Exception as e:
            console.print(f"[red]Execution error: {e}[/red]")
            return False
```

### Remote Script Execution

```python
sdk_dir = self.sdk_path or "/tmp"

script = f"""import sys
sys.path.insert(0, '{sdk_dir}')
import time
from ros_robot_controller_sdk import Board

board = Board()
board.enable_reception()

# ... SDK operations

print("Result: success")
"""

self.execute_script(script, timeout=30)
```

### SDK Path Resolution

```python
# Prefer permanent location, fall back to /tmp
REMOTE_SDK_DIR_PERMANENT = "/home/{user}/ros_robot_controller"
REMOTE_SDK_PATH_TMP = "/tmp/ros_robot_controller_sdk.py"

def upload_sdk(self) -> bool:
    """Upload SDK, prefer permanent location."""
    permanent_path = REMOTE_SDK_DIR_PERMANENT.format(user=self.user)

    # Check permanent first
    result = self.conn.run(f"test -f {permanent_path}/ros_robot_controller_sdk.py", warn=True)
    if result.ok:
        self.sdk_path = permanent_path
        return True

    # Fall back to /tmp
    # ... upload logic
```

## Module Design Patterns

The `validation/exploration/` module exemplifies proper modular design.

### Package Structure

```
exploration/
├── __init__.py           # Public API exports
├── explorer.py           # Main controller (orchestrates components)
├── ros2_hardware.py      # Hardware abstraction layer
├── safety_monitor.py     # Safety checks and battery monitoring
├── sensor_fusion.py      # Sensor data processing
├── frontier_planner.py   # Navigation planning
├── escape_handler.py     # Stuck detection and recovery
├── arm_scanner.py        # Arm-mounted camera scanning
├── data_logger.py        # Local logging
└── remote_data_logger.py # Remote logging variant
```

### Component Pattern

Each component follows this structure:

```python
from dataclasses import dataclass
from typing import Callable, Optional


@dataclass
class ComponentConfig:
    """Configuration with sensible defaults."""
    param1: float = 1.0
    param2: int = 10
    threshold: float = 0.5


class Component:
    """Single-responsibility component."""

    def __init__(
        self,
        config: Optional[ComponentConfig] = None,
        callback_func: Optional[Callable[[str], None]] = None,
    ):
        self.config = config or ComponentConfig()
        self.callback = callback_func or (lambda x: None)
        self._internal_state = []

    def process(self, data: dict) -> dict:
        """Public API method."""
        result = self._internal_process(data)
        return result

    def _internal_process(self, data: dict) -> dict:
        """Private implementation."""
        # ... processing
        return {}
```

### Dependency Injection

```python
class ExplorationController:
    def __init__(
        self,
        move_func: Callable[[float, float, float], bool],  # vx, vy, wz -> success
        stop_func: Callable[[], None],
        get_battery_func: Callable[[], Optional[float]],
        exploration_config: Optional[ExplorationConfig] = None,
        safety_config: Optional[SafetyConfig] = None,
        # ... more optional configs
    ):
        self.config = exploration_config or ExplorationConfig()
        self.move = move_func
        self.stop = stop_func

        # Inject sub-components
        self.safety = SafetyMonitor(
            safety_config or SafetyConfig(),
            get_battery_func,
        )
```

### Protocol-Based Interfaces

```python
from typing import Protocol


class LoggerProtocol(Protocol):
    """Protocol for pluggable loggers."""
    session_dir: str | None

    def start_session(self, config: dict | None = None) -> str: ...
    def log_event(self, event_type: str, details: dict | None = None) -> None: ...
    def end_session(self, summary: dict | None = None) -> dict: ...


# Usage: accept any logger implementing the protocol
def __init__(self, logger: Optional[LoggerProtocol] = None):
    self.logger = logger or DataLogger()
```

### Public API Exports

```python
# __init__.py
from .safety_monitor import SafetyMonitor, SafetyConfig
from .explorer import ExplorationController, ExplorationConfig
# ... all public classes

__all__ = [
    "SafetyMonitor",
    "SafetyConfig",
    "ExplorationController",
    "ExplorationConfig",
    # ... complete list
]
```

## Naming Conventions

| Element | Convention | Example |
|---------|------------|---------|
| Files | `snake_case.py` | `test_arm.py`, `deploy_ros2_stack.py` |
| Classes | `PascalCase` | `ExplorationController`, `SafetyMonitor` |
| Functions | `snake_case` | `load_config()`, `verify_connection()` |
| Constants | `UPPER_SNAKE` | `HOME_POSITION`, `DEFAULT_DURATION` |
| Config classes | `*Config` suffix | `ExplorationConfig`, `SafetyConfig` |
| Test files | `test_*.py` | `test_arm.py`, `test_lidar.py` |
| Deploy scripts | `deploy_*.py` | `deploy_ros2_stack.py` |
| Private methods | `_prefix` | `_internal_process()` |
| Callbacks | `*_func` suffix | `move_func`, `on_speak` |

## Validation Test Patterns

### Test File Structure

```python
#!/usr/bin/env python3
"""
LanderPi [component] test using [method].
[Additional context about requirements]

SAFETY: [Warning about what will happen]
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
from rich.table import Table
from fabric import Connection

app = typer.Typer(help="LanderPi [Component] Test")
console = Console()

# Configuration constants
DEFAULT_VALUE = 1.0
SERVO_IDS = [1, 2, 3, 4, 5]


class ComponentTest:
    """Test class encapsulating all test logic."""
    # ... implementation


@app.command()
def test(...):
    """Run component test."""

@app.command()
def status(...):
    """Get component status."""

@app.command()
def stop(...):
    """Emergency stop."""


if __name__ == "__main__":
    app()
```

### Direct vs ROS2 Test Variants

- `test_arm.py` - Direct SDK control (no ROS2)
- `test_arm_ros2.py` - ROS2 topic-based control

Direct tests use SDK for immediate hardware access. ROS2 tests use deployed stack via `docker exec`.

## ROS2 Integration Patterns

### ROS2 Command Execution

```python
def run_ros2_command(self, cmd: str, timeout: float = 5.0) -> Optional[str]:
    """Execute ROS2 command in Docker container."""
    full_cmd = f"docker exec landerpi-ros2 bash -c 'source /opt/ros/humble/setup.bash && {cmd}'"
    result = self.conn.run(full_cmd, hide=True, warn=True, timeout=timeout)
    if result.ok:
        return result.stdout.strip()
    return None
```

### Topic Publishing

```python
# Publish velocity command
cmd = f"ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{{linear: {{x: {vx}, y: {vy}}}, angular: {{z: {wz}}}}}'"
self.run_ros2_command(cmd)
```

### Topic Reading

```python
# Read single message with timeout
cmd = f"timeout 2 ros2 topic echo --once /scan --field ranges"
result = self.run_ros2_command(cmd, timeout=5.0)
```

## ROS2 Package Structure (ament_python)

All ROS2 nodes in `ros2_nodes/` use `ament_python` build type. **A common pitfall is missing `setup.cfg`**, which causes executables to be installed in the wrong location.

### Required Files for ament_python Package

```
ros2_nodes/<package_name>/
├── package.xml              # ROS2 package manifest
├── setup.py                 # Python package setup
├── setup.cfg                # CRITICAL: Script install location
├── resource/
│   └── <package_name>       # Empty marker file (required by ament)
└── <package_name>/
    ├── __init__.py          # Package init (can be empty)
    └── <node_name>.py       # Node implementation with main()
```

### setup.cfg (CRITICAL)

**Without `setup.cfg`, ROS2 launch will fail with:**
```
[ERROR] [launch]: package '<pkg>' found at '/ros2_ws/install/<pkg>',
but libexec directory '/ros2_ws/install/<pkg>/lib/<pkg>' does not exist
```

**Root cause:** By default, setuptools installs console_scripts to `bin/`, but ROS2 `launch_ros.actions.Node` expects them in `lib/<package_name>/`.

**Required setup.cfg content:**
```ini
[develop]
script_dir=$base/lib/<package_name>
[install]
install_scripts=$base/lib/<package_name>
```

**Example for `yolo_hailo_bridge`:**
```ini
[develop]
script_dir=$base/lib/yolo_hailo_bridge
[install]
install_scripts=$base/lib/yolo_hailo_bridge
```

### setup.py Template

```python
from setuptools import setup

package_name = 'my_ros2_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LanderPi',
    maintainer_email='landerpi@example.com',
    description='Node description',
    license='MIT',
    entry_points={
        'console_scripts': [
            'node_executable = my_ros2_node.node_file:main',
        ],
    },
)
```

### package.xml Template

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_ros2_node</name>
  <version>0.1.0</version>
  <description>Node description</description>
  <maintainer email="landerpi@example.com">LanderPi</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### ROS2 Package Checklist

When creating a new ROS2 node:

1. [ ] `package.xml` has `<build_type>ament_python</build_type>`
2. [ ] `setup.py` has entry_points with console_scripts
3. [ ] **`setup.cfg` exists with correct `install_scripts` path**
4. [ ] `resource/<package_name>` marker file exists (can be empty)
5. [ ] `<package_name>/__init__.py` exists
6. [ ] Node file has `main()` function matching entry_point

### Debugging ROS2 Package Install

After `colcon build`, verify executable location:

```bash
# Should show executable in lib/<package_name>/, NOT bin/
docker exec landerpi-ros2 ls -la /ros2_ws/install/<package_name>/lib/<package_name>/

# If in bin/ instead, setup.cfg is missing or incorrect
docker exec landerpi-ros2 ls -la /ros2_ws/install/<package_name>/bin/
```

## Configuration Management

### Connection Config (config.json)

```json
{
  "host": "192.168.50.169",
  "user": "username",
  "password": "password"
}
```

### Dataclass Configs

```python
@dataclass
class ComponentConfig:
    """Always provide sensible defaults."""
    speed: float = 0.35
    timeout: float = 10.0
    threshold: int = 3

    def __post_init__(self):
        """Validate or compute derived values."""
        if self.speed > 0.5:
            self.speed = 0.5  # Safety limit
```

## Error Handling

### Standard Pattern

```python
try:
    result = self.conn.run(cmd, hide=True, warn=True, timeout=timeout)
    if result.ok:
        return parse_result(result.stdout)
    else:
        console.print(f"[yellow]Command failed: {result.stderr}[/yellow]")
        return None
except Exception as e:
    console.print(f"[red]Error: {e}[/red]")
    return None
```

### Safety-Critical Operations

```python
# Always wrap robot movement in try/finally
try:
    self.move(vx, vy, wz)
    time.sleep(duration)
finally:
    self.stop()  # Always stop on exit
```

## Documentation Standards

### Docstrings

```python
def function_name(param1: str, param2: float = 1.0) -> Optional[dict]:
    """Brief description of function.

    More detailed explanation if needed.

    Args:
        param1: Description of param1
        param2: Description of param2 (default: 1.0)

    Returns:
        Description of return value, or None if failed

    Raises:
        ConnectionError: If SSH connection fails
    """
```

### CLAUDE.md Updates

When adding new commands or features, update CLAUDE.md with:
- Command syntax in Commands section
- Architecture description if new component
- Key patterns if introducing new conventions

## Common Positions/Constants

Define hardware constants at module level:

```python
# Arm positions
HOME_POSITION = 500
EXPLORE_HOME = {
    1: 547,   # Base
    2: 818,   # Shoulder
    3: 203,   # Elbow
    4: 58,    # Wrist
    5: 501,   # Rotate
    10: 496,  # Gripper
}

# Motion limits
MAX_SPEED = 0.5  # m/s
MAX_TURN = 2.0   # rad/s
```

## Testing Checklist

Before committing new validation tests:

1. [ ] CLI follows standard pattern (typer, rich, config loading)
2. [ ] Dangerous operations require `--yes` approval
3. [ ] Emergency stop command provided
4. [ ] Status command for diagnostics
5. [ ] Proper error handling with informative messages
6. [ ] Docstring with safety warnings
7. [ ] CLAUDE.md updated with new commands
8. [ ] Skill documentation updated if applicable

## Sensor Fusion Patterns

### Multi-Sensor Distance with Priority

```python
@dataclass
class SensorConfig:
    stop_distance: float = 0.15  # Lidar obstacles (walls)
    semantic_stop_distance: float = 0.8  # YOLO hazards (people, pets, bottles)
    lidar_priority: bool = True  # Lidar as primary sensor

class SensorFusion:
    def get_obstacle_state(self) -> ObstacleState:
        # 1. Lidar is PRIMARY distance source
        self.state.closest_distance = self.state.lidar_distance

        # 2. Camera can only LOWER distance (safety override)
        if depth_distance < lidar_distance * 0.7:
            self.state.closest_distance = depth_distance

        # 3. YOLO semantic hazard triggers IMMEDIATE STOP
        if self.state.semantic_hazard:
            self.state.should_stop = True
```

### Depth Camera Distance Lookup

```python
def get_depth_distance(self, cx, cy, bbox_width, bbox_height):
    """Get distance from depth camera at bounding box location."""
    # Sample inner 50% of bbox (avoid edges)
    sample_w = int(bbox_width * 0.5)
    sample_h = int(bbox_height * 0.5)

    # Extract ROI around bbox center
    roi = depth_img[y1:y2, x1:x2]

    # Filter valid depths and return minimum
    valid_depths = roi[(roi > 100) & (roi < 10000)]  # 10cm - 10m
    return float(np.min(valid_depths)) * depth_scale  # mm to meters
```

### Semantic vs Physical Obstacles

| Obstacle Type | Detection | Stop Distance | Example |
|---------------|-----------|---------------|---------|
| Physical | Lidar | 0.15m | Walls, furniture |
| Low obstacle | Depth camera | 0.15m | Stairs, cables |
| Semantic | YOLO + Depth | 0.8m | People, pets, bottles |

### Threshold Configuration (Critical)

The fusion node's `hazard_distance` must be **larger** than the explorer's `semantic_stop_distance`:

| Parameter | Location | Value | Purpose |
|-----------|----------|-------|---------|
| `hazard_distance` | launch file | 2.5m | Fusion reports hazards within this distance |
| `semantic_stop_distance` | sensor_fusion.py | 0.8m | Explorer stops for hazards within this distance |

**Why this matters:** If `hazard_distance < semantic_stop_distance`, hazards are filtered out before the explorer sees them, causing the robot to collide with detected objects.

## Timestamped Logging Pattern

All exploration-related output includes timestamps for debugging and log correlation:

### Timestamp Helper Function

```python
from datetime import datetime

def ts() -> str:
    """Return timestamp string for log correlation with YOLO logs."""
    return datetime.now().strftime("[%H:%M:%S.%f")[:-3] + "]"
```

### Usage in Console Output

```python
from rich.console import Console
console = Console()

# Status messages
console.print(f"{ts()} [red]Obstacle at {distance:.2f}m - stopped[/red]")
console.print(f"{ts()} [yellow]Starting progressive escape[/yellow]")
console.print(f"{ts()} [red]HAZARD: {h['type']} at {h['distance']:.2f}m - STOPPING[/red]")

# Plain print for robot scripts
print(f"{ts()} Status: {remaining:.1f} min remaining | action: {action}")
print(f"{ts()} [TARS] {message}")
```

### Output Format

```
[15:47:52.123] Obstacle at 0.08m - stopped (blocked: 1)
[15:47:52.456] [TARS] Starting exploration
[15:48:01.789] HAZARD: bottle at 0.85m - STOPPING
[15:48:02.012] Status: 4.4 min remaining | action: stopped | escape: NONE
```

### Log Correlation

Timestamps enable cross-referencing explorer output with YOLO detection logs:

1. Explorer output: `[15:48:01.789] HAZARD: bottle at 0.85m`
2. YOLO log (Unix): `{"timestamp": 1735760881.789, ...}`
3. Convert: `date -r 1735760881` → `15:48:01`

Files using this pattern:
- `validation/exploration/explorer.py`
- `validation/exploration/sensor_fusion.py`
- `validation/exploration/escape_handler.py`
- `robot_explorer.py`

## Hailo Integration Patterns

When working with Hailo-8 accelerated code:

1. **Check hardware first**: Always verify Hailo is available before using
   ```python
   try:
       from hailo_platform import HEF, VDevice
       HAILO_AVAILABLE = True
   except ImportError:
       HAILO_AVAILABLE = False
   ```

2. **Fallback to CPU**: Provide CPU fallback when Hailo unavailable
   ```python
   if HAILO_AVAILABLE and hef_path.exists():
       # Use Hailo inference
   else:
       # Fall back to ultralytics CPU inference
   ```

3. **Same interface**: Hailo nodes should match CPU node interfaces
   - Subscribe to same topics
   - Publish same message types
   - Use same parameter names

4. **HEF models**: Models must be pre-converted, not converted at runtime
   - Conversion requires x86_64 with Hailo SDK
   - HEF files stored in `hailo8-int/models/`
   - Deployed to `~/landerpi/hailo/models/` on robot

5. **Native installation**: HailoRT runs native (not in Docker)
   - Requires kernel module (`hailo_pci`)
   - Device node at `/dev/hailo0`
   - Python bindings via `pip install hailort`

## ZeroMQ Bridge Pattern

When bridging between different Python environments (e.g., host Python 3.13 + Docker Python 3.10):

### Server Side (Host)

```python
import zmq
import json

class InferenceServer:
    def __init__(self, port: int = 5555):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:{port}")

    def run(self):
        while True:
            if self.socket.poll(1000):  # 1 second timeout
                message = self.socket.recv()
                request = json.loads(message.decode())

                if request.get('type') == 'ping':
                    response = {'status': 'ok'}
                elif request.get('type') == 'infer':
                    # Process inference
                    result = self.infer(request['data'])
                    response = {'status': 'ok', 'detections': result}
                else:
                    response = {'status': 'error', 'message': 'Unknown type'}

                self.socket.send(json.dumps(response).encode())

    def shutdown(self):
        self.socket.close()
        self.context.term()
```

### Annotated Image Publishing Pattern

When adding annotated image output to a detection node:

```python
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        self.cv_bridge = CvBridge()
        self.annotated_publisher = self.create_publisher(
            Image, '/yolo/annotated_image', 10
        )

    def _draw_annotations(self, cv_image: np.ndarray, detections: list) -> np.ndarray:
        """Draw bounding boxes and labels on image."""
        for det in detections:
            cx, cy = int(det['cx']), int(det['cy'])
            w, h = int(det['width']), int(det['height'])
            x1, y1 = cx - w // 2, cy - h // 2
            x2, y2 = cx + w // 2, cy + h // 2

            # Red for hazards, green for others (BGR format)
            is_hazard = det['class'] in self.hazard_classes
            color = (0, 0, 255) if is_hazard else (0, 255, 0)

            cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)

            label = f"{det['class']} {det['score']:.2f}"
            (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(cv_image, (x1, y1-lh-6), (x1+lw+4, y1), color, -1)
            cv2.putText(cv_image, label, (x1+2, y1-4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return cv_image

    def _publish_annotated_image(self, original_msg: Image, detections: list):
        """Draw annotations and publish."""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(original_msg, 'bgr8')
            annotated = self._draw_annotations(cv_image, detections)
            annotated_msg = self.cv_bridge.cv2_to_imgmsg(annotated, 'bgr8')
            annotated_msg.header = original_msg.header
            self.annotated_publisher.publish(annotated_msg)
        except Exception:
            pass  # Skip frame on error
```

**Dependencies for package.xml:**
```xml
<exec_depend>cv_bridge</exec_depend>
```

**View with RViz:** Add Image display, set topic to `/yolo/annotated_image`

### Client Side (ROS2 Node in Docker)

```python
import zmq
import json
from rclpy.node import Node

class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')

        # IMPORTANT: Use _zmq_ctx, NOT context (reserved by ROS2 Node)
        self._zmq_ctx = zmq.Context()
        self.socket = self._zmq_ctx.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, 500)  # 500ms timeout
        self.socket.setsockopt(zmq.SNDTIMEO, 500)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect("tcp://localhost:5555")

    def send_request(self, data: dict) -> dict:
        try:
            self.socket.send(json.dumps(data).encode())
            response = json.loads(self.socket.recv().decode())
            return response
        except zmq.ZMQError as e:
            self.get_logger().warn(f'ZMQ error: {e}')
            self._reconnect()
            return {'status': 'error'}

    def _reconnect(self):
        self.socket.close()
        self.socket = self._zmq_ctx.socket(zmq.REQ)
        # ... reconfigure socket options
        self.socket.connect("tcp://localhost:5555")

    def destroy_node(self):
        self.socket.close()
        self._zmq_ctx.term()
        super().destroy_node()
```

### ROS2 Node Reserved Properties

**CRITICAL:** ROS2 `rclpy.node.Node` reserves certain property names. Using them causes `AttributeError: can't set attribute`.

| Reserved Name | Use Instead |
|---------------|-------------|
| `context` | `_zmq_ctx`, `zmq_context` |
| `node_name` | Custom prefix like `_node_name` |
| `logger` | Use `get_logger()` method |

Example error:
```
AttributeError: can't set attribute 'context'
```

Fix:
```python
# WRONG
self.context = zmq.Context()

# CORRECT
self._zmq_ctx = zmq.Context()
```

## Reading ROS2 Topics from Python Subprocess

When reading ROS2 topics from Python code running on the robot (not in Docker), there are critical performance considerations.

### The Problem: ros2 topic echo Initialization Overhead

`ros2 topic echo` creates a new ROS2 node each time it runs. This initialization takes **300-500ms** before any messages can be received:

```bash
# This takes ~400ms even if /hazards publishes at 10Hz
time docker exec landerpi-ros2 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic echo /hazards --once'
```

**Impact on control loops:**
- A 10Hz control loop expects 100ms per iteration
- A 400ms blocking read causes the loop to run at ~2Hz
- Robot can't react to obstacles fast enough
- Even lidar detection fails because the loop is too slow

### Shell Quoting in Nested Docker Commands

Running Python code inside `docker exec` with shell-executed subprocess has **three layers of quoting**:
1. Python subprocess shell
2. Docker exec shell
3. Inner bash shell

**Common failure:**
```python
# This FAILS with SyntaxError - quotes are stripped
cmd = '''docker exec landerpi-ros2 bash -c "python3 -c 'import rclpy; n = rclpy.create_node(\"reader\")'"'''

# Error: n = rclpy.create_node(reader)  <-- quotes stripped!
```

**Escape attempts that still fail:**
```python
# Still breaks - escaping doesn't survive all layers
cmd = 'docker exec landerpi-ros2 bash -c "python3 -c \\"x = \'test\'\\""'
```

**Solution: Avoid inline Python in docker exec**
```python
# GOOD: Use ros2 CLI tools with grep/sed outside docker
cmd = (
    'docker exec landerpi-ros2 bash -c '
    '"source /opt/ros/humble/setup.bash && timeout 0.3 ros2 topic echo /hazards 2>/dev/null" '
    '| grep -m1 "data:" | sed \'s/data: //\''
)
```

### Caching Pattern for Slow I/O

When reading data that takes longer than your control loop period, use caching:

```python
class HardwareInterface:
    def __init__(self):
        # Cache state
        self._last_read_time: float = 0.0
        self._cached_data: List[dict] = []
        self._read_interval: float = 0.5  # Only read every 0.5s

    def read_slow_topic(self) -> List[dict]:
        """Read with caching to avoid blocking control loop."""
        now = time.time()

        # Return cached value if interval not elapsed
        if now - self._last_read_time < self._read_interval:
            return self._cached_data

        self._last_read_time = now

        try:
            result = subprocess.run(
                cmd, shell=True, capture_output=True,
                text=True, timeout=1.0
            )
            if result.stdout.strip():
                self._cached_data = self._parse_output(result.stdout)
        except subprocess.TimeoutExpired:
            pass  # Keep cached value

        return self._cached_data
```

**Key points:**
1. Cache interval should be >= expected read time
2. On timeout, keep previous cached value (don't return empty)
3. Subprocess timeout should be > inner command timeout
4. Primary sensors (lidar) should NOT use this pattern - they need real-time data

### Control Loop Considerations

A robot control loop has strict timing requirements:

| Loop Rate | Max Blocking Time | Notes |
|-----------|-------------------|-------|
| 10 Hz | 100ms | Typical exploration loop |
| 20 Hz | 50ms | High-speed navigation |
| 5 Hz | 200ms | Slow/safe mode |

**Rules:**
1. **Never block primary sensors** (lidar, IMU) - they determine obstacle avoidance
2. **Cache secondary data** (YOLO hazards, battery) - they enhance but don't replace primary
3. **Measure actual timing** before deploying - use `time.time()` around slow calls
4. **Use background threads** for slow operations if caching isn't sufficient

### Recommended Architecture for Topic Reading

**Legacy approach (fallback when sensor bridge unavailable):**
```
Python subprocess → docker exec → ros2 topic echo → parse output
(~400-500ms per read, cached)
```

**Current approach (sensor_bridge node - IMPLEMENTED):**
```
Docker: sensor_bridge_node → rclpy subscriber → writes JSON to /landerpi_data/
Host: ROS2Hardware → reads JSON files (~10ms latency)
```

**Sensor Bridge Files:**
| File | Topic | Update Rate |
|------|-------|-------------|
| `lidar.json` | /scan | 20 Hz |
| `hazards.json` | /hazards | 20 Hz |
| `depth_stats.json` | /depth_stats | 10 Hz |
| `battery.json` | /battery | 1 Hz |

**Configuration:**
```python
# In ros2_hardware.py
ROS2Config(
    use_file_bridge=True,     # Enable file-based reading (default)
    data_dir="~/landerpi_data",
    file_stale_threshold=2.0  # Fallback if data > 2s old
)
```

**Performance Comparison:**
| Metric | Subprocess | Sensor Bridge |
|--------|------------|---------------|
| Topic read latency | 400-500ms | 5-10ms |
| Control loop rate | ~2-3Hz | 10Hz |
| Obstacle reaction | >1s | <200ms |

## Whisplay HAT Integration

The Whisplay HAT (PiSugar) provides LCD screen, physical buttons, LED indicators, and audio functions via WM8960 codec.

### Hardware Components

| Component | Interface | Description |
|-----------|-----------|-------------|
| LCD Screen | SPI | Display for status/UI |
| Physical Buttons | GPIO | User input |
| RGB LEDs | GPIO | Status indicators |
| WM8960 Codec | I2S/I2C | Audio input/output |

### Deployment Script

`deploy_whisplay.py` follows standard CLI patterns:

```bash
uv run python deploy_whisplay.py check      # Check HAT status
uv run python deploy_whisplay.py install    # Install WM8960 driver
uv run python deploy_whisplay.py test       # LCD/button/LED test
uv run python deploy_whisplay.py mic-test   # Microphone/speaker test
uv run python deploy_whisplay.py status     # Detailed audio status
uv run python deploy_whisplay.py uninstall  # Remove driver
```

### Ubuntu Compatibility

The upstream Whisplay driver requires `raspi-config` (Raspberry Pi OS only). For Ubuntu:

1. Custom installer `whisplay/install_wm8960_ubuntu.sh` bypasses raspi-config
2. Uses `$SUDO_USER` to find correct home directory when running as root
3. Manually configures `/boot/firmware/config.txt` for I2S/I2C overlays

### Installation Steps

The `deploy_whisplay.py install` command performs:
1. Install prerequisites (git, i2c-tools, swig, liblgpio-dev)
2. Clone Whisplay repository from GitHub
3. Upload Ubuntu-compatible installer (bypasses raspi-config)
4. Install WM8960 kernel driver and configure device tree overlays
5. Install Python dependencies: `pillow`, `pygame`, `spidev`, `rpi-lgpio`
6. Fix test scripts for Ubuntu (use `python3` instead of `python`)

**Note:** `rpi-lgpio` is used instead of `RPi.GPIO` because the latter doesn't work on Ubuntu 25.10+ (cannot determine SOC peripheral base address).

**Requires reboot** after installation to load kernel modules.

### WM8960 Audio Driver Details

The driver installation:
1. Adds kernel modules to `/etc/modules`: `i2c-dev`, `snd-soc-wm8960`, `snd-soc-wm8960-soundcard`
2. Configures device tree overlays in `/boot/firmware/config.txt`
3. Installs ALSA config files to `/etc/wm8960-soundcard/`
4. Enables `wm8960-soundcard.service` systemd service

### Audio Device Detection

After installation, verify with:
```bash
# Check I2C (WM8960 at 0x1a)
i2cdetect -y 1

# Check audio devices
aplay -l    # Should show wm8960
arecord -l  # Should show wm8960

# Check kernel module
lsmod | grep snd_soc_wm8960
```

## Resources

### Reference Files

- `references/architecture.md` - Detailed architecture diagrams and component interactions
