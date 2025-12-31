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

## Resources

### Reference Files

- `references/architecture.md` - Detailed architecture diagrams and component interactions
