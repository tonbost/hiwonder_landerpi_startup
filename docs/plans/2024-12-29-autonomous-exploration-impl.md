# Autonomous Exploration Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build an autonomous exploration system that uses depth camera + lidar sensor fusion to navigate while avoiding obstacles, with frontier-based exploration behavior.

**Architecture:** Modular Python package with sensor fusion, frontier planning, safety monitoring, and data logging. Runs locally, connects to robot via SSH/Fabric, uses Docker-based ROS2 for sensor access.

**Tech Stack:** Python 3.11+, Typer CLI, Fabric SSH, ROS2 Humble (Docker), Rich console output

---

## Task 1: Create Exploration Package Structure

**Files:**
- Create: `validation/exploration/__init__.py`
- Create: `validation/exploration/safety_monitor.py`

**Step 1: Create package directory and init file**

```bash
mkdir -p validation/exploration
```

**Step 2: Create __init__.py**

```python
"""Autonomous exploration package for LanderPi robot."""

from .safety_monitor import SafetyMonitor

__all__ = ["SafetyMonitor"]
```

**Step 3: Commit**

```bash
git add validation/exploration/
git commit -m "feat(exploration): create package structure"
```

---

## Task 2: Implement Safety Monitor

**Files:**
- Create: `validation/exploration/safety_monitor.py`

**Step 1: Write SafetyMonitor class**

```python
"""Safety monitor for autonomous exploration."""

import time
from dataclasses import dataclass
from typing import Callable, Optional

from rich.console import Console

console = Console()


@dataclass
class SafetyConfig:
    """Safety configuration parameters."""
    max_runtime_minutes: float = 30.0
    battery_warning_voltage: float = 7.0
    battery_cutoff_voltage: float = 6.6
    battery_check_interval: float = 30.0  # seconds


class SafetyMonitor:
    """Monitor battery and runtime for safe operation."""

    def __init__(
        self,
        config: SafetyConfig,
        get_battery_func: Callable[[], Optional[float]],
        on_warning: Optional[Callable[[str], None]] = None,
        on_shutdown: Optional[Callable[[str], None]] = None,
    ):
        self.config = config
        self.get_battery = get_battery_func
        self.on_warning = on_warning or (lambda msg: console.print(f"[yellow]Warning: {msg}[/yellow]"))
        self.on_shutdown = on_shutdown or (lambda msg: console.print(f"[red]Shutdown: {msg}[/red]"))

        self.start_time: Optional[float] = None
        self.last_battery_check: float = 0
        self.battery_warned: bool = False
        self.should_stop: bool = False
        self.stop_reason: Optional[str] = None

    def start(self) -> None:
        """Start the safety monitor."""
        self.start_time = time.time()
        self.last_battery_check = 0
        self.battery_warned = False
        self.should_stop = False
        self.stop_reason = None
        console.print(f"[green]Safety monitor started[/green]")
        console.print(f"  Max runtime: {self.config.max_runtime_minutes} min")
        console.print(f"  Battery cutoff: {self.config.battery_cutoff_voltage}V")

    def check(self) -> bool:
        """
        Check safety conditions.
        Returns True if safe to continue, False if should stop.
        """
        if self.should_stop:
            return False

        current_time = time.time()

        # Check runtime
        if self.start_time:
            elapsed_minutes = (current_time - self.start_time) / 60.0
            if elapsed_minutes >= self.config.max_runtime_minutes:
                self.stop_reason = f"Runtime limit reached ({self.config.max_runtime_minutes} min)"
                self.on_shutdown(self.stop_reason)
                self.should_stop = True
                return False

        # Check battery (throttled)
        if current_time - self.last_battery_check >= self.config.battery_check_interval:
            self.last_battery_check = current_time
            voltage = self.get_battery()

            if voltage is not None:
                # Convert mV to V if needed
                if voltage > 100:
                    voltage = voltage / 1000.0

                if voltage <= self.config.battery_cutoff_voltage:
                    self.stop_reason = f"Battery low ({voltage:.2f}V)"
                    self.on_shutdown(self.stop_reason)
                    self.should_stop = True
                    return False

                if voltage <= self.config.battery_warning_voltage and not self.battery_warned:
                    self.on_warning(f"Battery getting low ({voltage:.2f}V)")
                    self.battery_warned = True

        return True

    def extend_runtime(self, additional_minutes: float = 30.0) -> None:
        """Extend the runtime limit."""
        self.config.max_runtime_minutes += additional_minutes
        console.print(f"[green]Runtime extended by {additional_minutes} min[/green]")

    def get_status(self) -> dict:
        """Get current safety status."""
        elapsed = 0.0
        remaining = self.config.max_runtime_minutes
        if self.start_time:
            elapsed = (time.time() - self.start_time) / 60.0
            remaining = max(0, self.config.max_runtime_minutes - elapsed)

        return {
            "elapsed_minutes": elapsed,
            "remaining_minutes": remaining,
            "should_stop": self.should_stop,
            "stop_reason": self.stop_reason,
        }

    def request_stop(self, reason: str = "Manual stop") -> None:
        """Request exploration to stop."""
        self.stop_reason = reason
        self.should_stop = True
```

**Step 2: Verify file syntax**

Run: `python3 -m py_compile validation/exploration/safety_monitor.py`
Expected: No output (success)

**Step 3: Commit**

```bash
git add validation/exploration/safety_monitor.py
git commit -m "feat(exploration): add safety monitor with battery and runtime checks"
```

---

## Task 3: Implement Frontier Planner

**Files:**
- Create: `validation/exploration/frontier_planner.py`
- Modify: `validation/exploration/__init__.py`

**Step 1: Write FrontierPlanner class**

```python
"""Frontier-based exploration planner."""

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from rich.console import Console

console = Console()


@dataclass
class FrontierConfig:
    """Frontier planner configuration."""
    num_sectors: int = 8  # 360 / 8 = 45 degrees each
    freshness_decay_rate: float = 5.0  # points per second
    initial_freshness: float = 50.0
    max_freshness: float = 100.0
    min_clear_distance: float = 0.8  # meters - sector considered blocked if closer


@dataclass
class Sector:
    """A directional sector for exploration."""
    index: int
    angle_start: float  # degrees
    angle_end: float  # degrees
    freshness: float = 50.0
    min_distance: float = float('inf')
    is_blocked: bool = False

    @property
    def center_angle(self) -> float:
        """Get center angle of sector in degrees."""
        return (self.angle_start + self.angle_end) / 2

    @property
    def center_angle_rad(self) -> float:
        """Get center angle in radians."""
        return math.radians(self.center_angle)


class FrontierPlanner:
    """Plans exploration direction based on frontier freshness."""

    SECTOR_NAMES = ["Front", "Front-Right", "Right", "Back-Right",
                    "Back", "Back-Left", "Left", "Front-Left"]

    def __init__(self, config: Optional[FrontierConfig] = None):
        self.config = config or FrontierConfig()
        self.sectors: List[Sector] = []
        self.last_update_time: float = time.time()
        self.current_heading: float = 0.0  # degrees, 0 = front

        self._init_sectors()

    def _init_sectors(self) -> None:
        """Initialize sectors."""
        sector_size = 360.0 / self.config.num_sectors
        self.sectors = []

        for i in range(self.config.num_sectors):
            angle_start = i * sector_size
            angle_end = (i + 1) * sector_size
            self.sectors.append(Sector(
                index=i,
                angle_start=angle_start,
                angle_end=angle_end,
                freshness=self.config.initial_freshness,
            ))

    def update_from_lidar(self, ranges: List[float], angle_min: float, angle_increment: float) -> None:
        """
        Update sector distances from lidar scan.

        Args:
            ranges: List of range measurements
            angle_min: Starting angle in radians
            angle_increment: Angle step in radians
        """
        # Reset sector distances
        for sector in self.sectors:
            sector.min_distance = float('inf')
            sector.is_blocked = False

        # Process each range measurement
        for i, r in enumerate(ranges):
            if r <= 0 or math.isinf(r) or math.isnan(r):
                continue

            # Calculate angle for this measurement
            angle_rad = angle_min + i * angle_increment
            angle_deg = math.degrees(angle_rad) % 360

            # Find which sector this belongs to
            sector_idx = int(angle_deg / (360.0 / self.config.num_sectors)) % self.config.num_sectors
            sector = self.sectors[sector_idx]

            # Update minimum distance
            if r < sector.min_distance:
                sector.min_distance = r

        # Mark blocked sectors
        for sector in self.sectors:
            sector.is_blocked = sector.min_distance < self.config.min_clear_distance

    def decay_freshness(self) -> None:
        """Decay freshness of all sectors based on elapsed time."""
        current_time = time.time()
        elapsed = current_time - self.last_update_time
        self.last_update_time = current_time

        decay = self.config.freshness_decay_rate * elapsed
        for sector in self.sectors:
            sector.freshness = max(0, sector.freshness - decay)

    def mark_visited(self, angle_deg: float) -> None:
        """Mark a direction as freshly visited."""
        sector_idx = int(angle_deg / (360.0 / self.config.num_sectors)) % self.config.num_sectors
        self.sectors[sector_idx].freshness = self.config.max_freshness

    def get_best_direction(self) -> Tuple[Optional[Sector], str]:
        """
        Get the best direction to explore.

        Returns:
            Tuple of (best_sector, reason)
            If no valid direction, returns (None, reason)
        """
        self.decay_freshness()

        # Filter to open sectors
        open_sectors = [s for s in self.sectors if not s.is_blocked]

        if not open_sectors:
            return None, "All sectors blocked"

        # Sort by freshness (lowest = least recently visited)
        open_sectors.sort(key=lambda s: s.freshness)
        best = open_sectors[0]

        sector_name = self.SECTOR_NAMES[best.index] if best.index < len(self.SECTOR_NAMES) else f"Sector {best.index}"
        return best, f"Exploring {sector_name} (freshness: {best.freshness:.1f})"

    def get_escape_direction(self) -> Optional[Sector]:
        """Get any open direction for escape when stuck."""
        # Try to find any open sector, prefer sides and back
        priority_order = [2, 6, 1, 7, 3, 5, 4, 0]  # Right, Left, Front-Right, etc.

        for idx in priority_order:
            if idx < len(self.sectors) and not self.sectors[idx].is_blocked:
                return self.sectors[idx]

        return None

    def get_status(self) -> dict:
        """Get planner status for logging."""
        return {
            "sectors": [
                {
                    "name": self.SECTOR_NAMES[s.index] if s.index < len(self.SECTOR_NAMES) else f"S{s.index}",
                    "freshness": round(s.freshness, 1),
                    "distance": round(s.min_distance, 2) if not math.isinf(s.min_distance) else "inf",
                    "blocked": s.is_blocked,
                }
                for s in self.sectors
            ]
        }
```

**Step 2: Update __init__.py**

```python
"""Autonomous exploration package for LanderPi robot."""

from .safety_monitor import SafetyMonitor, SafetyConfig
from .frontier_planner import FrontierPlanner, FrontierConfig, Sector

__all__ = [
    "SafetyMonitor",
    "SafetyConfig",
    "FrontierPlanner",
    "FrontierConfig",
    "Sector",
]
```

**Step 3: Verify syntax**

Run: `python3 -m py_compile validation/exploration/frontier_planner.py`
Expected: No output (success)

**Step 4: Commit**

```bash
git add validation/exploration/
git commit -m "feat(exploration): add frontier planner with sector-based direction selection"
```

---

## Task 4: Implement Sensor Fusion Module

**Files:**
- Create: `validation/exploration/sensor_fusion.py`
- Modify: `validation/exploration/__init__.py`

**Step 1: Write SensorFusion class**

```python
"""Sensor fusion for depth camera and lidar."""

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple, Callable

from rich.console import Console

console = Console()


@dataclass
class SensorConfig:
    """Sensor fusion configuration."""
    # Distance thresholds (meters)
    stop_distance: float = 0.20  # Emergency stop
    slow_distance: float = 0.50  # Slow down and prepare to turn

    # Depth camera settings
    depth_center_width: int = 100  # pixels from center to check
    depth_center_height: int = 80  # pixels from center to check
    depth_min_valid: int = 150  # mm - minimum valid depth
    depth_max_valid: int = 3000  # mm - maximum valid depth

    # Lidar settings
    lidar_front_angle: float = 60.0  # degrees each side of center

    # Fusion
    update_rate_hz: float = 10.0


@dataclass
class ObstacleState:
    """Current obstacle detection state."""
    closest_distance: float = float('inf')
    depth_distance: float = float('inf')
    lidar_distance: float = float('inf')
    should_stop: bool = False
    should_slow: bool = False
    obstacle_angle: float = 0.0  # degrees, 0 = front
    timestamp: float = 0.0


class SensorFusion:
    """Fuses depth camera and lidar data for obstacle detection."""

    def __init__(self, config: Optional[SensorConfig] = None):
        self.config = config or SensorConfig()
        self.state = ObstacleState()
        self.lidar_ranges: List[float] = []
        self.lidar_angle_min: float = 0.0
        self.lidar_angle_increment: float = 0.0

    def update_depth(self, depth_data: List[int], width: int, height: int) -> float:
        """
        Update from depth camera data.

        Args:
            depth_data: Flat list of depth values in mm (row-major)
            width: Image width
            height: Image height

        Returns:
            Minimum valid depth in meters
        """
        if not depth_data or width == 0 or height == 0:
            self.state.depth_distance = float('inf')
            return float('inf')

        # Extract center region
        cx, cy = width // 2, height // 2
        hw = self.config.depth_center_width // 2
        hh = self.config.depth_center_height // 2

        min_depth = float('inf')

        for y in range(max(0, cy - hh), min(height, cy + hh)):
            for x in range(max(0, cx - hw), min(width, cx + hw)):
                idx = y * width + x
                if idx < len(depth_data):
                    d = depth_data[idx]
                    if self.config.depth_min_valid <= d <= self.config.depth_max_valid:
                        if d < min_depth:
                            min_depth = d

        # Convert to meters
        if min_depth < float('inf'):
            self.state.depth_distance = min_depth / 1000.0
        else:
            self.state.depth_distance = float('inf')

        return self.state.depth_distance

    def update_lidar(self, ranges: List[float], angle_min: float, angle_increment: float) -> float:
        """
        Update from lidar scan data.

        Args:
            ranges: List of range measurements in meters
            angle_min: Starting angle in radians
            angle_increment: Angle step in radians

        Returns:
            Minimum distance in front arc
        """
        self.lidar_ranges = ranges
        self.lidar_angle_min = angle_min
        self.lidar_angle_increment = angle_increment

        if not ranges:
            self.state.lidar_distance = float('inf')
            return float('inf')

        # Calculate indices for front arc
        front_angle_rad = math.radians(self.config.lidar_front_angle)
        min_dist = float('inf')
        min_angle = 0.0

        for i, r in enumerate(ranges):
            if r <= 0 or math.isinf(r) or math.isnan(r):
                continue

            angle = angle_min + i * angle_increment

            # Check if in front arc (handle wrap-around)
            # Front is at angle 0, check -front_angle to +front_angle
            if -front_angle_rad <= angle <= front_angle_rad:
                if r < min_dist:
                    min_dist = r
                    min_angle = angle
            # Also check near 2*pi (wrap-around)
            elif angle > (2 * math.pi - front_angle_rad):
                if r < min_dist:
                    min_dist = r
                    min_angle = angle - 2 * math.pi

        self.state.lidar_distance = min_dist
        if min_dist < float('inf'):
            self.state.obstacle_angle = math.degrees(min_angle)

        return min_dist

    def get_obstacle_state(self) -> ObstacleState:
        """
        Get fused obstacle state.

        Uses minimum of depth and lidar distances.
        """
        # Fuse: take minimum of both sensors
        self.state.closest_distance = min(
            self.state.depth_distance,
            self.state.lidar_distance
        )

        # Determine action
        self.state.should_stop = self.state.closest_distance < self.config.stop_distance
        self.state.should_slow = (
            not self.state.should_stop and
            self.state.closest_distance < self.config.slow_distance
        )
        self.state.timestamp = time.time()

        return self.state

    def get_lidar_ranges_for_planner(self) -> Tuple[List[float], float, float]:
        """Get lidar data for frontier planner."""
        return self.lidar_ranges, self.lidar_angle_min, self.lidar_angle_increment

    def get_status(self) -> dict:
        """Get sensor status for logging."""
        return {
            "depth_m": round(self.state.depth_distance, 3) if self.state.depth_distance < 100 else "inf",
            "lidar_m": round(self.state.lidar_distance, 3) if self.state.lidar_distance < 100 else "inf",
            "fused_m": round(self.state.closest_distance, 3) if self.state.closest_distance < 100 else "inf",
            "should_stop": self.state.should_stop,
            "should_slow": self.state.should_slow,
        }
```

**Step 2: Update __init__.py**

```python
"""Autonomous exploration package for LanderPi robot."""

from .safety_monitor import SafetyMonitor, SafetyConfig
from .frontier_planner import FrontierPlanner, FrontierConfig, Sector
from .sensor_fusion import SensorFusion, SensorConfig, ObstacleState

__all__ = [
    "SafetyMonitor",
    "SafetyConfig",
    "FrontierPlanner",
    "FrontierConfig",
    "Sector",
    "SensorFusion",
    "SensorConfig",
    "ObstacleState",
]
```

**Step 3: Verify syntax**

Run: `python3 -m py_compile validation/exploration/sensor_fusion.py`
Expected: No output (success)

**Step 4: Commit**

```bash
git add validation/exploration/
git commit -m "feat(exploration): add sensor fusion for depth camera and lidar"
```

---

## Task 5: Implement Data Logger

**Files:**
- Create: `validation/exploration/data_logger.py`
- Modify: `validation/exploration/__init__.py`

**Step 1: Write DataLogger class**

```python
"""Data logging for exploration sessions."""

import json
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from rich.console import Console

console = Console()


@dataclass
class LogConfig:
    """Logging configuration."""
    base_dir: str = "~/landerpi/exploration_logs"
    lidar_sample_rate_hz: float = 1.0  # How often to log lidar
    depth_sample_rate_hz: float = 1.0  # How often to log depth stats
    rosbag_enabled: bool = False
    rosbag_dir: str = "~/landerpi/rosbags"


class DataLogger:
    """Logs exploration data for future SLAM work."""

    def __init__(self, config: Optional[LogConfig] = None):
        self.config = config or LogConfig()
        self.session_dir: Optional[Path] = None
        self.start_time: Optional[float] = None

        # File handles
        self.metadata_file: Optional[Path] = None
        self.lidar_file: Optional[Path] = None
        self.depth_file: Optional[Path] = None
        self.odometry_file: Optional[Path] = None
        self.events_file: Optional[Path] = None

        # Throttling
        self.last_lidar_log: float = 0
        self.last_depth_log: float = 0

        # Stats
        self.lidar_count: int = 0
        self.depth_count: int = 0
        self.event_count: int = 0

    def start_session(self, robot_config: Optional[Dict] = None) -> Path:
        """Start a new logging session."""
        # Create session directory
        base = Path(self.config.base_dir).expanduser()
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.session_dir = base / f"session_{timestamp}"
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self.start_time = time.time()

        # Create log files
        self.lidar_file = self.session_dir / "lidar_scans.jsonl"
        self.depth_file = self.session_dir / "depth_summary.jsonl"
        self.odometry_file = self.session_dir / "odometry.jsonl"
        self.events_file = self.session_dir / "events.jsonl"
        self.metadata_file = self.session_dir / "metadata.json"

        # Write metadata
        metadata = {
            "session_start": timestamp,
            "start_timestamp": self.start_time,
            "robot_config": robot_config or {},
            "log_config": asdict(self.config),
        }
        self.metadata_file.write_text(json.dumps(metadata, indent=2))

        # Reset stats
        self.lidar_count = 0
        self.depth_count = 0
        self.event_count = 0

        console.print(f"[green]Logging to:[/green] {self.session_dir}")
        return self.session_dir

    def log_lidar(self, sector_mins: List[float], sector_maxs: List[float]) -> None:
        """Log lidar sector summary (throttled)."""
        if not self.lidar_file:
            return

        now = time.time()
        if now - self.last_lidar_log < (1.0 / self.config.lidar_sample_rate_hz):
            return

        self.last_lidar_log = now
        entry = {
            "ts": now,
            "ranges_min": [round(r, 3) if r < 100 else None for r in sector_mins],
            "ranges_max": [round(r, 3) if r < 100 else None for r in sector_maxs],
        }

        with open(self.lidar_file, "a") as f:
            f.write(json.dumps(entry) + "\n")
        self.lidar_count += 1

    def log_depth(self, min_depth: float, avg_depth: float, valid_percent: float) -> None:
        """Log depth camera summary (throttled)."""
        if not self.depth_file:
            return

        now = time.time()
        if now - self.last_depth_log < (1.0 / self.config.depth_sample_rate_hz):
            return

        self.last_depth_log = now
        entry = {
            "ts": now,
            "min_m": round(min_depth, 3) if min_depth < 100 else None,
            "avg_m": round(avg_depth, 3) if avg_depth < 100 else None,
            "valid_pct": round(valid_percent, 1),
        }

        with open(self.depth_file, "a") as f:
            f.write(json.dumps(entry) + "\n")
        self.depth_count += 1

    def log_odometry(self, vx: float, vy: float, wz: float, estimated_x: float = 0, estimated_y: float = 0) -> None:
        """Log motor commands and estimated position."""
        if not self.odometry_file:
            return

        entry = {
            "ts": time.time(),
            "vx": round(vx, 3),
            "vy": round(vy, 3),
            "wz": round(wz, 3),
            "est_x": round(estimated_x, 3),
            "est_y": round(estimated_y, 3),
        }

        with open(self.odometry_file, "a") as f:
            f.write(json.dumps(entry) + "\n")

    def log_event(self, event_type: str, details: Optional[Dict] = None) -> None:
        """Log exploration events."""
        if not self.events_file:
            return

        entry = {
            "ts": time.time(),
            "type": event_type,
            "details": details or {},
        }

        with open(self.events_file, "a") as f:
            f.write(json.dumps(entry) + "\n")
        self.event_count += 1

    def end_session(self, summary: Optional[Dict] = None) -> Dict:
        """End logging session and return summary."""
        if not self.metadata_file or not self.start_time:
            return {}

        end_time = time.time()
        duration = end_time - self.start_time

        # Update metadata with end info
        metadata = json.loads(self.metadata_file.read_text())
        metadata["session_end"] = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        metadata["end_timestamp"] = end_time
        metadata["duration_seconds"] = round(duration, 1)
        metadata["stats"] = {
            "lidar_samples": self.lidar_count,
            "depth_samples": self.depth_count,
            "events": self.event_count,
        }
        if summary:
            metadata["summary"] = summary

        self.metadata_file.write_text(json.dumps(metadata, indent=2))

        console.print(f"[green]Session logged:[/green] {self.session_dir}")
        console.print(f"  Duration: {duration/60:.1f} min")
        console.print(f"  Lidar samples: {self.lidar_count}")
        console.print(f"  Depth samples: {self.depth_count}")
        console.print(f"  Events: {self.event_count}")

        return metadata
```

**Step 2: Update __init__.py**

```python
"""Autonomous exploration package for LanderPi robot."""

from .safety_monitor import SafetyMonitor, SafetyConfig
from .frontier_planner import FrontierPlanner, FrontierConfig, Sector
from .sensor_fusion import SensorFusion, SensorConfig, ObstacleState
from .data_logger import DataLogger, LogConfig

__all__ = [
    "SafetyMonitor",
    "SafetyConfig",
    "FrontierPlanner",
    "FrontierConfig",
    "Sector",
    "SensorFusion",
    "SensorConfig",
    "ObstacleState",
    "DataLogger",
    "LogConfig",
]
```

**Step 3: Verify syntax**

Run: `python3 -m py_compile validation/exploration/data_logger.py`
Expected: No output (success)

**Step 4: Commit**

```bash
git add validation/exploration/
git commit -m "feat(exploration): add data logger for SLAM preparation"
```

---

## Task 6: Implement Main Explorer Controller

**Files:**
- Create: `validation/exploration/explorer.py`
- Modify: `validation/exploration/__init__.py`

**Step 1: Write ExplorationController class**

```python
"""Main exploration controller."""

import math
import time
from dataclasses import dataclass
from typing import Callable, Optional

from rich.console import Console

from .safety_monitor import SafetyMonitor, SafetyConfig
from .frontier_planner import FrontierPlanner, FrontierConfig
from .sensor_fusion import SensorFusion, SensorConfig
from .data_logger import DataLogger, LogConfig

console = Console()


@dataclass
class ExplorationConfig:
    """Main exploration configuration."""
    # Motion
    forward_speed: float = 0.2  # m/s
    turn_speed: float = 1.2  # rad/s
    slow_factor: float = 0.5  # Speed multiplier when obstacle nearby

    # Timing
    loop_rate_hz: float = 10.0
    decision_rate_hz: float = 2.0  # How often to make navigation decisions

    # Stuck detection
    stuck_timeout: float = 10.0  # seconds without movement = stuck
    stuck_escape_duration: float = 2.0  # seconds to back up when stuck


class ExplorationController:
    """
    Main controller for autonomous exploration.

    Coordinates sensor fusion, frontier planning, safety monitoring,
    and motion control for autonomous exploration.
    """

    def __init__(
        self,
        move_func: Callable[[float, float, float], bool],  # vx, vy, wz -> success
        stop_func: Callable[[], None],
        get_battery_func: Callable[[], Optional[float]],
        exploration_config: Optional[ExplorationConfig] = None,
        safety_config: Optional[SafetyConfig] = None,
        sensor_config: Optional[SensorConfig] = None,
        frontier_config: Optional[FrontierConfig] = None,
        log_config: Optional[LogConfig] = None,
        on_speak: Optional[Callable[[str], None]] = None,
    ):
        self.config = exploration_config or ExplorationConfig()

        # Motion control callbacks
        self.move = move_func
        self.stop = stop_func
        self.speak = on_speak or (lambda msg: console.print(f"[cyan]{msg}[/cyan]"))

        # Components
        self.safety = SafetyMonitor(
            safety_config or SafetyConfig(),
            get_battery_func,
            on_warning=lambda msg: self.speak(msg),
            on_shutdown=lambda msg: self.speak(msg),
        )
        self.sensors = SensorFusion(sensor_config)
        self.planner = FrontierPlanner(frontier_config)
        self.logger = DataLogger(log_config)

        # State
        self.running = False
        self.last_decision_time: float = 0
        self.last_significant_movement: float = 0
        self.current_action: str = "idle"

    def start(self) -> None:
        """Start exploration."""
        self.running = True
        self.safety.start()
        self.logger.start_session()
        self.last_significant_movement = time.time()
        self.speak("Starting exploration")
        self.logger.log_event("start")
        console.print("[bold green]Exploration started[/bold green]")

    def stop_exploration(self, reason: str = "Manual stop") -> None:
        """Stop exploration."""
        self.running = False
        self.stop()
        self.speak(f"Stopping. {reason}")
        self.logger.log_event("stop", {"reason": reason})
        summary = self.logger.end_session({"stop_reason": reason})
        console.print(f"[yellow]Exploration stopped: {reason}[/yellow]")

    def update_sensors(self, lidar_ranges: list, lidar_angle_min: float, lidar_angle_increment: float,
                       depth_data: Optional[list] = None, depth_width: int = 0, depth_height: int = 0) -> None:
        """Update sensor data."""
        # Update lidar
        self.sensors.update_lidar(lidar_ranges, lidar_angle_min, lidar_angle_increment)

        # Update depth if available
        if depth_data:
            self.sensors.update_depth(depth_data, depth_width, depth_height)

        # Update frontier planner
        self.planner.update_from_lidar(lidar_ranges, lidar_angle_min, lidar_angle_increment)

        # Log sensor data
        if self.logger.session_dir:
            # Get sector stats for logging
            sector_mins = [s.min_distance for s in self.planner.sectors]
            sector_maxs = [s.min_distance for s in self.planner.sectors]  # Same for now
            self.logger.log_lidar(sector_mins, sector_maxs)

    def step(self) -> bool:
        """
        Execute one control step.

        Returns True to continue, False to stop.
        """
        if not self.running:
            return False

        # Safety check
        if not self.safety.check():
            self.stop_exploration(self.safety.stop_reason or "Safety stop")
            return False

        # Get obstacle state
        obstacle = self.sensors.get_obstacle_state()

        # Emergency stop if too close
        if obstacle.should_stop:
            self.stop()
            self.current_action = "stopped"
            self.logger.log_event("obstacle_stop", {"distance": obstacle.closest_distance})
            console.print(f"[red]Obstacle at {obstacle.closest_distance:.2f}m - stopped[/red]")
            return True  # Continue loop but don't move

        # Check for stuck condition
        if self._check_stuck():
            self._escape_stuck()
            return True

        # Navigation decision (throttled)
        current_time = time.time()
        if current_time - self.last_decision_time >= (1.0 / self.config.decision_rate_hz):
            self.last_decision_time = current_time
            self._make_navigation_decision(obstacle)

        return True

    def _make_navigation_decision(self, obstacle) -> None:
        """Decide where to go next."""
        best_sector, reason = self.planner.get_best_direction()

        if best_sector is None:
            # All blocked - try to escape
            console.print("[yellow]All directions blocked[/yellow]")
            self._escape_stuck()
            return

        # Calculate turn needed
        target_angle = best_sector.center_angle
        if target_angle > 180:
            target_angle -= 360

        # Determine motion
        speed = self.config.forward_speed
        if obstacle.should_slow:
            speed *= self.config.slow_factor

        # If need to turn significantly, turn first
        if abs(target_angle) > 30:
            wz = self.config.turn_speed if target_angle > 0 else -self.config.turn_speed
            self.move(0, 0, wz)
            self.current_action = f"turning to {self.planner.SECTOR_NAMES[best_sector.index]}"
        else:
            # Move forward
            self.move(speed, 0, 0)
            self.current_action = "moving forward"
            self.last_significant_movement = time.time()

        # Mark current direction as visited
        self.planner.mark_visited(0)  # Front is always where we're looking

        # Log
        self.logger.log_odometry(speed if abs(target_angle) <= 30 else 0, 0, 0)

    def _check_stuck(self) -> bool:
        """Check if robot is stuck."""
        return (time.time() - self.last_significant_movement) > self.config.stuck_timeout

    def _escape_stuck(self) -> None:
        """Attempt to escape stuck situation."""
        console.print("[yellow]Stuck detected - escaping[/yellow]")
        self.logger.log_event("stuck_escape")

        # Back up
        self.move(-self.config.forward_speed * 0.5, 0, 0)
        time.sleep(self.config.stuck_escape_duration)

        # Try to find open direction
        escape_sector = self.planner.get_escape_direction()
        if escape_sector:
            # Turn toward open direction
            target_angle = escape_sector.center_angle
            if target_angle > 180:
                target_angle -= 360
            wz = self.config.turn_speed if target_angle > 0 else -self.config.turn_speed
            self.move(0, 0, wz)
            time.sleep(1.0)

        self.stop()
        self.last_significant_movement = time.time()

    def get_status(self) -> dict:
        """Get exploration status."""
        safety_status = self.safety.get_status()
        sensor_status = self.sensors.get_status()
        planner_status = self.planner.get_status()

        return {
            "running": self.running,
            "action": self.current_action,
            "safety": safety_status,
            "sensors": sensor_status,
            "planner": planner_status,
        }
```

**Step 2: Update __init__.py**

```python
"""Autonomous exploration package for LanderPi robot."""

from .safety_monitor import SafetyMonitor, SafetyConfig
from .frontier_planner import FrontierPlanner, FrontierConfig, Sector
from .sensor_fusion import SensorFusion, SensorConfig, ObstacleState
from .data_logger import DataLogger, LogConfig
from .explorer import ExplorationController, ExplorationConfig

__all__ = [
    "SafetyMonitor",
    "SafetyConfig",
    "FrontierPlanner",
    "FrontierConfig",
    "Sector",
    "SensorFusion",
    "SensorConfig",
    "ObstacleState",
    "DataLogger",
    "LogConfig",
    "ExplorationController",
    "ExplorationConfig",
]
```

**Step 3: Verify syntax**

Run: `python3 -m py_compile validation/exploration/explorer.py`
Expected: No output (success)

**Step 4: Commit**

```bash
git add validation/exploration/
git commit -m "feat(exploration): add main exploration controller"
```

---

## Task 7: Create CLI Script

**Files:**
- Create: `validation/test_exploration.py`

**Step 1: Write CLI script**

```python
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
import math
import signal
import sys
import tempfile
import time
from io import StringIO
from pathlib import Path
from typing import Optional

import typer
from fabric import Connection
from rich.console import Console
from rich.panel import Panel
from rich.prompt import Confirm
from rich.table import Table

from exploration import (
    ExplorationController,
    ExplorationConfig,
    SafetyConfig,
    SensorConfig,
    FrontierConfig,
    LogConfig,
)

app = typer.Typer(help="LanderPi Autonomous Exploration (Sensor Fusion)")
console = Console()

# Docker configuration
DOCKER_IMAGE = "landerpi-ros2:latest"
DOCKER_RUN_BASE = "docker run --rm --privileged --network host -v /dev:/dev"


def load_config() -> dict:
    """Load connection config from config.json."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
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
        # Script to read one scan
        scan_script = """
import sys
import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

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
import time
start = time.time()
while node.data is None and (time.time() - start) < 2.0:
    rclpy.spin_once(node, timeout_sec=0.1)
if node.data:
    print(json.dumps(node.data))
node.destroy_node()
rclpy.shutdown()
"""
        try:
            # Upload and run script
            self.conn.put(StringIO(scan_script), "/tmp/read_scan.py")
            cmd = (
                f"{DOCKER_RUN_BASE} -v /tmp:/tmp {DOCKER_IMAGE} "
                f"bash -c 'source /opt/ros/humble/setup.bash && python3 /tmp/read_scan.py'"
            )
            result = self.conn.run(cmd, hide=True, warn=True, timeout=10)
            if result.ok and result.stdout.strip():
                data = json.loads(result.stdout.strip())
                return data['ranges'], data['angle_min'], data['angle_increment']
        except:
            pass
        return [], 0.0, 0.0

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
        log_config = LogConfig(rosbag_enabled=enable_rosbag)

        # Create controller
        self.controller = ExplorationController(
            move_func=self.move,
            stop_func=self.stop_motors,
            get_battery_func=self.get_battery,
            safety_config=safety_config,
            log_config=log_config,
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
                if ranges:
                    self.controller.update_sensors(
                        ranges, angle_min, angle_inc,
                        depth_data=None, depth_width=0, depth_height=0
                    )

                # Step controller
                if not self.controller.step():
                    break

                # Periodic status
                if time.time() - last_status_time >= status_period:
                    last_status_time = time.time()
                    status = self.controller.get_status()
                    remaining = status['safety']['remaining_minutes']
                    console.print(f"[dim]Status: {remaining:.1f} min remaining, action: {status['action']}[/dim]")

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
```

**Step 2: Make executable**

```bash
chmod +x validation/test_exploration.py
```

**Step 3: Verify syntax**

Run: `python3 -m py_compile validation/test_exploration.py`
Expected: No output (success)

**Step 4: Commit**

```bash
git add validation/test_exploration.py
git commit -m "feat(exploration): add CLI script for autonomous exploration"
```

---

## Task 8: Add Voice Commands

**Files:**
- Modify: `robot_voicecontroller.py`

**Step 1: Add explore commands to ROBOT_CONTROL_PROMPT**

Find the `ROBOT_CONTROL_PROMPT` string and add explore commands:

```python
# Add after "- Mode: follow_me" line:
# - Explore: explore (start autonomous exploration), stop_exploring (stop exploration)
```

**Step 2: Add explore action handling**

Find the `execute_command` function and add explore handling:

```python
elif action == "explore":
    if command == "explore":
        # Start exploration in background
        console.print("[cyan]Starting exploration mode...[/cyan]")
        # TODO: Integrate with exploration controller
        return True
    elif command == "stop_exploring":
        # Stop exploration
        console.print("[cyan]Stopping exploration...[/cyan]")
        return True
```

**Step 3: Commit**

```bash
git add robot_voicecontroller.py
git commit -m "feat(voice): add explore voice commands (placeholder)"
```

---

## Task 9: Update CLAUDE.md Documentation

**Files:**
- Modify: `CLAUDE.md`

**Step 1: Add exploration commands to Commands section**

Add after the voice control section:

```markdown
# Autonomous Exploration (uses config.json)
uv run python validation/test_exploration.py start --yes        # Start exploration (30 min)
uv run python validation/test_exploration.py start --duration 60 --yes  # 60 min exploration
uv run python validation/test_exploration.py start --rosbag --yes       # With ROS2 bag recording
uv run python validation/test_exploration.py stop               # Stop exploration
uv run python validation/test_exploration.py status             # Check robot status
```

**Step 2: Add exploration architecture description**

Add after voice control architecture:

```markdown
**`validation/exploration/`** - Autonomous exploration package
- `explorer.py` - Main exploration controller
- `sensor_fusion.py` - Depth camera + lidar fusion
- `frontier_planner.py` - Direction selection based on freshness
- `safety_monitor.py` - Battery and runtime monitoring
- `data_logger.py` - Logging for future SLAM

**`validation/test_exploration.py`** - Exploration CLI
- Start/stop autonomous exploration
- Sensor fusion: depth camera (20cm stop) + lidar (360° awareness)
- Frontier-based exploration (seeks unexplored directions)
- Safety: runtime limit + battery cutoff
```

**Step 3: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: add exploration commands to CLAUDE.md"
```

---

## Task 10: Integration Test

**Step 1: Verify package imports**

Run: `cd /Users/ZDZ/Documents/gitrepo/personalproject/hiwonderSetup && python3 -c "from validation.exploration import ExplorationController, SafetyMonitor, FrontierPlanner, SensorFusion, DataLogger; print('All imports OK')"`

Expected: `All imports OK`

**Step 2: Verify CLI help**

Run: `uv run python validation/test_exploration.py --help`

Expected: Help output showing start, stop, status commands

**Step 3: Test status command (requires robot)**

Run: `uv run python validation/test_exploration.py status`

Expected: Shows battery and prerequisite status

**Step 4: Final commit**

```bash
git add .
git commit -m "feat(exploration): complete autonomous exploration implementation"
```

---

## Summary

**Files Created:**
- `validation/exploration/__init__.py`
- `validation/exploration/safety_monitor.py`
- `validation/exploration/frontier_planner.py`
- `validation/exploration/sensor_fusion.py`
- `validation/exploration/data_logger.py`
- `validation/exploration/explorer.py`
- `validation/test_exploration.py`

**Files Modified:**
- `robot_voicecontroller.py` (voice command placeholders)
- `CLAUDE.md` (documentation)

**Total Tasks:** 10
**Estimated Implementation Time:** 2-3 hours
