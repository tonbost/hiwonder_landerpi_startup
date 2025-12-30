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
