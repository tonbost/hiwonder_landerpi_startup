"""Remote data logging for exploration sessions - writes to robot via SSH."""

import json
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from typing import Any, Dict, List, Optional, Callable

from rich.console import Console

console = Console()


@dataclass
class RemoteLogConfig:
    """Remote logging configuration."""
    base_dir: str = "~/landerpi/exploration_logs"
    lidar_sample_rate_hz: float = 1.0
    depth_sample_rate_hz: float = 1.0
    rosbag_enabled: bool = False
    rosbag_dir: str = "~/landerpi/rosbags"


class RemoteDataLogger:
    """Logs exploration data to the robot via SSH."""

    def __init__(
        self,
        config: Optional[RemoteLogConfig] = None,
        run_command: Optional[Callable[[str], tuple]] = None,
    ):
        """
        Initialize remote logger.

        Args:
            config: Logging configuration
            run_command: Function to execute commands on robot.
                         Should return (success: bool, output: str)
        """
        self.config = config or RemoteLogConfig()
        self.run_cmd = run_command
        self.session_dir: Optional[str] = None
        self.start_time: Optional[float] = None

        # Throttling
        self.last_lidar_log: float = 0
        self.last_depth_log: float = 0

        # Stats (tracked locally)
        self.lidar_count: int = 0
        self.depth_count: int = 0
        self.event_count: int = 0

    def _remote_write(self, filepath: str, content: str, append: bool = False) -> bool:
        """Write content to a file on the robot."""
        if not self.run_cmd:
            return False

        # Escape content for shell
        escaped = content.replace("'", "'\\''")
        op = ">>" if append else ">"
        cmd = f"echo '{escaped}' {op} {filepath}"

        try:
            success, _ = self.run_cmd(cmd)
            return success
        except Exception:
            return False

    def _remote_mkdir(self, dirpath: str) -> bool:
        """Create directory on robot."""
        if not self.run_cmd:
            return False

        try:
            success, _ = self.run_cmd(f"mkdir -p {dirpath}")
            return success
        except Exception:
            return False

    def start_session(self, robot_config: Optional[Dict] = None) -> str:
        """Start a new logging session on the robot."""
        if not self.run_cmd:
            console.print("[yellow]No remote command runner - logging disabled[/yellow]")
            return ""

        # Create session directory on robot
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.session_dir = f"{self.config.base_dir}/session_{timestamp}"

        if not self._remote_mkdir(self.session_dir):
            console.print("[red]Failed to create session directory on robot[/red]")
            return ""

        self.start_time = time.time()

        # Write metadata
        metadata = {
            "session_start": timestamp,
            "start_timestamp": self.start_time,
            "robot_config": robot_config or {},
            "log_config": asdict(self.config),
        }
        self._remote_write(
            f"{self.session_dir}/metadata.json",
            json.dumps(metadata, indent=2)
        )

        # Reset stats
        self.lidar_count = 0
        self.depth_count = 0
        self.event_count = 0

        console.print(f"[green]Logging to robot:[/green] {self.session_dir}")
        return self.session_dir

    def log_lidar(self, sector_mins: List[float], sector_maxs: List[float]) -> None:
        """Log lidar sector summary (throttled)."""
        if not self.session_dir:
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

        self._remote_write(
            f"{self.session_dir}/lidar_scans.jsonl",
            json.dumps(entry),
            append=True
        )
        self.lidar_count += 1

    def log_depth(self, min_depth: float, avg_depth: float, valid_percent: float) -> None:
        """Log depth camera summary (throttled)."""
        if not self.session_dir:
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

        self._remote_write(
            f"{self.session_dir}/depth_summary.jsonl",
            json.dumps(entry),
            append=True
        )
        self.depth_count += 1

    def log_odometry(self, vx: float, vy: float, wz: float,
                     estimated_x: float = 0, estimated_y: float = 0) -> None:
        """Log motor commands and estimated position."""
        if not self.session_dir:
            return

        entry = {
            "ts": time.time(),
            "vx": round(vx, 3),
            "vy": round(vy, 3),
            "wz": round(wz, 3),
            "est_x": round(estimated_x, 3),
            "est_y": round(estimated_y, 3),
        }

        self._remote_write(
            f"{self.session_dir}/odometry.jsonl",
            json.dumps(entry),
            append=True
        )

    def log_event(self, event_type: str, details: Optional[Dict] = None) -> None:
        """Log exploration events."""
        if not self.session_dir:
            return

        entry = {
            "ts": time.time(),
            "type": event_type,
            "details": details or {},
        }

        self._remote_write(
            f"{self.session_dir}/events.jsonl",
            json.dumps(entry),
            append=True
        )
        self.event_count += 1

    def end_session(self, summary: Optional[Dict] = None) -> Dict:
        """End logging session and return summary."""
        if not self.session_dir or not self.start_time:
            return {}

        end_time = time.time()
        duration = end_time - self.start_time

        # Build final metadata
        metadata = {
            "session_start": datetime.fromtimestamp(self.start_time).strftime("%Y-%m-%d_%H-%M-%S"),
            "start_timestamp": self.start_time,
            "session_end": datetime.now().strftime("%Y-%m-%d_%H-%M-%S"),
            "end_timestamp": end_time,
            "duration_seconds": round(duration, 1),
            "stats": {
                "lidar_samples": self.lidar_count,
                "depth_samples": self.depth_count,
                "events": self.event_count,
            },
        }
        if summary:
            metadata["summary"] = summary

        # Overwrite metadata with final version
        self._remote_write(
            f"{self.session_dir}/metadata.json",
            json.dumps(metadata, indent=2)
        )

        console.print(f"[green]Session logged on robot:[/green] {self.session_dir}")
        console.print(f"  Duration: {duration/60:.1f} min")
        console.print(f"  Lidar samples: {self.lidar_count}")
        console.print(f"  Depth samples: {self.depth_count}")
        console.print(f"  Events: {self.event_count}")

        return metadata
