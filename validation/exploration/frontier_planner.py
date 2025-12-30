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
    min_clear_distance: float = 0.40  # meters - sector considered blocked if closer


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
