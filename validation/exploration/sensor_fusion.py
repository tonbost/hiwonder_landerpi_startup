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
