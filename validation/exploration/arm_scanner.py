"""Arm-mounted depth camera scanner for escape direction finding."""

import time
from dataclasses import dataclass
from typing import Callable, List, Optional

from rich.console import Console

console = Console()


@dataclass
class DepthReading:
    """Depth reading at a specific arm angle."""
    angle: float          # Degrees from center (-60 to +60)
    min_depth: float      # Closest point (meters)
    avg_depth: float      # Average depth (meters)
    valid_percent: float  # Percentage of valid readings


@dataclass
class ArmScannerConfig:
    """Arm scanner configuration."""
    # Servo positions (servo 1 = base rotation)
    servo_right: int = 200   # Looking right
    servo_center: int = 500  # Looking forward
    servo_left: int = 800    # Looking left

    # Timing
    settle_time: float = 0.4  # Time to wait after moving arm (seconds)
    move_duration: float = 0.8  # Duration for arm movement

    # Depth thresholds
    min_valid_depth: float = 0.15  # meters
    max_valid_depth: float = 3.0   # meters
    min_clearance: float = 1.0     # meters - minimum to consider "open"


class ArmScanner:
    """
    Scans for escape directions using the arm-mounted depth camera.

    The depth camera is mounted on servo 1 (base rotation), allowing
    ~180° field of view by panning the arm.
    """

    def __init__(
        self,
        arm_move_func: Callable[[int, int, float], bool],  # servo_id, position, duration
        get_depth_func: Callable[[], Optional[dict]],  # Returns depth stats
        config: Optional[ArmScannerConfig] = None,
    ):
        """
        Args:
            arm_move_func: Function to move arm servo. Args: servo_id, position, duration
            get_depth_func: Function to get current depth reading. Returns dict with
                           'min_depth', 'avg_depth', 'valid_percent' keys (depths in meters)
        """
        self.arm_move = arm_move_func
        self.get_depth = get_depth_func
        self.config = config or ArmScannerConfig()

    def quick_glance(self, direction: str) -> float:
        """
        Quick look left or right.

        Args:
            direction: "left" or "right"

        Returns:
            Average depth in that direction (meters), 0 if failed
        """
        target = self.config.servo_left if direction == "left" else self.config.servo_right

        try:
            # Move arm
            self.arm_move(1, target, self.config.move_duration)
            time.sleep(self.config.settle_time)

            # Read depth
            depth_data = self.get_depth()

            # Return to center
            self.arm_move(1, self.config.servo_center, self.config.move_duration)

            if depth_data and depth_data.get("avg_depth"):
                return depth_data["avg_depth"]
            return 0.0

        except Exception as e:
            console.print(f"[dim]Quick glance failed: {e}[/dim]")
            # Try to return to center
            try:
                self.arm_move(1, self.config.servo_center, self.config.move_duration)
            except:
                pass
            return 0.0

    def full_sweep(self) -> List[DepthReading]:
        """
        Sweep arm across 180° field, collecting depth readings.

        Returns:
            List of DepthReading objects at different angles
        """
        readings = []

        # Sample positions: right to left
        # Servo range 200-800, angles approximately -60° to +60° from center
        positions = [200, 300, 400, 500, 600, 700, 800]
        angles = [-60, -40, -20, 0, 20, 40, 60]

        console.print("[dim]Starting arm sweep...[/dim]")

        try:
            for pos, angle in zip(positions, angles):
                # Move arm
                self.arm_move(1, pos, self.config.move_duration)
                time.sleep(self.config.settle_time)

                # Read depth
                depth_data = self.get_depth()

                if depth_data:
                    reading = DepthReading(
                        angle=angle,
                        min_depth=depth_data.get("min_depth", 0),
                        avg_depth=depth_data.get("avg_depth", 0),
                        valid_percent=depth_data.get("valid_percent", 0),
                    )
                    readings.append(reading)
                    console.print(
                        f"[dim]  {angle:+3d}°: avg={reading.avg_depth:.2f}m, "
                        f"min={reading.min_depth:.2f}m[/dim]"
                    )
                else:
                    console.print(f"[dim]  {angle:+3d}°: no data[/dim]")

        finally:
            # Always return to center
            console.print("[dim]Returning arm to center[/dim]")
            self.arm_move(1, self.config.servo_center, self.config.move_duration)

        return readings

    def find_best_opening(self, readings: List[DepthReading]) -> Optional[float]:
        """
        Find the angle with the most open space.

        Args:
            readings: List of DepthReading from full_sweep()

        Returns:
            Angle in degrees to turn toward, or None if no good opening found
        """
        if not readings:
            return None

        # Filter to readings with sufficient clearance
        valid_readings = [
            r for r in readings
            if r.avg_depth >= self.config.min_clearance
            and r.valid_percent > 30  # At least 30% valid depth pixels
        ]

        if not valid_readings:
            console.print("[dim]No openings with sufficient clearance[/dim]")
            return None

        # Find the one with greatest average depth
        best = max(valid_readings, key=lambda r: r.avg_depth)

        console.print(
            f"[dim]Best opening: {best.angle}° with {best.avg_depth:.2f}m clearance[/dim]"
        )

        return best.angle

    def get_status(self) -> dict:
        """Get scanner status."""
        return {
            "config": {
                "servo_range": f"{self.config.servo_right}-{self.config.servo_left}",
                "settle_time": self.config.settle_time,
                "min_clearance": self.config.min_clearance,
            }
        }
