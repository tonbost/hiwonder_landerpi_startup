"""Arm-mounted depth camera scanner for escape direction finding."""

import math
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

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
    # Servo 1 positions (base rotation / pan)
    servo_right: int = 200   # Looking right
    servo_center: int = 550  # Looking forward (verified center)
    servo_left: int = 800    # Looking left

    # Scan pose - arm extended forward with camera looking horizontally
    # These positions extend the arm forward so camera faces straight ahead
    scan_pose: dict = None  # Will be set in __post_init__

    # Home pose - safe resting position
    home_pose: dict = None  # Will be set in __post_init__

    # Timing
    settle_time: float = 0.5  # Time to wait after moving arm (seconds)
    move_duration: float = 1.0  # Duration for arm movement
    pose_duration: float = 1.5  # Duration for pose changes (multi-servo)

    # Depth thresholds
    min_valid_depth: float = 0.15  # meters
    max_valid_depth: float = 3.0   # meters
    min_clearance: float = 1.0     # meters - minimum to consider "open"

    def __post_init__(self):
        # Scan pose: camera looking horizontal (same as home - camera is straight at home)
        # These values verified with camera pointing straight on the robot
        # Servo 1: base pan (controlled separately during scan)
        # Servo 2: shoulder
        # Servo 3: elbow
        # Servo 4: wrist - tilts camera up/down
        # Servo 5: wrist rotate
        if self.scan_pose is None:
            self.scan_pose = {
                2: 785,   # Shoulder: home position (camera horizontal)
                3: 0,     # Elbow: home position
                4: 350,   # Wrist: camera looking horizontal
                5: 501,   # Rotate: level
            }
        if self.home_pose is None:
            # Exploring home: arm folded back, out of the way for navigation
            self.home_pose = {
                1: 547,   # Base: slightly rotated
                2: 818,   # Shoulder: raised/folded back
                3: 203,   # Elbow: bent forward
                4: 58,    # Wrist: tilted down (arm tucked)
                5: 501,   # Rotate: centered
            }


class ArmScanner:
    """
    Scans for escape directions using the arm-mounted depth camera.

    The depth camera is mounted near the end of the arm. To scan horizontally,
    the arm must first be positioned so the camera looks straight ahead, then
    servo 1 (base) is rotated to pan left/right.
    """

    def __init__(
        self,
        arm_move_func: Callable[[int, int, float], bool],  # servo_id, position, duration
        get_depth_func: Callable[[], Optional[dict]],  # Returns depth stats
        config: Optional[ArmScannerConfig] = None,
        arm_pose_func: Optional[Callable[[list, float], bool]] = None,  # positions, duration
    ):
        """
        Args:
            arm_move_func: Function to move single arm servo. Args: servo_id, position, duration
            get_depth_func: Function to get current depth reading. Returns dict with
                           'min_depth', 'avg_depth', 'valid_percent' keys (depths in meters)
            config: Scanner configuration
            arm_pose_func: Function to move multiple servos at once. Args: [[id, pos], ...], duration
                          If not provided, will use arm_move_func sequentially.
        """
        self.arm_move = arm_move_func
        self.get_depth = get_depth_func
        self.config = config or ArmScannerConfig()
        self.arm_pose = arm_pose_func
        self._in_scan_pose = False

    def _set_pose(self, pose: dict, duration: float = None) -> bool:
        """Set arm to a specific pose (multiple servos)."""
        duration = duration or self.config.pose_duration
        positions = [[servo_id, pos] for servo_id, pos in pose.items()]

        if self.arm_pose:
            # Use batch move if available
            return self.arm_pose(positions, duration)
        else:
            # Move servos sequentially
            for servo_id, pos in pose.items():
                self.arm_move(servo_id, pos, duration)
            return True

    def enter_scan_pose(self) -> bool:
        """
        Move arm to scan pose with camera looking horizontally.

        Must be called before scanning to ensure camera isn't pointing at floor.
        """
        if self._in_scan_pose:
            return True

        console.print("[dim]Moving arm to scan pose (camera horizontal)...[/dim]")
        try:
            # First move to scan pose (shoulder, elbow, wrist)
            success = self._set_pose(self.config.scan_pose)
            time.sleep(self.config.settle_time)

            # Then center the base
            self.arm_move(1, self.config.servo_center, self.config.move_duration)
            time.sleep(self.config.settle_time)

            self._in_scan_pose = True
            console.print("[dim]Scan pose ready[/dim]")
            return success
        except Exception as e:
            console.print(f"[red]Failed to enter scan pose: {e}[/red]")
            return False

    def exit_scan_pose(self) -> bool:
        """Return arm to home/safe position."""
        console.print("[dim]Returning arm to home pose...[/dim]")
        try:
            success = self._set_pose(self.config.home_pose)
            time.sleep(self.config.settle_time)
            self._in_scan_pose = False
            return success
        except Exception as e:
            console.print(f"[red]Failed to exit scan pose: {e}[/red]")
            return False

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
            # Ensure we're in scan pose first
            if not self._in_scan_pose:
                self.enter_scan_pose()

            # Pan to target direction
            self.arm_move(1, target, self.config.move_duration)
            time.sleep(self.config.settle_time)

            # Read depth
            depth_data = self.get_depth()

            # Return to center (stay in scan pose)
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

        First enters scan pose (camera horizontal), then pans left-to-right,
        and finally returns arm to home position.

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
            # Enter scan pose first (camera looking horizontal)
            if not self._in_scan_pose:
                self.enter_scan_pose()

            for pos, angle in zip(positions, angles):
                # Pan to position (only servo 1)
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
            # Return arm to home position (safe for movement)
            self.exit_scan_pose()

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

    def chassis_360_scan(
        self,
        chassis_move_func: Callable[[float, float, float], bool],
        chassis_stop_func: Callable[[], None],
        turn_speed: float = 0.8,
        turn_time_multiplier: float = 2.5,
        num_samples: int = 8,
    ) -> Dict[float, DepthReading]:
        """
        Perform a 360° scan using chassis rotation + depth camera.

        The arm stays in scan pose (camera horizontal) while the chassis rotates,
        sampling depth at evenly spaced angles.

        Args:
            chassis_move_func: Function to move chassis (vx, vy, wz) -> success
            chassis_stop_func: Function to stop chassis
            turn_speed: Rotation speed in rad/s
            turn_time_multiplier: Carpet friction compensation multiplier
            num_samples: Number of samples (8 = every 45°)

        Returns:
            Dict mapping angle (degrees) to DepthReading at that angle.
            Angles are 0=front, 45=front-left, 90=left, etc.
        """
        readings: Dict[float, DepthReading] = {}

        # Calculate timing
        angle_per_sample = 360.0 / num_samples  # degrees
        angle_per_sample_rad = math.radians(angle_per_sample)
        # Time to rotate one sample's worth of angle, with carpet compensation
        time_per_sample = (angle_per_sample_rad / turn_speed) * turn_time_multiplier
        total_duration = time_per_sample * num_samples

        console.print(f"[dim]360° chassis scan: {num_samples} samples, {total_duration:.1f}s total[/dim]")

        try:
            # Ensure arm is in scan pose (camera horizontal)
            if not self._in_scan_pose:
                self.enter_scan_pose()

            # Center the arm (servo 1) so camera faces forward relative to chassis
            self.arm_move(1, self.config.servo_center, self.config.move_duration)
            time.sleep(self.config.settle_time)

            # Sample at starting position (0°) before rotating
            depth_data = self.get_depth()
            if depth_data:
                readings[0.0] = DepthReading(
                    angle=0.0,
                    min_depth=depth_data.get("min_depth", 0),
                    avg_depth=depth_data.get("avg_depth", 0),
                    valid_percent=depth_data.get("valid_percent", 0),
                )
                console.print(f"[dim]  0°: avg={readings[0.0].avg_depth:.2f}m[/dim]")

            # Rotate and sample at each position
            for i in range(1, num_samples):
                # Rotate one step (positive = counter-clockwise = left)
                chassis_move_func(0, 0, turn_speed)
                time.sleep(time_per_sample)
                chassis_stop_func()
                time.sleep(0.3)  # Brief settle

                # Calculate current angle (cumulative)
                angle = angle_per_sample * i

                # Sample depth
                depth_data = self.get_depth()
                if depth_data:
                    readings[angle] = DepthReading(
                        angle=angle,
                        min_depth=depth_data.get("min_depth", 0),
                        avg_depth=depth_data.get("avg_depth", 0),
                        valid_percent=depth_data.get("valid_percent", 0),
                    )
                    console.print(f"[dim]  {angle:.0f}°: avg={readings[angle].avg_depth:.2f}m[/dim]")
                else:
                    console.print(f"[dim]  {angle:.0f}°: no data[/dim]")

        except Exception as e:
            console.print(f"[red]Chassis 360° scan failed: {e}[/red]")
            chassis_stop_func()

        return readings

    def find_best_360_opening(
        self,
        readings: Dict[float, DepthReading],
        min_clearance: float = None,
    ) -> Optional[float]:
        """
        Find the best escape direction from a 360° scan.

        Args:
            readings: Dict from chassis_360_scan()
            min_clearance: Minimum depth to consider "open" (default: config value)

        Returns:
            Angle in degrees to turn toward, or None if no good opening found.
            Angle is relative to starting position (0=front at scan start).
        """
        if not readings:
            return None

        min_clearance = min_clearance or self.config.min_clearance

        # Filter to readings with sufficient clearance
        valid_readings = [
            r for r in readings.values()
            if r.avg_depth >= min_clearance and r.valid_percent > 30
        ]

        if not valid_readings:
            console.print(f"[dim]No openings with ≥{min_clearance}m clearance[/dim]")
            return None

        # Find the one with greatest average depth
        best = max(valid_readings, key=lambda r: r.avg_depth)

        console.print(
            f"[green]Best 360° opening: {best.angle:.0f}° with {best.avg_depth:.2f}m clearance[/green]"
        )

        return best.angle
