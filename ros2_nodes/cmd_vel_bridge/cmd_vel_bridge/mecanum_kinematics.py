"""Mecanum wheel kinematics for 4-wheel robot."""

from dataclasses import dataclass
from typing import Tuple


@dataclass
class WheelSpeeds:
    """Wheel speeds in rotations per second (RPS)."""
    front_left: float
    back_left: float
    front_right: float
    back_right: float


@dataclass
class RobotGeometry:
    """Robot physical dimensions."""
    wheel_radius: float  # meters
    wheel_base: float    # front-to-back distance (meters)
    track_width: float   # left-to-right distance (meters)

    @property
    def lx(self) -> float:
        """Half of wheel base."""
        return self.wheel_base / 2.0

    @property
    def ly(self) -> float:
        """Half of track width."""
        return self.track_width / 2.0


def twist_to_wheel_speeds(
    vx: float,
    vy: float,
    wz: float,
    geometry: RobotGeometry,
    max_rps: float = 3.0
) -> WheelSpeeds:
    """
    Convert Twist velocities to individual wheel speeds.

    Args:
        vx: Linear velocity X (forward/backward, m/s)
        vy: Linear velocity Y (strafe left/right, m/s)
        wz: Angular velocity Z (rotation, rad/s)
        geometry: Robot physical dimensions
        max_rps: Maximum wheel rotations per second (safety limit)

    Returns:
        WheelSpeeds with RPS for each wheel

    LanderPi motor configuration:
        - Left motors (M1, M2): negative = forward, positive = backward
        - Right motors (M3, M4): positive = forward, negative = backward

    Mecanum kinematics with LanderPi motor sign convention:
        FL = -(vx - vy - (lx + ly) * wz) / r  (negated for left side)
        BL = -(vx + vy - (lx + ly) * wz) / r  (negated for left side)
        FR = (vx + vy + (lx + ly) * wz) / r
        BR = (vx - vy + (lx + ly) * wz) / r
    """
    r = geometry.wheel_radius
    k = geometry.lx + geometry.ly  # Combined geometry factor

    # Calculate wheel angular velocities (rad/s)
    # Left side negated to match LanderPi motor convention
    fl_rad = -(vx - vy - k * wz) / r
    bl_rad = -(vx + vy - k * wz) / r
    fr_rad = (vx + vy + k * wz) / r
    br_rad = (vx - vy + k * wz) / r

    # Convert to RPS (rad/s -> rev/s)
    fl_rps = fl_rad / (2.0 * 3.14159265359)
    bl_rps = bl_rad / (2.0 * 3.14159265359)
    fr_rps = fr_rad / (2.0 * 3.14159265359)
    br_rps = br_rad / (2.0 * 3.14159265359)

    # Clamp to max RPS
    def clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    return WheelSpeeds(
        front_left=clamp(fl_rps, max_rps),
        back_left=clamp(bl_rps, max_rps),
        front_right=clamp(fr_rps, max_rps),
        back_right=clamp(br_rps, max_rps),
    )
