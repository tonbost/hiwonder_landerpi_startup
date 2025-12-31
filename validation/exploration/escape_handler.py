"""Progressive escape handler for stuck situations."""

import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional, TYPE_CHECKING

from rich.console import Console

if TYPE_CHECKING:
    from .arm_scanner import ArmScanner

console = Console()


class EscapeLevel(Enum):
    """Escape escalation levels."""
    NONE = 0
    WIDE_TURN = 1      # 90° turn toward more open side
    REVERSE_180 = 2    # Back up and turn 180°
    FULL_SCAN = 3      # Arm sweep + optional chassis rotation
    TRAPPED = 4        # Give up, wait for help


@dataclass
class EscapeConfig:
    """Escape handler configuration."""
    # Timing
    cooldown_seconds: float = 15.0  # Don't re-escalate for this long after success
    level_timeout: float = 3.0  # Time before escalating to next level

    # Level 1: Wide turn
    wide_turn_angle: float = 90.0  # degrees
    wide_turn_speed: float = 1.2  # rad/s

    # Level 2: Reverse
    reverse_distance: float = 0.5  # meters (approximate)
    reverse_speed: float = 0.15  # m/s
    reverse_duration: float = 2.0  # seconds
    turn_180_duration: float = 2.5  # seconds for 180° turn (base, before multiplier)

    # Level 3: Full scan
    scan_turn_speed: float = 0.8  # rad/s for chassis rotation
    full_rotation_duration: float = 8.0  # seconds for 360° (base, before multiplier)

    # Carpet/friction compensation - mecanum wheels slip on carpet
    # Default 2.5x multiplier based on observed 90° command → ~30° actual rotation
    turn_time_multiplier: float = 2.5


@dataclass
class EscapeResult:
    """Result of an escape attempt."""
    success: bool
    direction: Optional[float]  # Angle to turn toward (degrees), None if no direction found
    message: str
    level: EscapeLevel


class EscapeHandler:
    """
    Handles progressive escape from stuck situations.

    Escalates through increasingly aggressive maneuvers:
    1. Wide turn (90°) toward more open side
    2. Reverse + turn 180°
    3. Full scan (arm sweep + chassis rotation)
    """

    def __init__(
        self,
        move_func: Callable[[float, float, float], bool],  # vx, vy, wz
        stop_func: Callable[[], None],
        arm_scanner: Optional['ArmScanner'] = None,
        config: Optional[EscapeConfig] = None,
        on_speak: Optional[Callable[[str], None]] = None,
    ):
        self.move = move_func
        self.stop = stop_func
        self.arm_scanner = arm_scanner
        self.config = config or EscapeConfig()
        self.speak = on_speak or (lambda msg: console.print(f"[cyan]{msg}[/cyan]"))

        # State
        self.level = EscapeLevel.NONE
        self.last_escape_time: float = 0
        self.level_start_time: float = 0
        self.escape_in_progress: bool = False

    def in_cooldown(self) -> bool:
        """Check if still in cooldown period after successful escape."""
        return (time.time() - self.last_escape_time) < self.config.cooldown_seconds

    def should_escalate(self) -> bool:
        """Check if current level has timed out and should escalate."""
        if not self.escape_in_progress:
            return False
        return (time.time() - self.level_start_time) > self.config.level_timeout

    def start_escape(self) -> EscapeResult:
        """Start escape sequence at level 1."""
        if self.in_cooldown():
            return EscapeResult(False, None, "In cooldown", EscapeLevel.NONE)

        self.escape_in_progress = True
        self.level = EscapeLevel.WIDE_TURN
        self.level_start_time = time.time()

        return self._execute_current_level()

    def escalate(self) -> EscapeResult:
        """Escalate to next level."""
        if self.level.value >= EscapeLevel.TRAPPED.value:
            return EscapeResult(False, None, "Already at max level", EscapeLevel.TRAPPED)

        self.level = EscapeLevel(self.level.value + 1)
        self.level_start_time = time.time()

        console.print(f"[yellow]Escalating to {self.level.name}[/yellow]")
        return self._execute_current_level()

    def _execute_current_level(self) -> EscapeResult:
        """Execute the current escape level."""
        if self.level == EscapeLevel.WIDE_TURN:
            return self._execute_wide_turn()
        elif self.level == EscapeLevel.REVERSE_180:
            return self._execute_reverse_180()
        elif self.level == EscapeLevel.FULL_SCAN:
            return self._execute_full_scan()
        elif self.level == EscapeLevel.TRAPPED:
            return self._handle_trapped()
        else:
            return EscapeResult(False, None, "Unknown level", self.level)

    def _execute_wide_turn(self) -> EscapeResult:
        """Level 1: Turn 90° toward more open side using arm glance."""
        console.print("[yellow]Level 1: Wide turn with arm glance[/yellow]")
        self.speak("Looking for escape route")

        # Determine which side is more open
        direction = 90.0  # Default: turn left

        if self.arm_scanner:
            try:
                left_depth = self.arm_scanner.quick_glance("left")
                right_depth = self.arm_scanner.quick_glance("right")

                console.print(f"[dim]Arm glance: left={left_depth:.2f}m, right={right_depth:.2f}m[/dim]")

                if right_depth > left_depth:
                    direction = -90.0  # Turn right
            except Exception as e:
                console.print(f"[dim]Arm glance failed: {e}[/dim]")

        return EscapeResult(
            success=True,
            direction=direction,
            message=f"Wide turn {'left' if direction > 0 else 'right'}",
            level=self.level
        )

    def _execute_reverse_180(self) -> EscapeResult:
        """Level 2: Back up and turn 180°."""
        console.print("[yellow]Level 2: Reverse and turn 180[/yellow]")
        self.speak("Backing up")

        # Back up
        self.move(-self.config.reverse_speed, 0, 0)
        time.sleep(self.config.reverse_duration)
        self.stop()
        time.sleep(0.2)

        return EscapeResult(
            success=True,
            direction=180.0,
            message="Reverse and turn around",
            level=self.level
        )

    def _execute_full_scan(self) -> EscapeResult:
        """Level 3: Full scan with arm sweep, fallback to chassis rotation."""
        console.print("[yellow]Level 3: Full scan[/yellow]")
        self.speak("Scanning for escape")

        # Try arm sweep first (faster, more precise)
        if self.arm_scanner:
            try:
                console.print("[dim]Performing arm sweep...[/dim]")
                readings = self.arm_scanner.full_sweep()
                best_angle = self.arm_scanner.find_best_opening(readings)

                if best_angle is not None:
                    console.print(f"[green]Arm scan found opening at {best_angle}°[/green]")
                    return EscapeResult(
                        success=True,
                        direction=best_angle,
                        message=f"Arm scan found opening at {best_angle}°",
                        level=self.level
                    )
                else:
                    console.print("[dim]No opening found in arm sweep[/dim]")
            except Exception as e:
                console.print(f"[dim]Arm sweep failed: {e}[/dim]")

        # Fallback: chassis rotation (handled by caller with lidar)
        # Return a special result indicating chassis rotation is needed
        return EscapeResult(
            success=False,
            direction=None,
            message="Need chassis rotation for full 360° scan",
            level=self.level
        )

    def _handle_trapped(self) -> EscapeResult:
        """Level 4: Give up, wait for help."""
        console.print("[bold red]Level 4: Trapped - waiting for help[/bold red]")
        self.speak("I'm trapped. Please help me.")
        self.stop()

        return EscapeResult(
            success=False,
            direction=None,
            message="Trapped - manual intervention required",
            level=self.level
        )

    def reset(self) -> None:
        """Reset after successful escape."""
        self.level = EscapeLevel.NONE
        self.escape_in_progress = False
        self.last_escape_time = time.time()
        console.print("[green]Escape successful, resetting[/green]")

    def abort(self) -> None:
        """Abort current escape attempt."""
        self.level = EscapeLevel.NONE
        self.escape_in_progress = False
        self.stop()

    def get_status(self) -> dict:
        """Get escape handler status."""
        return {
            "level": self.level.name,
            "in_progress": self.escape_in_progress,
            "in_cooldown": self.in_cooldown(),
            "time_in_level": time.time() - self.level_start_time if self.escape_in_progress else 0,
        }
