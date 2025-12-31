"""Main exploration controller."""

import math
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

from rich.console import Console

from .safety_monitor import SafetyMonitor, SafetyConfig
from .frontier_planner import FrontierPlanner, FrontierConfig
from .sensor_fusion import SensorFusion, SensorConfig
from .data_logger import DataLogger, LogConfig
from .escape_handler import EscapeHandler, EscapeConfig, EscapeLevel
from .arm_scanner import ArmScanner, ArmScannerConfig
from typing import Protocol


class LoggerProtocol(Protocol):
    """Protocol for data loggers (local or remote)."""
    session_dir: str | None

    def start_session(self, robot_config: dict | None = None) -> str: ...
    def log_lidar(self, sector_mins: list, sector_maxs: list) -> None: ...
    def log_depth(self, min_depth: float, avg_depth: float, valid_percent: float) -> None: ...
    def log_odometry(self, vx: float, vy: float, wz: float, estimated_x: float = 0, estimated_y: float = 0) -> None: ...
    def log_event(self, event_type: str, details: dict | None = None) -> None: ...
    def end_session(self, summary: dict | None = None) -> dict: ...

console = Console()


@dataclass
class ExplorationConfig:
    """Main exploration configuration."""
    # Motion
    forward_speed: float = 0.35  # m/s (faster exploration when clear)
    turn_speed: float = 1.5  # rad/s (faster turning)
    slow_factor: float = 0.5  # Speed multiplier when obstacle nearby (0.175 m/s)

    # Timing
    loop_rate_hz: float = 10.0
    decision_rate_hz: float = 2.0  # Slower decisions to match sensor update rate

    # Stuck detection (legacy, now handled by escape handler)
    stuck_timeout: float = 20.0  # seconds without movement = stuck (increased for slow sensor reads)
    stuck_escape_duration: float = 2.0  # seconds to back up when stuck

    # Oscillation detection
    turn_history_window: float = 10.0  # seconds to track turn history
    oscillation_threshold: int = 3  # alternating turns to trigger escape
    blocked_count_threshold: int = 3  # forward blocks to trigger escape

    # Carpet/friction compensation - mecanum wheels slip on carpet
    # Default 2.5x multiplier based on observed 90° command → ~30° actual rotation
    turn_time_multiplier: float = 2.5


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
        logger: Optional[LoggerProtocol] = None,
        on_speak: Optional[Callable[[str], None]] = None,
        # New: arm scanner for escape
        arm_scanner: Optional[ArmScanner] = None,
        escape_config: Optional[EscapeConfig] = None,
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
        # Use provided logger or create local one
        self.logger: LoggerProtocol = logger or DataLogger(log_config)

        # Escape handler with optional arm scanner
        self.arm_scanner = arm_scanner
        self.escape_handler = EscapeHandler(
            move_func=move_func,
            stop_func=stop_func,
            arm_scanner=arm_scanner,
            config=escape_config,
            on_speak=self.speak,
        )

        # State
        self.running = False
        self.last_decision_time: float = 0
        self.last_significant_movement: float = 0
        self.current_action: str = "idle"

        # Oscillation detection state
        self.turn_history: List[Tuple[str, float]] = []  # (direction, timestamp)
        self.forward_blocked_count: int = 0

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

        # Log depth stats if available
        if depth_data and self.logger.session_dir:
            valid_depths = [d for d in depth_data if 150 <= d <= 3000]
            if valid_depths:
                min_depth = min(valid_depths) / 1000.0  # mm to m
                avg_depth = sum(valid_depths) / len(valid_depths) / 1000.0
                valid_pct = len(valid_depths) / len(depth_data) * 100.0
                self.logger.log_depth(min_depth, avg_depth, valid_pct)

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
            self.forward_blocked_count += 1
            self.logger.log_event("obstacle_stop", {"distance": obstacle.closest_distance})
            console.print(f"[red]Obstacle at {obstacle.closest_distance:.2f}m - stopped (blocked: {self.forward_blocked_count})[/red]")

            # Check if we should trigger escape
            if self._should_escape():
                self._handle_escape()
            return True  # Continue loop but don't move

        # Check for escape in progress
        if self.escape_handler.escape_in_progress:
            if self.escape_handler.should_escalate():
                self._handle_escape_escalation()
            return True

        # Check for oscillation-triggered escape (even if not currently blocked)
        if self._is_oscillating() and not self.escape_handler.in_cooldown():
            console.print("[yellow]Oscillation detected![/yellow]")
            self._handle_escape()
            return True

        # Check for stuck condition (legacy fallback)
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
            self._handle_escape()
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

            # Record turn direction for oscillation detection
            direction = "left" if target_angle > 0 else "right"
            self._record_turn(direction)

            # DON'T mark as visited yet - we'll mark it when we actually move forward
            # This keeps the same target direction until we're facing it
        else:
            # Move forward - reset blocked count since we're making progress
            self.move(speed, 0, 0)
            self.current_action = "moving forward"
            self.last_significant_movement = time.time()
            self.forward_blocked_count = 0  # Reset on successful forward movement

            # Mark front as visited when actually moving forward
            self.planner.mark_visited(0)

        # Log
        self.logger.log_odometry(speed if abs(target_angle) <= 30 else 0, 0, 0)

    def _check_stuck(self) -> bool:
        """Check if robot is stuck."""
        return (time.time() - self.last_significant_movement) > self.config.stuck_timeout

    def _escape_stuck(self) -> None:
        """Attempt to escape stuck situation (legacy fallback)."""
        console.print("[yellow]Stuck detected - using legacy escape[/yellow]")
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

    # --- Oscillation Detection ---

    def _record_turn(self, direction: str) -> None:
        """Record a turn direction for oscillation detection."""
        now = time.time()
        self.turn_history.append((direction, now))
        # Keep only recent turns within the window
        cutoff = now - self.config.turn_history_window
        self.turn_history = [(d, t) for d, t in self.turn_history if t >= cutoff]

    def _is_oscillating(self) -> bool:
        """Detect oscillation pattern in recent turns."""
        if len(self.turn_history) < 4:
            return False

        # Count alternations in the last turns
        recent = [d for d, _ in self.turn_history[-6:]]
        if len(recent) < 4:
            return False

        alternations = sum(1 for i in range(1, len(recent)) if recent[i] != recent[i - 1])
        return alternations >= self.config.oscillation_threshold

    def _should_escape(self) -> bool:
        """Check if escape should be triggered."""
        if self.escape_handler.in_cooldown():
            return False
        if self.escape_handler.escape_in_progress:
            return False
        return (
            self._is_oscillating() or
            self.forward_blocked_count >= self.config.blocked_count_threshold
        )

    # --- Escape Handling ---

    def _handle_escape(self) -> None:
        """Start or continue escape sequence."""
        if self.escape_handler.escape_in_progress:
            return

        console.print("[bold yellow]Starting progressive escape[/bold yellow]")
        self.logger.log_event("escape_start", {
            "oscillating": self._is_oscillating(),
            "blocked_count": self.forward_blocked_count,
        })

        result = self.escape_handler.start_escape()
        self._execute_escape_result(result)

    def _handle_escape_escalation(self) -> None:
        """Escalate to next escape level."""
        console.print("[yellow]Escape level timeout - escalating[/yellow]")
        result = self.escape_handler.escalate()
        self._execute_escape_result(result)

    def _execute_escape_result(self, result) -> None:
        """Execute the motion from an escape result."""
        self.logger.log_event("escape_action", {
            "level": result.level.name,
            "direction": result.direction,
            "message": result.message,
        })

        if result.level == EscapeLevel.TRAPPED:
            # Give up
            self.current_action = "trapped"
            return

        if result.direction is not None:
            # Execute the turn
            self._execute_turn(result.direction)

            # Check if escape was successful (did we find a way out?)
            # For now, assume success and reset
            self.escape_handler.reset()
            self._clear_escape_state()
        elif not result.success and result.level == EscapeLevel.FULL_SCAN:
            # Need chassis rotation - do it here with lidar
            self._execute_chassis_360_scan()

    def _execute_turn(self, angle_degrees: float) -> None:
        """Execute a turn by the specified angle."""
        # Calculate turn duration based on angle and speed
        # Apply carpet friction multiplier to compensate for mecanum wheel slip
        angle_rad = abs(math.radians(angle_degrees))
        duration = (angle_rad / self.config.turn_speed) * self.config.turn_time_multiplier

        console.print(f"[dim]Executing turn: {angle_degrees}° ({duration:.1f}s, multiplier={self.config.turn_time_multiplier})[/dim]")

        # Turn direction
        wz = self.config.turn_speed if angle_degrees > 0 else -self.config.turn_speed

        self.move(0, 0, wz)
        time.sleep(duration)
        self.stop()

        self.current_action = f"turned {angle_degrees:.0f}°"
        self.last_significant_movement = time.time()

    def _execute_chassis_360_scan(self) -> None:
        """Execute a 360° chassis rotation to scan with lidar."""
        console.print("[yellow]Performing 360° chassis scan[/yellow]")
        self.speak("Scanning all around")

        best_angle = None
        best_distance = 0

        # Rotate slowly, sampling lidar
        # Apply carpet friction multiplier to compensate for mecanum wheel slip
        rotation_speed = 0.8  # rad/s
        sample_interval = 0.5  # seconds
        # Base duration ~8s for 360° at 0.8 rad/s, but carpet needs multiplier
        total_duration = (2 * math.pi / rotation_speed) * self.config.turn_time_multiplier

        console.print(f"[dim]360° scan duration: {total_duration:.1f}s (multiplier={self.config.turn_time_multiplier})[/dim]")

        start_time = time.time()
        angle_traveled = 0

        self.move(0, 0, rotation_speed)

        while (time.time() - start_time) < total_duration:
            time.sleep(sample_interval)
            angle_traveled += rotation_speed * sample_interval

            # Check front sector distance
            front_distance = self.planner.sectors[0].min_distance if self.planner.sectors else 0

            if front_distance > best_distance:
                best_distance = front_distance
                best_angle = math.degrees(angle_traveled)
                console.print(f"[dim]Found opening at {best_angle:.0f}°: {best_distance:.2f}m[/dim]")

        self.stop()

        if best_angle is not None and best_distance > 0.8:
            console.print(f"[green]Best opening at {best_angle:.0f}° with {best_distance:.2f}m[/green]")
            # Turn to face the best direction
            # We've already rotated past it, so calculate remaining turn
            remaining_turn = best_angle - math.degrees(angle_traveled)
            if remaining_turn < -180:
                remaining_turn += 360
            elif remaining_turn > 180:
                remaining_turn -= 360

            self._execute_turn(remaining_turn)
            self.escape_handler.reset()
            self._clear_escape_state()
        else:
            console.print("[red]No opening found in 360° scan[/red]")
            # Escalate to trapped
            self.escape_handler.escalate()

    def _clear_escape_state(self) -> None:
        """Clear escape-related state after successful escape."""
        self.turn_history.clear()
        self.forward_blocked_count = 0
        self.last_significant_movement = time.time()

    def get_status(self) -> dict:
        """Get exploration status."""
        safety_status = self.safety.get_status()
        sensor_status = self.sensors.get_status()
        planner_status = self.planner.get_status()
        escape_status = self.escape_handler.get_status()

        return {
            "running": self.running,
            "action": self.current_action,
            "safety": safety_status,
            "sensors": sensor_status,
            "planner": planner_status,
            "escape": escape_status,
            "oscillation": {
                "turn_history_count": len(self.turn_history),
                "forward_blocked_count": self.forward_blocked_count,
                "is_oscillating": self._is_oscillating(),
            },
        }
