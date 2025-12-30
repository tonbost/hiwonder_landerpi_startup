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
        logger: Optional[LoggerProtocol] = None,
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
        # Use provided logger or create local one
        self.logger: LoggerProtocol = logger or DataLogger(log_config)

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
