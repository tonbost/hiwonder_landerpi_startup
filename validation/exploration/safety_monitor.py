"""Safety monitor for autonomous exploration."""

import time
from dataclasses import dataclass
from typing import Callable, Optional

from rich.console import Console

console = Console()


@dataclass
class SafetyConfig:
    """Safety configuration parameters."""
    max_runtime_minutes: float = 30.0
    battery_warning_voltage: float = 7.0
    battery_cutoff_voltage: float = 6.6
    battery_check_interval: float = 30.0  # seconds


class SafetyMonitor:
    """Monitor battery and runtime for safe operation."""

    def __init__(
        self,
        config: SafetyConfig,
        get_battery_func: Callable[[], Optional[float]],
        on_warning: Optional[Callable[[str], None]] = None,
        on_shutdown: Optional[Callable[[str], None]] = None,
    ):
        self.config = config
        self.get_battery = get_battery_func
        self.on_warning = on_warning or (lambda msg: console.print(f"[yellow]Warning: {msg}[/yellow]"))
        self.on_shutdown = on_shutdown or (lambda msg: console.print(f"[red]Shutdown: {msg}[/red]"))

        self.start_time: Optional[float] = None
        self.last_battery_check: float = 0
        self.battery_warned: bool = False
        self.should_stop: bool = False
        self.stop_reason: Optional[str] = None

    def start(self) -> None:
        """Start the safety monitor."""
        self.start_time = time.time()
        self.last_battery_check = 0
        self.battery_warned = False
        self.should_stop = False
        self.stop_reason = None
        console.print(f"[green]Safety monitor started[/green]")
        console.print(f"  Max runtime: {self.config.max_runtime_minutes} min")
        console.print(f"  Battery cutoff: {self.config.battery_cutoff_voltage}V")

    def check(self) -> bool:
        """
        Check safety conditions.
        Returns True if safe to continue, False if should stop.
        """
        if self.should_stop:
            return False

        current_time = time.time()

        # Check runtime
        if self.start_time:
            elapsed_minutes = (current_time - self.start_time) / 60.0
            if elapsed_minutes >= self.config.max_runtime_minutes:
                self.stop_reason = f"Runtime limit reached ({self.config.max_runtime_minutes} min)"
                self.on_shutdown(self.stop_reason)
                self.should_stop = True
                return False

        # Check battery (throttled)
        if current_time - self.last_battery_check >= self.config.battery_check_interval:
            self.last_battery_check = current_time
            voltage = self.get_battery()

            if voltage is not None:
                # Convert mV to V if needed
                if voltage > 100:
                    voltage = voltage / 1000.0

                if voltage <= self.config.battery_cutoff_voltage:
                    self.stop_reason = f"Battery low ({voltage:.2f}V)"
                    self.on_shutdown(self.stop_reason)
                    self.should_stop = True
                    return False

                if voltage <= self.config.battery_warning_voltage and not self.battery_warned:
                    self.on_warning(f"Battery getting low ({voltage:.2f}V)")
                    self.battery_warned = True

        return True

    def extend_runtime(self, additional_minutes: float = 30.0) -> None:
        """Extend the runtime limit."""
        self.config.max_runtime_minutes += additional_minutes
        console.print(f"[green]Runtime extended by {additional_minutes} min[/green]")

    def get_status(self) -> dict:
        """Get current safety status."""
        elapsed = 0.0
        remaining = self.config.max_runtime_minutes
        if self.start_time:
            elapsed = (time.time() - self.start_time) / 60.0
            remaining = max(0, self.config.max_runtime_minutes - elapsed)

        return {
            "elapsed_minutes": elapsed,
            "remaining_minutes": remaining,
            "should_stop": self.should_stop,
            "stop_reason": self.stop_reason,
        }

    def request_stop(self, reason: str = "Manual stop") -> None:
        """Request exploration to stop."""
        self.stop_reason = reason
        self.should_stop = True
