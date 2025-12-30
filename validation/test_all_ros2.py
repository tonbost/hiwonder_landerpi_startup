#!/usr/bin/env python3
"""
Run all ROS2 test scripts and generate a summary report.
Requires deployed ROS2 stack: `deploy_ros2_stack.py deploy`
"""
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

import typer
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

app = typer.Typer(help="LanderPi ROS2 Test Suite")
console = Console()

# Test definitions: (name, script, command, needs_approval, description)
TESTS = [
    ("Arm", "test_arm_ros2.py", "test", True, "Arm servo control and positions"),
    ("Lidar", "test_lidar_ros2.py", "scan", False, "Lidar scan data acquisition"),
    ("Camera", "test_cameradepth_ros2.py", "check", False, "Depth camera topics"),
    ("Chassis", "test_chassis_motion_ros2.py", "test", True, "Chassis motion (forward/back/turn/strafe)"),
]


@dataclass
class TestResult:
    name: str
    passed: bool
    duration: float
    output: str = ""
    error: str = ""


@dataclass
class TestReport:
    results: list[TestResult] = field(default_factory=list)
    start_time: datetime = field(default_factory=datetime.now)
    end_time: Optional[datetime] = None

    @property
    def total_duration(self) -> float:
        if self.end_time:
            return (self.end_time - self.start_time).total_seconds()
        return 0.0

    @property
    def passed_count(self) -> int:
        return sum(1 for r in self.results if r.passed)

    @property
    def failed_count(self) -> int:
        return sum(1 for r in self.results if not r.passed)

    @property
    def all_passed(self) -> bool:
        return all(r.passed for r in self.results)


def run_test(script: str, command: str, needs_approval: bool = True, extra_args: list[str] = None) -> tuple[bool, float, str, str]:
    """Run a test script and return (passed, duration, stdout, stderr)."""
    script_path = Path(__file__).parent / script

    cmd = ["uv", "run", "python", str(script_path), command]
    if needs_approval:
        cmd.append("--yes")
    if extra_args:
        cmd.extend(extra_args)

    start = time.time()
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=180,  # 3 minute timeout per test
        )
        duration = time.time() - start
        passed = result.returncode == 0
        return passed, duration, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        duration = time.time() - start
        return False, duration, "", "Test timed out after 180 seconds"
    except Exception as e:
        duration = time.time() - start
        return False, duration, "", str(e)


def print_report(report: TestReport):
    """Print the test report."""
    console.print("\n")

    # Results table
    table = Table(title="Test Results", show_header=True, header_style="bold")
    table.add_column("Test", style="cyan")
    table.add_column("Status", justify="center")
    table.add_column("Duration", justify="right")

    for result in report.results:
        status = "[green]PASS[/green]" if result.passed else "[red]FAIL[/red]"
        table.add_row(result.name, status, f"{result.duration:.1f}s")

    console.print(table)

    # Summary
    status_color = "green" if report.all_passed else "red"
    console.print(Panel.fit(
        f"[bold]Passed:[/bold] {report.passed_count}/{len(report.results)}\n"
        f"[bold]Failed:[/bold] {report.failed_count}/{len(report.results)}\n"
        f"[bold]Total Time:[/bold] {report.total_duration:.1f}s",
        title=f"[{status_color}]{'ALL TESTS PASSED' if report.all_passed else 'SOME TESTS FAILED'}[/{status_color}]",
        border_style=status_color
    ))

    # Show failures
    if report.failed_count > 0:
        console.print("\n[bold red]Failed Tests:[/bold red]")
        for result in report.results:
            if not result.passed:
                console.print(f"\n[red]{result.name}:[/red]")
                if result.error:
                    console.print(f"[dim]{result.error[:500]}[/dim]")


@app.command()
def run(
    skip_chassis: bool = typer.Option(False, "--skip-chassis", help="Skip chassis motion test"),
    skip_arm: bool = typer.Option(False, "--skip-arm", help="Skip arm test"),
    skip_lidar: bool = typer.Option(False, "--skip-lidar", help="Skip lidar test"),
    skip_camera: bool = typer.Option(False, "--skip-camera", help="Skip camera test"),
    distance: float = typer.Option(0.3, "--distance", "-d", help="Chassis test distance in meters"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Show test output"),
):
    """
    Run all ROS2 tests and generate a report.

    Requires: deploy_ros2_stack.py deploy
    """
    console.print(Panel.fit(
        "[bold]LanderPi ROS2 Test Suite[/bold]\n\n"
        "Running all ROS2 tests sequentially.\n"
        "[dim]Ensure ROS2 stack is deployed.[/dim]",
        title="Test Suite",
        border_style="blue"
    ))

    # Build test list based on skip flags
    tests_to_run = []
    for name, script, cmd, needs_approval, desc in TESTS:
        if name == "Arm" and skip_arm:
            continue
        if name == "Lidar" and skip_lidar:
            continue
        if name == "Camera" and skip_camera:
            continue
        if name == "Chassis" and skip_chassis:
            continue
        tests_to_run.append((name, script, cmd, needs_approval, desc))

    if not tests_to_run:
        console.print("[yellow]No tests to run (all skipped)[/yellow]")
        sys.exit(0)

    report = TestReport()

    for name, script, cmd, needs_approval, desc in tests_to_run:
        console.print(f"\n[bold blue]Running {name} test...[/bold blue]")
        console.print(f"[dim]{desc}[/dim]")

        # Add extra args for chassis test
        extra_args = []
        if name == "Chassis":
            extra_args = ["--distance", str(distance), "--direction", "all"]

        passed, duration, stdout, stderr = run_test(script, cmd, needs_approval, extra_args)

        result = TestResult(
            name=name,
            passed=passed,
            duration=duration,
            output=stdout,
            error=stderr
        )
        report.results.append(result)

        if passed:
            console.print(f"[green]✓ {name} passed ({duration:.1f}s)[/green]")
        else:
            console.print(f"[red]✗ {name} failed ({duration:.1f}s)[/red]")

        if verbose and stdout:
            console.print(f"[dim]{stdout[:1000]}[/dim]")

        # Brief pause between tests
        time.sleep(1)

    report.end_time = datetime.now()
    print_report(report)

    sys.exit(0 if report.all_passed else 1)


@app.command()
def list_tests():
    """List available tests."""
    table = Table(title="Available Tests", show_header=True)
    table.add_column("Name", style="cyan")
    table.add_column("Script")
    table.add_column("Description")

    for name, script, cmd, needs_approval, desc in TESTS:
        table.add_row(name, script, desc)

    console.print(table)


if __name__ == "__main__":
    app()
