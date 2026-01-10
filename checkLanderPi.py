#!/usr/bin/env python3
"""
LanderPi Health Check Tool

Remotely diagnoses a Raspberry Pi 5 robot to verify all peripherals
and system components are functioning correctly.
"""

import json
import sys
from pathlib import Path
from typing import Optional
from dataclasses import dataclass, field

import typer
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich import box
from fabric import Connection
from invoke.exceptions import UnexpectedExit

app = typer.Typer(help="HiWonder LanderPi Health Check Tool")
console = Console()


def get_config() -> dict:
    """Load connection config from config.json."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        with open(config_path) as f:
            return json.load(f)
    return {}


def get_connection_params(
    host: Optional[str],
    user: Optional[str],
    password: Optional[str]
) -> tuple[str, str, Optional[str]]:
    """Get connection parameters from args or config.json."""
    config = get_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host:
        console.print("[red]Missing host. Use --host or add to config.json[/red]")
        raise typer.Exit(1)
    if not user:
        console.print("[red]Missing user. Use --user or add to config.json[/red]")
        raise typer.Exit(1)

    return host, user, password


@dataclass
class CheckResult:
    """Result of a single health check."""
    name: str
    status: str  # "ok", "warning", "error"
    message: str
    details: str = ""


@dataclass
class HealthReport:
    """Complete health report for the robot."""
    hostname: str = ""
    model: str = ""
    kernel: str = ""
    uptime: str = ""
    temperature: str = ""
    cpu_load: str = ""
    memory_used: str = ""
    memory_total: str = ""
    disk_used: str = ""
    disk_total: str = ""
    checks: list = field(default_factory=list)

    def add_check(self, check: CheckResult):
        self.checks.append(check)

    @property
    def has_errors(self) -> bool:
        return any(c.status == "error" for c in self.checks)

    @property
    def has_warnings(self) -> bool:
        return any(c.status == "warning" for c in self.checks)


class LanderPiChecker:
    """Health checker for LanderPi robot."""

    # Expected peripherals
    EXPECTED_USB_DEVICES = [
        ("1a86:55d4", "USB Serial", "/dev/ttyACM0"),
        ("3251:1930", "Aurora 930 Depth Camera", None),
        ("1a86:7523", "CH340 Serial", "/dev/ttyUSB*"),
        ("0c76:161f", "USB Audio Device", None),
    ]

    REQUIRED_GROUPS = ["dialout", "audio", "video", "docker"]
    OPTIONAL_GROUPS = ["i2c", "gpio"]

    def __init__(self, host: str, user: str, password: Optional[str] = None, key_filename: Optional[str] = None):
        self.host = host
        self.user = user
        self.console = console
        self.report = HealthReport()

        connect_kwargs = {}
        if password:
            connect_kwargs['password'] = password
        if key_filename:
            connect_kwargs['key_filename'] = key_filename

        self.conn = Connection(host=host, user=user, connect_kwargs=connect_kwargs)

    def run_cmd(self, command: str, warn: bool = True) -> tuple[bool, str, str]:
        """Run a command and return (success, stdout, stderr)."""
        try:
            result = self.conn.run(command, hide=True, warn=warn)
            return result.ok, result.stdout.strip(), result.stderr.strip()
        except UnexpectedExit as e:
            return False, "", str(e)
        except Exception as e:
            return False, "", str(e)

    def run_sudo(self, command: str, warn: bool = True) -> tuple[bool, str, str]:
        """Run a sudo command and return (success, stdout, stderr)."""
        try:
            result = self.conn.sudo(command, hide=True, warn=warn, pty=True)
            return result.ok, result.stdout.strip(), result.stderr.strip()
        except UnexpectedExit as e:
            return False, "", str(e)
        except Exception as e:
            return False, "", str(e)

    def check_connection(self) -> bool:
        """Verify SSH connection."""
        try:
            self.console.print(f"[bold blue]Connecting to {self.user}@{self.host}...[/bold blue]")
            self.conn.run("echo 'connected'", hide=True)
            self.console.print("[green]✓ Connected successfully[/green]")
            return True
        except Exception as e:
            self.console.print(f"[red]✗ Connection failed:[/red] {e}")
            return False

    def check_system_info(self):
        """Gather basic system information."""
        self.console.print("[bold]Checking system info...[/bold]")

        # Model
        ok, out, _ = self.run_cmd("cat /proc/device-tree/model 2>/dev/null | tr -d '\\0'")
        self.report.model = out if ok else "Unknown"

        # Hostname
        ok, out, _ = self.run_cmd("hostname")
        self.report.hostname = out if ok else "Unknown"

        # Kernel
        ok, out, _ = self.run_cmd("uname -r")
        self.report.kernel = out if ok else "Unknown"

        # Uptime
        ok, out, _ = self.run_cmd("uptime -p")
        self.report.uptime = out.replace("up ", "") if ok else "Unknown"

        # Temperature
        ok, out, _ = self.run_cmd("vcgencmd measure_temp 2>/dev/null")
        if ok and "temp=" in out:
            self.report.temperature = out.split("=")[1]
            temp_val = float(self.report.temperature.replace("'C", ""))
            if temp_val > 80:
                self.report.add_check(CheckResult("Temperature", "error", f"{temp_val}°C - OVERHEATING"))
            elif temp_val > 70:
                self.report.add_check(CheckResult("Temperature", "warning", f"{temp_val}°C - High"))
            else:
                self.report.add_check(CheckResult("Temperature", "ok", f"{temp_val}°C"))
        else:
            # Try thermal zone as fallback
            ok, out, _ = self.run_cmd("cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null")
            if ok:
                temp_val = int(out) / 1000
                self.report.temperature = f"{temp_val}'C"
                self.report.add_check(CheckResult("Temperature", "ok", f"{temp_val}°C"))
            else:
                self.report.temperature = "N/A"
                self.report.add_check(CheckResult("Temperature", "warning", "Cannot read temperature", "vcgencmd or thermal zone not accessible"))

        # CPU Load
        ok, out, _ = self.run_cmd("cat /proc/loadavg")
        if ok:
            loads = out.split()[:3]
            self.report.cpu_load = ", ".join(loads)

        # Memory
        ok, out, _ = self.run_cmd("free -h | grep Mem")
        if ok:
            parts = out.split()
            self.report.memory_total = parts[1]
            self.report.memory_used = parts[2]

        # Disk
        ok, out, _ = self.run_cmd("df -h / | tail -1")
        if ok:
            parts = out.split()
            self.report.disk_total = parts[1]
            self.report.disk_used = parts[2]
            usage_pct = int(parts[4].replace("%", ""))
            if usage_pct > 90:
                self.report.add_check(CheckResult("Disk Space", "error", f"{usage_pct}% used", "Critical: Low disk space"))
            elif usage_pct > 80:
                self.report.add_check(CheckResult("Disk Space", "warning", f"{usage_pct}% used", "Warning: Disk filling up"))
            else:
                self.report.add_check(CheckResult("Disk Space", "ok", f"{usage_pct}% used"))

    def check_user_groups(self):
        """Check if user has required group memberships."""
        self.console.print("[bold]Checking user groups...[/bold]")

        ok, out, _ = self.run_cmd("groups")
        if not ok:
            self.report.add_check(CheckResult("User Groups", "error", "Cannot determine groups"))
            return

        current_groups = out.split()

        # Check required groups
        missing_required = [g for g in self.REQUIRED_GROUPS if g not in current_groups]
        missing_optional = [g for g in self.OPTIONAL_GROUPS if g not in current_groups]

        if missing_required:
            self.report.add_check(CheckResult(
                "Required Groups",
                "error",
                f"Missing: {', '.join(missing_required)}",
                "Run setup script or: sudo usermod -aG <group> $USER"
            ))
        else:
            self.report.add_check(CheckResult("Required Groups", "ok", "All present"))

        if missing_optional:
            self.report.add_check(CheckResult(
                "Optional Groups",
                "warning",
                f"Missing: {', '.join(missing_optional)}",
                "May limit some hardware access"
            ))

    def check_usb_devices(self):
        """Check for expected USB devices."""
        self.console.print("[bold]Checking USB devices...[/bold]")

        ok, out, _ = self.run_cmd("lsusb")
        if not ok:
            self.report.add_check(CheckResult("USB Devices", "error", "Cannot list USB devices"))
            return

        usb_output = out.lower()

        for vid_pid, name, _ in self.EXPECTED_USB_DEVICES:
            if vid_pid.lower() in usb_output:
                self.report.add_check(CheckResult(f"USB: {name}", "ok", "Detected"))
            else:
                self.report.add_check(CheckResult(f"USB: {name}", "warning", "Not detected", f"Expected VID:PID {vid_pid}"))

    def check_serial_ports(self):
        """Check serial port accessibility."""
        self.console.print("[bold]Checking serial ports...[/bold]")

        serial_devices = [
            ("/dev/ttyACM0", "Lidar/USB ACM"),
            ("/dev/ttyUSB0", "CH340 Serial #1"),
            ("/dev/ttyUSB1", "CH340 Serial #2"),
        ]

        for device, name in serial_devices:
            ok, out, err = self.run_cmd(f"test -e {device} && stty -F {device} 2>&1 | head -1")
            if ok and "speed" in out:
                self.report.add_check(CheckResult(f"Serial: {name}", "ok", f"{device} accessible"))
            elif ok:
                # Device exists but can't read settings
                self.report.add_check(CheckResult(
                    f"Serial: {name}",
                    "warning",
                    f"{device} exists but may have permission issues"
                ))
            else:
                # Check if device exists at all
                ok2, _, _ = self.run_cmd(f"test -e {device}")
                if ok2:
                    self.report.add_check(CheckResult(
                        f"Serial: {name}",
                        "error",
                        f"{device} permission denied",
                        "User may not be in dialout group"
                    ))
                else:
                    self.report.add_check(CheckResult(
                        f"Serial: {name}",
                        "warning",
                        f"{device} not present",
                        "Device may not be connected"
                    ))

    def check_audio(self):
        """Check audio device status."""
        self.console.print("[bold]Checking audio...[/bold]")

        # Check ALSA
        ok, out, _ = self.run_cmd("aplay -l 2>&1")
        if "no soundcards found" in out.lower():
            self.report.add_check(CheckResult(
                "ALSA Audio",
                "error",
                "No sound cards found",
                "User may not be in audio group"
            ))
        elif ok and "USB" in out:
            self.report.add_check(CheckResult("ALSA Audio", "ok", "USB audio device detected"))
        elif ok:
            self.report.add_check(CheckResult("ALSA Audio", "warning", "Only HDMI audio detected"))
        else:
            self.report.add_check(CheckResult("ALSA Audio", "error", "Cannot list audio devices"))

        # Check PipeWire
        ok, out, _ = self.run_cmd("XDG_RUNTIME_DIR=/run/user/$(id -u) wpctl status 2>/dev/null | head -20")
        if ok:
            if "Dummy Output" in out and "USB" not in out:
                self.report.add_check(CheckResult(
                    "PipeWire",
                    "warning",
                    "Only Dummy Output available",
                    "USB audio not detected by PipeWire"
                ))
            elif "USB" in out:
                self.report.add_check(CheckResult("PipeWire", "ok", "USB audio detected"))
            else:
                self.report.add_check(CheckResult("PipeWire", "ok", "Running"))

    def check_i2c_spi(self):
        """Check I2C and SPI interfaces."""
        self.console.print("[bold]Checking I2C/SPI...[/bold]")

        # I2C
        ok, out, _ = self.run_cmd("ls /dev/i2c-* 2>/dev/null")
        if ok and out:
            devices = out.split()
            self.report.add_check(CheckResult("I2C", "ok", f"{len(devices)} bus(es): {', '.join(devices)}"))
        else:
            self.report.add_check(CheckResult(
                "I2C",
                "error",
                "No I2C devices found",
                "Check /boot/firmware/config.txt for dtparam=i2c_arm=on"
            ))

        # SPI
        ok, out, _ = self.run_cmd("ls /dev/spidev* 2>/dev/null")
        if ok and out:
            devices = out.split()
            self.report.add_check(CheckResult("SPI", "ok", f"{len(devices)} device(s)"))
        else:
            self.report.add_check(CheckResult(
                "SPI",
                "warning",
                "No SPI devices found",
                "Check /boot/firmware/config.txt for dtparam=spi=on"
            ))

    def check_gpio(self):
        """Check GPIO access."""
        self.console.print("[bold]Checking GPIO...[/bold]")

        ok, out, _ = self.run_cmd("ls /dev/gpiochip* 2>/dev/null")
        if ok and out:
            chips = [c for c in out.split() if "gpiochip" in c]
            self.report.add_check(CheckResult("GPIO", "ok", f"{len(chips)} chip(s) available"))
        else:
            self.report.add_check(CheckResult("GPIO", "error", "No GPIO chips found"))

    def check_network(self):
        """Check network connectivity."""
        self.console.print("[bold]Checking network...[/bold]")

        # WiFi status
        ok, out, _ = self.run_cmd("ip addr show wlan0 2>/dev/null | grep 'inet '")
        if ok and out:
            ip = out.split()[1].split('/')[0]
            self.report.add_check(CheckResult("WiFi", "ok", f"Connected: {ip}"))
        else:
            self.report.add_check(CheckResult("WiFi", "warning", "Not connected or not available"))

        # Ethernet status
        ok, out, _ = self.run_cmd("ip addr show eth0 2>/dev/null | grep 'inet '")
        if ok and out:
            ip = out.split()[1].split('/')[0]
            self.report.add_check(CheckResult("Ethernet", "ok", f"Connected: {ip}"))
        else:
            ok2, out2, _ = self.run_cmd("ip link show eth0 2>/dev/null | grep 'state'")
            if "DOWN" in out2:
                self.report.add_check(CheckResult("Ethernet", "warning", "Cable not connected"))

        # Internet connectivity
        ok, out, _ = self.run_cmd("ping -c 1 -W 3 8.8.8.8 2>/dev/null | grep 'time='")
        if ok:
            latency = out.split("time=")[1].split()[0] if "time=" in out else "?"
            self.report.add_check(CheckResult("Internet", "ok", f"Connected ({latency})"))
        else:
            self.report.add_check(CheckResult("Internet", "error", "No internet connectivity"))

    def check_docker(self):
        """Check Docker status."""
        self.console.print("[bold]Checking Docker...[/bold]")

        # Docker installed
        ok, out, _ = self.run_cmd("docker --version 2>/dev/null")
        if not ok:
            self.report.add_check(CheckResult("Docker", "error", "Not installed"))
            return

        version = out.split(",")[0] if "," in out else out
        self.report.add_check(CheckResult("Docker", "ok", version))

        # Docker service
        ok, out, _ = self.run_cmd("systemctl is-active docker 2>/dev/null")
        if ok and "active" in out:
            self.report.add_check(CheckResult("Docker Service", "ok", "Running"))
        else:
            self.report.add_check(CheckResult("Docker Service", "error", "Not running"))

        # Check for LanderPi images
        ok, out, _ = self.run_cmd("docker images --format '{{.Repository}}:{{.Tag}}' 2>/dev/null | grep -E '(landerpi|ros)'")
        if ok and out:
            images = out.split('\n')
            self.report.add_check(CheckResult("Docker Images", "ok", f"{len(images)} ROS/LanderPi image(s)"))
        else:
            self.report.add_check(CheckResult("Docker Images", "warning", "No LanderPi/ROS images found"))

    def check_ros2_workspace(self):
        """Check ROS2 workspace."""
        self.console.print("[bold]Checking ROS2 workspace...[/bold]")

        ok, out, _ = self.run_cmd(f"test -d /home/{self.user}/landerpi_ros && ls /home/{self.user}/landerpi_ros/")
        if ok:
            self.report.add_check(CheckResult("ROS2 Workspace", "ok", "Present"))
        else:
            self.report.add_check(CheckResult("ROS2 Workspace", "warning", "Not found"))

    def check_depth_camera(self):
        """Check Aurora 930 depth camera setup."""
        self.console.print("[bold]Checking depth camera setup...[/bold]")

        # Check if camera is detected via USB
        ok, out, _ = self.run_cmd("lsusb -d 3251:1930 2>/dev/null")
        if ok and "3251:1930" in out:
            self.report.add_check(CheckResult("Depth Camera USB", "ok", "Aurora 930 detected"))
        else:
            self.report.add_check(CheckResult(
                "Depth Camera USB",
                "warning",
                "Aurora 930 not detected",
                "Connect camera to USB port"
            ))

        # Check udev rules for Deptrum cameras
        ok, out, _ = self.run_cmd("test -f /etc/udev/rules.d/99-deptrum-libusb.rules && cat /etc/udev/rules.d/99-deptrum-libusb.rules")
        if ok and "3251" in out:
            self.report.add_check(CheckResult("Depth Camera udev", "ok", "Deptrum udev rules installed"))
        else:
            self.report.add_check(CheckResult(
                "Depth Camera udev",
                "warning",
                "Deptrum udev rules not found",
                "Run setup_landerpi.py deploy or install manually"
            ))

        # Check Deptrum workspace exists
        ok, out, _ = self.run_cmd(f"test -d /home/{self.user}/deptrum_ws/src && ls /home/{self.user}/deptrum_ws/src/")
        if ok and out:
            self.report.add_check(CheckResult("Deptrum Workspace", "ok", "Source directory present"))
        else:
            self.report.add_check(CheckResult(
                "Deptrum Workspace",
                "warning",
                "Not found",
                "See DepthCameraHowTo.md for setup"
            ))
            return  # Skip build check if workspace doesn't exist

        # Check if driver is built
        ok, out, _ = self.run_cmd(f"test -d /home/{self.user}/deptrum_ws/install/deptrum-ros-driver-aurora930")
        if ok:
            self.report.add_check(CheckResult("Depth Camera Driver", "ok", "ROS2 driver built"))
        else:
            self.report.add_check(CheckResult(
                "Depth Camera Driver",
                "warning",
                "Driver not built",
                "Build with: docker run --rm -v ~/deptrum_ws:/deptrum_ws landerpi-ros2 colcon build"
            ))

    def run_all_checks(self):
        """Run all health checks."""
        self.check_system_info()
        self.check_user_groups()
        self.check_usb_devices()
        self.check_serial_ports()
        self.check_audio()
        self.check_i2c_spi()
        self.check_gpio()
        self.check_network()
        self.check_docker()
        self.check_ros2_workspace()
        self.check_depth_camera()

    def print_report(self):
        """Print the health report."""
        # System info panel
        info_text = f"""[bold]Model:[/bold] {self.report.model}
[bold]Hostname:[/bold] {self.report.hostname}
[bold]Kernel:[/bold] {self.report.kernel}
[bold]Uptime:[/bold] {self.report.uptime}
[bold]Temperature:[/bold] {self.report.temperature}
[bold]CPU Load:[/bold] {self.report.cpu_load}
[bold]Memory:[/bold] {self.report.memory_used} / {self.report.memory_total}
[bold]Disk:[/bold] {self.report.disk_used} / {self.report.disk_total}"""

        console.print(Panel(info_text, title="System Information", border_style="blue"))

        # Results table
        table = Table(title="Health Check Results", box=box.ROUNDED)
        table.add_column("Check", style="cyan")
        table.add_column("Status", justify="center")
        table.add_column("Message")
        table.add_column("Details", style="dim")

        for check in self.report.checks:
            if check.status == "ok":
                status = "[green]✓ OK[/green]"
            elif check.status == "warning":
                status = "[yellow]⚠ WARN[/yellow]"
            else:
                status = "[red]✗ ERROR[/red]"

            table.add_row(check.name, status, check.message, check.details)

        console.print(table)

        # Summary
        ok_count = sum(1 for c in self.report.checks if c.status == "ok")
        warn_count = sum(1 for c in self.report.checks if c.status == "warning")
        error_count = sum(1 for c in self.report.checks if c.status == "error")

        summary = f"[green]{ok_count} OK[/green] | [yellow]{warn_count} Warnings[/yellow] | [red]{error_count} Errors[/red]"

        if error_count > 0:
            console.print(Panel(summary, title="[red]Health Check: ISSUES FOUND[/red]", border_style="red"))
        elif warn_count > 0:
            console.print(Panel(summary, title="[yellow]Health Check: WARNINGS[/yellow]", border_style="yellow"))
        else:
            console.print(Panel(summary, title="[green]Health Check: ALL GOOD[/green]", border_style="green"))


@app.command()
def check(
    host: Optional[str] = typer.Option(None, help="Raspberry Pi IP address (or use config.json)"),
    user: Optional[str] = typer.Option(None, help="SSH Username (or use config.json)"),
    password: Optional[str] = typer.Option(None, help="SSH Password (or use config.json)"),
    key_path: Optional[str] = typer.Option(None, help="Path to SSH private key"),
):
    """Run a comprehensive health check on the LanderPi robot."""
    console.print(Panel.fit("LanderPi Health Check", style="bold magenta"))

    host, user, password = get_connection_params(host, user, password)
    checker = LanderPiChecker(host, user, password, key_path)

    if not checker.check_connection():
        sys.exit(1)

    console.print()
    checker.run_all_checks()
    console.print()
    checker.print_report()

    # Exit with appropriate code
    if checker.report.has_errors:
        sys.exit(2)
    elif checker.report.has_warnings:
        sys.exit(1)
    sys.exit(0)


@app.command()
def quick(
    host: Optional[str] = typer.Option(None, help="Raspberry Pi IP address (or use config.json)"),
    user: Optional[str] = typer.Option(None, help="SSH Username (or use config.json)"),
    password: Optional[str] = typer.Option(None, help="SSH Password (or use config.json)"),
    key_path: Optional[str] = typer.Option(None, help="Path to SSH private key"),
):
    """Run a quick connectivity and basic system check."""
    console.print(Panel.fit("LanderPi Quick Check", style="bold cyan"))

    host, user, password = get_connection_params(host, user, password)
    checker = LanderPiChecker(host, user, password, key_path)

    if not checker.check_connection():
        sys.exit(1)

    # Just system info and critical checks
    checker.check_system_info()
    checker.check_user_groups()
    checker.check_serial_ports()

    console.print()
    checker.print_report()


if __name__ == "__main__":
    app()
