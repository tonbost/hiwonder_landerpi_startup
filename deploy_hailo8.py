#!/usr/bin/env python3
"""
Deploy Hailo 8 AI Accelerator to LanderPi Robot.

Installs HailoRT driver, uploads models, and deploys Hailo-accelerated YOLO node.

Usage:
    uv run python deploy_hailo8.py check     # Check Hailo hardware/driver status
    uv run python deploy_hailo8.py install   # Install HailoRT on robot
    uv run python deploy_hailo8.py deploy    # Upload models + ROS2 node
    uv run python deploy_hailo8.py test      # Run validation test
    uv run python deploy_hailo8.py status    # Show Hailo device info
"""

import json
from pathlib import Path

import typer
from fabric import Connection
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

app = typer.Typer(help="Deploy Hailo 8 AI Accelerator to LanderPi Robot")
console = Console()

CONFIG_FILE = Path(__file__).parent / "config.json"
HAILO_DIR = Path(__file__).parent / "hailo8-int"


def load_config() -> dict:
    """Load robot connection config."""
    if CONFIG_FILE.exists():
        with open(CONFIG_FILE) as f:
            return json.load(f)
    raise ValueError("config.json not found")


def get_connection() -> Connection:
    """Get SSH connection to robot."""
    config = load_config()
    return Connection(
        host=config["host"],
        user=config["user"],
        connect_kwargs={"password": config["password"]}
    )


@app.command()
def check():
    """Check Hailo hardware and driver status (read-only)."""
    console.print(Panel("Checking Hailo 8 Status", style="bold blue"))

    conn = get_connection()

    # Check PCIe device
    console.print("\n[bold]PCIe Device:[/bold]")
    result = conn.run("lspci | grep -i hailo", hide=True, warn=True)
    if result.return_code == 0:
        console.print(f"  [green]✓[/green] {result.stdout.strip()}")
        pcie_ok = True
    else:
        console.print("  [red]✗[/red] Hailo device not found on PCIe bus")
        pcie_ok = False

    # Check device node
    console.print("\n[bold]Device Node:[/bold]")
    result = conn.run("ls -la /dev/hailo* 2>/dev/null", hide=True, warn=True)
    if result.return_code == 0:
        console.print(f"  [green]✓[/green] {result.stdout.strip()}")
        device_ok = True
    else:
        console.print("  [red]✗[/red] /dev/hailo* not found (driver not loaded)")
        device_ok = False

    # Check kernel module
    console.print("\n[bold]Kernel Module:[/bold]")
    result = conn.run("lsmod | grep hailo", hide=True, warn=True)
    if result.return_code == 0:
        console.print(f"  [green]✓[/green] {result.stdout.strip()}")
        module_ok = True
    else:
        console.print("  [red]✗[/red] hailo kernel module not loaded")
        module_ok = False

    # Check hailortcli
    console.print("\n[bold]HailoRT CLI:[/bold]")
    result = conn.run("which hailortcli && hailortcli --version", hide=True, warn=True)
    if result.return_code == 0:
        console.print(f"  [green]✓[/green] {result.stdout.strip()}")
        cli_ok = True
    else:
        console.print("  [red]✗[/red] hailortcli not installed")
        cli_ok = False

    # Check Python bindings
    console.print("\n[bold]Python Bindings:[/bold]")
    result = conn.run("python3 -c 'import hailo_platform; print(hailo_platform.__version__)'", hide=True, warn=True)
    if result.return_code == 0:
        console.print(f"  [green]✓[/green] hailo_platform {result.stdout.strip()}")
        python_ok = True
    else:
        console.print("  [red]✗[/red] hailo_platform not installed")
        python_ok = False

    # Summary
    console.print("\n[bold]Summary:[/bold]")
    table = Table(show_header=False)
    table.add_column("Component", style="cyan")
    table.add_column("Status")

    table.add_row("PCIe Device", "[green]OK[/green]" if pcie_ok else "[red]MISSING[/red]")
    table.add_row("Device Node", "[green]OK[/green]" if device_ok else "[yellow]NOT READY[/yellow]")
    table.add_row("Kernel Module", "[green]OK[/green]" if module_ok else "[yellow]NOT LOADED[/yellow]")
    table.add_row("HailoRT CLI", "[green]OK[/green]" if cli_ok else "[yellow]NOT INSTALLED[/yellow]")
    table.add_row("Python Bindings", "[green]OK[/green]" if python_ok else "[yellow]NOT INSTALLED[/yellow]")

    console.print(table)

    if all([pcie_ok, device_ok, module_ok, cli_ok, python_ok]):
        console.print("\n[bold green]Hailo 8 is fully operational![/bold green]")
    elif pcie_ok:
        console.print("\n[bold yellow]Hardware detected. Run 'deploy_hailo8.py install' to set up driver.[/bold yellow]")
    else:
        console.print("\n[bold red]Hailo 8 hardware not detected on PCIe bus.[/bold red]")


@app.command()
def status():
    """Show detailed Hailo device information."""
    console.print(Panel("Hailo 8 Device Status", style="bold blue"))

    conn = get_connection()

    # Run hailortcli scan
    console.print("\n[bold]Device Scan:[/bold]")
    result = conn.run("hailortcli scan", hide=True, warn=True)
    if result.return_code == 0:
        console.print(result.stdout)
    else:
        console.print("[red]Failed to scan devices. Is driver installed?[/red]")
        return

    # Run fw-control identify
    console.print("\n[bold]Firmware Info:[/bold]")
    result = conn.run("hailortcli fw-control identify", hide=True, warn=True)
    if result.return_code == 0:
        console.print(result.stdout)
    else:
        console.print("[yellow]Could not get firmware info[/yellow]")

    # Get temperature if available
    console.print("\n[bold]Temperature:[/bold]")
    result = conn.run("hailortcli fw-control identify | grep -i temp", hide=True, warn=True)
    if result.return_code == 0 and result.stdout.strip():
        console.print(f"  {result.stdout.strip()}")
    else:
        console.print("  Temperature info not available")


@app.command()
def install():
    """Install HailoRT driver stack on robot."""
    console.print(Panel("Installing HailoRT Driver", style="bold blue"))
    console.print("[yellow]TODO: Implement driver installation[/yellow]")
    console.print("\nThis will:")
    console.print("  1. Add Hailo APT repository")
    console.print("  2. Install hailort and hailort-dkms packages")
    console.print("  3. Load hailo_pci kernel module")
    console.print("  4. Install Python bindings")


@app.command()
def deploy():
    """Upload models and ROS2 node to robot."""
    console.print(Panel("Deploying Hailo Models and Node", style="bold blue"))
    console.print("[yellow]TODO: Implement deployment[/yellow]")
    console.print("\nThis will:")
    console.print("  1. Upload HEF models to ~/landerpi/hailo/models/")
    console.print("  2. Upload yolo_hailo ROS2 node")
    console.print("  3. Update launch configuration")


@app.command()
def test():
    """Run Hailo validation test."""
    console.print(Panel("Running Hailo Validation Test", style="bold blue"))
    console.print("[yellow]TODO: Implement validation test[/yellow]")
    console.print("\nThis will:")
    console.print("  1. Run hailortcli benchmark")
    console.print("  2. Test inference on sample image")
    console.print("  3. Report FPS and latency")


if __name__ == "__main__":
    app()
