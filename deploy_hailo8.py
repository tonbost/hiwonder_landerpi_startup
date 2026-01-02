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
def install(
    skip_reboot: bool = typer.Option(False, "--skip-reboot", help="Skip reboot prompt after DKMS install"),
):
    """Install HailoRT driver stack on robot."""
    console.print(Panel("Installing HailoRT Driver", style="bold blue"))

    conn = get_connection()
    config = load_config()
    home_dir = f"/home/{config['user']}"
    marker_dir = f"{home_dir}/.landerpi_setup"

    # Create marker directory
    conn.run(f"mkdir -p {marker_dir}", hide=True)

    def is_done(step: str) -> bool:
        result = conn.run(f"test -f {marker_dir}/hailo_{step}", hide=True, warn=True)
        return result.return_code == 0

    def mark_done(step: str):
        conn.run(f"touch {marker_dir}/hailo_{step}", hide=True)

    # Step 1: Add Hailo repository
    if is_done("repo_added"):
        console.print("[dim]Step 1: Hailo repository already added[/dim]")
    else:
        console.print("\n[bold]Step 1: Adding Hailo APT repository...[/bold]")
        conn.sudo(
            "wget -qO - https://hailo.ai/keys/hailo-public.gpg | "
            "gpg --dearmor -o /usr/share/keyrings/hailo-archive-keyring.gpg",
            hide=True,
            pty=True
        )
        conn.sudo(
            'echo "deb [arch=arm64 signed-by=/usr/share/keyrings/hailo-archive-keyring.gpg] '
            'https://hailo.ai/raspberry-pi/ stable main" | '
            'tee /etc/apt/sources.list.d/hailo.list',
            hide=True,
            pty=True
        )
        mark_done("repo_added")
        console.print("  [green]✓[/green] Repository added")

    # Step 2: Install packages
    if is_done("packages_installed"):
        console.print("[dim]Step 2: HailoRT packages already installed[/dim]")
    else:
        console.print("\n[bold]Step 2: Installing HailoRT packages...[/bold]")
        console.print("  Updating package lists...")
        conn.sudo("apt-get update -qq", hide=True, pty=True)
        console.print("  Installing hailort...")
        conn.sudo("apt-get install -y hailort", hide=True, pty=True)
        console.print("  Installing hailort-dkms...")
        conn.sudo("apt-get install -y hailort-dkms", hide=True, pty=True)
        mark_done("packages_installed")
        console.print("  [green]✓[/green] Packages installed")

    # Step 3: Load kernel module
    console.print("\n[bold]Step 3: Loading kernel module...[/bold]")
    conn.sudo("modprobe hailo_pci", hide=True, warn=True, pty=True)

    # Verify device node
    result = conn.run("ls /dev/hailo0", hide=True, warn=True)
    if result.return_code == 0:
        console.print("  [green]✓[/green] /dev/hailo0 available")
    else:
        console.print("  [yellow]![/yellow] /dev/hailo0 not found - may need reboot")
        if not skip_reboot:
            console.print("\n[bold yellow]Reboot recommended to load DKMS module.[/bold yellow]")
            console.print(f"Run: ssh {config['user']}@{config['host']} 'sudo reboot'")
            console.print("Then re-run: uv run python deploy_hailo8.py install --skip-reboot")
            return

    # Step 4: Install Python bindings
    if is_done("python_installed"):
        console.print("[dim]Step 4: Python bindings already installed[/dim]")
    else:
        console.print("\n[bold]Step 4: Installing Python bindings...[/bold]")
        conn.run("pip install hailort --break-system-packages", hide=True)
        mark_done("python_installed")
        console.print("  [green]✓[/green] Python bindings installed")

    # Verify
    console.print("\n[bold]Verification:[/bold]")
    result = conn.run("hailortcli scan", hide=True, warn=True)
    if result.return_code == 0:
        console.print(result.stdout)
        console.print("\n[bold green]HailoRT installation complete![/bold green]")
    else:
        console.print("[red]Verification failed. Check logs above.[/red]")


@app.command()
def deploy():
    """Upload models and ROS2 node to robot."""
    console.print(Panel("Deploying Hailo Models and Node", style="bold blue"))

    conn = get_connection()
    config = load_config()
    home_dir = f"/home/{config['user']}"

    # Create remote directories
    console.print("\n[bold]Creating directories...[/bold]")
    conn.run(f"mkdir -p {home_dir}/landerpi/hailo/models", hide=True)
    conn.run(f"mkdir -p {home_dir}/landerpi/ros2_nodes/yolo_hailo", hide=True)

    # Upload models
    console.print("\n[bold]Uploading models...[/bold]")
    models_dir = HAILO_DIR / "models"
    hef_files = list(models_dir.glob("*.hef"))
    if hef_files:
        for hef_file in hef_files:
            remote_path = f"{home_dir}/landerpi/hailo/models/{hef_file.name}"
            console.print(f"  Uploading {hef_file.name}...")
            conn.put(str(hef_file), remote_path)
        console.print(f"  [green]✓[/green] Uploaded {len(hef_files)} model(s)")
    else:
        console.print("  [yellow]![/yellow] No HEF models found in hailo8-int/models/")
        console.print("  See docs/MODEL_CONVERSION.md for conversion instructions")

    # Upload ROS2 node
    console.print("\n[bold]Uploading ROS2 node...[/bold]")
    node_dir = HAILO_DIR / "ros2_nodes" / "yolo_hailo"
    if node_dir.exists():
        for item in node_dir.rglob("*"):
            if item.is_file() and "__pycache__" not in str(item):
                rel_path = item.relative_to(node_dir)
                remote_path = f"{home_dir}/landerpi/ros2_nodes/yolo_hailo/{rel_path}"
                remote_dir = str(Path(remote_path).parent)
                conn.run(f"mkdir -p {remote_dir}", hide=True)
                conn.put(str(item), remote_path)
                console.print(f"  [dim]{rel_path}[/dim]")
        console.print("  [green]✓[/green] ROS2 node uploaded")
    else:
        console.print("  [red]✗[/red] ROS2 node directory not found")

    # Upload hazard config
    console.print("\n[bold]Uploading config...[/bold]")
    config_src = Path(__file__).parent / "config" / "yolo_hazards.json"
    if config_src.exists():
        conn.put(str(config_src), f"{home_dir}/landerpi/config/yolo_hazards.json")
        console.print("  [green]✓[/green] Hazard config uploaded")

    console.print("\n[bold green]Deployment complete![/bold green]")
    console.print("\nTo use Hailo-accelerated YOLO:")
    console.print("  1. Ensure ROS2 stack is running: uv run python deploy_ros2_stack.py deploy")
    console.print("  2. Launch with Hailo: use_hailo:=true in launch config")


@app.command()
def test(
    benchmark: bool = typer.Option(False, "--benchmark", help="Run hailortcli benchmark"),
):
    """Run Hailo validation test."""
    console.print(Panel("Running Hailo Validation Test", style="bold blue"))

    conn = get_connection()
    config = load_config()
    home_dir = f"/home/{config['user']}"

    # Check device is accessible
    console.print("\n[bold]Device Check:[/bold]")
    result = conn.run("hailortcli scan", hide=True, warn=True)
    if result.return_code != 0:
        console.print("[red]Hailo device not accessible. Run 'deploy_hailo8.py install' first.[/red]")
        return
    console.print("  [green]✓[/green] Device accessible")

    # Check model exists
    console.print("\n[bold]Model Check:[/bold]")
    model_path = f"{home_dir}/landerpi/hailo/models/yolo11n_hailo.hef"
    result = conn.run(f"ls {model_path}", hide=True, warn=True)
    if result.return_code != 0:
        console.print(f"  [yellow]![/yellow] Model not found: {model_path}")
        console.print("  Run 'deploy_hailo8.py deploy' after converting model")
        # Continue with other tests
    else:
        console.print(f"  [green]✓[/green] Model found: {model_path}")

        # Run benchmark if requested and model exists
        if benchmark:
            console.print("\n[bold]Running Benchmark...[/bold]")
            result = conn.run(f"hailortcli benchmark {model_path}", hide=False, warn=True)
            if result.return_code == 0:
                console.print("\n[green]Benchmark complete![/green]")
            else:
                console.print("[red]Benchmark failed[/red]")

    # Test Python import and basic device access
    console.print("\n[bold]Python Integration Test:[/bold]")
    test_script = '''
import sys
try:
    from hailo_platform import HEF, VDevice
    print("  ✓ hailo_platform imported")

    vdevice = VDevice()
    print("  ✓ VDevice created")

    # Try loading HEF if path provided
    if len(sys.argv) > 1:
        import os
        if os.path.exists(sys.argv[1]):
            hef = HEF(sys.argv[1])
            print(f"  ✓ HEF loaded: {hef.get_input_vstream_infos()[0].shape}")

    print("\\n  [SUCCESS] Hailo integration working!")
except Exception as e:
    print(f"  ✗ Error: {e}")
    sys.exit(1)
'''
    result = conn.run(f"python3 -c '{test_script}' {model_path}", hide=False, warn=True)

    if result.return_code == 0:
        console.print("\n[bold green]All tests passed![/bold green]")
    else:
        console.print("\n[bold yellow]Some tests failed. Check output above.[/bold yellow]")


if __name__ == "__main__":
    app()
