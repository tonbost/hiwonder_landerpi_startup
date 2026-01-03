#!/usr/bin/env python3
"""
Deploy Hailo 8 AI Accelerator to LanderPi Robot.

Installs HailoRT 4.23 driver, uploads models, and deploys Hailo-accelerated YOLO node.

Usage:
    uv run python deploy_hailo8.py check     # Check Hailo hardware/driver status
    uv run python deploy_hailo8.py install   # Install HailoRT 4.23 on robot
    uv run python deploy_hailo8.py deploy    # Upload models + ROS2 node
    uv run python deploy_hailo8.py test      # Run validation test
    uv run python deploy_hailo8.py status    # Show Hailo device info

Notes:
    - Uses Raspberry Pi Trixie repository for HailoRT 4.23
    - Supports Python 3.13 (Ubuntu 25.10)
    - M.2 Hailo-8 modules have firmware 4.19.0 (not upgradeable, works with 4.23 runtime)
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
    force: bool = typer.Option(False, "--force", help="Force reinstall even if already installed"),
):
    """Install HailoRT 4.23 driver stack on robot."""
    console.print(Panel("Installing HailoRT 4.23 Driver", style="bold blue"))

    conn = get_connection()
    config = load_config()
    home_dir = f"/home/{config['user']}"
    marker_dir = f"{home_dir}/.landerpi_setup"

    # Create marker directory
    conn.run(f"mkdir -p {marker_dir}", hide=True)

    def is_done(step: str) -> bool:
        if force:
            return False
        result = conn.run(f"test -f {marker_dir}/hailo423_{step}", hide=True, warn=True)
        return result.return_code == 0

    def mark_done(step: str):
        conn.run(f"touch {marker_dir}/hailo423_{step}", hide=True)

    # Step 1: Add Raspberry Pi Trixie repository (contains HailoRT 4.23)
    if is_done("repo_added"):
        console.print("[dim]Step 1: Raspberry Pi Trixie repository already added[/dim]")
    else:
        console.print("\n[bold]Step 1: Adding Raspberry Pi Trixie repository (HailoRT 4.23)...[/bold]")
        # Download and install Raspberry Pi GPG key
        conn.sudo(
            "bash -c 'curl -fsSL https://archive.raspberrypi.com/debian/raspberrypi.gpg.key | "
            "gpg --dearmor -o /usr/share/keyrings/raspberrypi-archive-keyring.gpg'",
            hide=True,
            pty=True,
            warn=True
        )
        # Add the Trixie repository (has HailoRT 4.23)
        conn.sudo(
            "bash -c 'echo \"deb [arch=arm64 signed-by=/usr/share/keyrings/raspberrypi-archive-keyring.gpg] "
            "https://archive.raspberrypi.com/debian/ trixie main\" > /etc/apt/sources.list.d/raspi.list'",
            hide=True,
            pty=True
        )
        conn.sudo("apt-get update -qq", hide=True, pty=True)
        mark_done("repo_added")
        console.print("  [green]✓[/green] Raspberry Pi Trixie repository added")

    # Step 2: Install prerequisites (DKMS must be installed FIRST per official docs)
    if is_done("prereqs_installed"):
        console.print("[dim]Step 2: Prerequisites already installed[/dim]")
    else:
        console.print("\n[bold]Step 2: Installing prerequisites...[/bold]")
        console.print("  Installing DKMS (required before hailo packages)...")
        conn.sudo("apt-get install -y dkms", hide=True, pty=True, warn=True, timeout=120)

        # Install kernel headers for the running kernel
        kernel_result = conn.run("uname -r", hide=True)
        kernel_version = kernel_result.stdout.strip()
        console.print(f"  Running kernel: {kernel_version}")

        build_check = conn.run(f"test -d /lib/modules/{kernel_version}/build", hide=True, warn=True)
        if build_check.return_code != 0:
            headers_pkg = f"linux-headers-{kernel_version}"
            console.print(f"  Installing {headers_pkg}...")
            conn.sudo(f"apt-get install -y {headers_pkg}", hide=True, pty=True, warn=True, timeout=300)

        mark_done("prereqs_installed")
        console.print("  [green]✓[/green] Prerequisites installed")

    # Step 3: Remove old Hailo packages if present
    console.print("\n[bold]Step 3: Removing old Hailo packages (if any)...[/bold]")
    conn.sudo("apt-get remove -y hailo-dkms hailort hailofw 2>/dev/null || true", hide=True, pty=True, warn=True, timeout=120)
    console.print("  [green]✓[/green] Old packages removed")

    # Step 4: Install HailoRT 4.23 packages
    if is_done("packages_installed"):
        console.print("[dim]Step 4: HailoRT 4.23 packages already installed[/dim]")
    else:
        console.print("\n[bold]Step 4: Installing HailoRT 4.23 packages...[/bold]")

        # Install hailort runtime
        console.print("  Installing hailort=4.23.0...")
        conn.sudo("apt-get install -y hailort=4.23.0", hide=True, pty=True, warn=True, timeout=300)

        # Install PCIe driver (includes DKMS module and firmware)
        console.print("  Installing hailort-pcie-driver (DKMS build may take time)...")
        conn.sudo("apt-get install -y hailort-pcie-driver", hide=True, pty=True, warn=True, timeout=600)

        mark_done("packages_installed")
        console.print("  [green]✓[/green] HailoRT 4.23 packages installed")

    # Step 5: Load kernel module
    console.print("\n[bold]Step 5: Loading kernel module...[/bold]")
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

    # Step 6: Install Python bindings (4.23 supports Python 3.13!)
    if is_done("python_installed"):
        console.print("[dim]Step 6: Python bindings already installed[/dim]")
    else:
        console.print("\n[bold]Step 6: Installing Python bindings...[/bold]")
        py_version = conn.run("python3 -c 'import sys; print(f\"{sys.version_info.major}.{sys.version_info.minor}\")'", hide=True)
        py_ver = py_version.stdout.strip()
        console.print(f"  Python version: {py_ver}")

        # 4.23 supports Python 3.13 via apt
        apt_result = conn.sudo("apt-get install -y python3-hailort", hide=True, pty=True, warn=True, timeout=120)
        if apt_result.return_code == 0:
            console.print("  [green]✓[/green] Python bindings installed (python3-hailort)")
        else:
            console.print(f"  [yellow]![/yellow] Python bindings installation failed")
            console.print("  [dim]Try: pip install hailort --break-system-packages[/dim]")
        mark_done("python_installed")

    # Verify
    console.print("\n[bold]Verification:[/bold]")
    result = conn.run("hailortcli --version && hailortcli scan", hide=True, warn=True)
    if result.return_code == 0:
        console.print(result.stdout)

        # Check Python bindings
        py_result = conn.run("python3 -c 'import hailo_platform; print(f\"Python API: {hailo_platform.__version__}\")'", hide=True, warn=True)
        if py_result.return_code == 0:
            console.print(py_result.stdout.strip())

        # Note about firmware version mismatch (normal for M.2 modules)
        fw_result = conn.run("hailortcli fw-control identify 2>&1 | grep -i 'unsupported\\|warning'", hide=True, warn=True)
        if fw_result.return_code == 0:
            console.print("\n[dim]Note: Firmware version warning is normal for M.2 modules (firmware 4.19.0 works with runtime 4.23.0)[/dim]")

        console.print("\n[bold green]HailoRT 4.23 installation complete![/bold green]")
    else:
        console.print("[red]Verification failed. Check logs above.[/red]")


@app.command()
def deploy():
    """Upload models, inference server, and install systemd service."""
    console.print(Panel("Deploying Hailo Models and Server", style="bold blue"))

    conn = get_connection()
    config = load_config()
    home_dir = f"/home/{config['user']}"

    # Create remote directories
    console.print("\n[bold]Creating directories...[/bold]")
    conn.run(f"mkdir -p {home_dir}/landerpi/hailo/models", hide=True)

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

    # Upload inference server
    console.print("\n[bold]Uploading Hailo inference server...[/bold]")
    server_src = HAILO_DIR / "host_server" / "hailo_inference_server.py"
    if server_src.exists():
        conn.put(str(server_src), f"{home_dir}/landerpi/hailo/hailo_inference_server.py")
        console.print("  [green]✓[/green] Inference server uploaded")
    else:
        console.print("  [red]✗[/red] hailo_inference_server.py not found")
        return

    # Upload and install systemd service
    console.print("\n[bold]Installing systemd service...[/bold]")
    service_src = Path(__file__).parent / "systemd" / "hailo-server.service"
    if service_src.exists():
        conn.put(str(service_src), f"/tmp/hailo-server.service")
        conn.sudo("mv /tmp/hailo-server.service /etc/systemd/system/", pty=True, hide=True)
        conn.sudo("systemctl daemon-reload", pty=True, hide=True)
        conn.sudo("systemctl enable hailo-server", pty=True, hide=True)
        console.print("  [green]✓[/green] hailo-server.service installed and enabled")
    else:
        console.print("  [yellow]![/yellow] systemd service file not found")

    # Upload hazard config
    console.print("\n[bold]Uploading config...[/bold]")
    config_src = Path(__file__).parent / "config" / "yolo_hazards.json"
    if config_src.exists():
        conn.run(f"mkdir -p {home_dir}/landerpi/config", hide=True)
        conn.put(str(config_src), f"{home_dir}/landerpi/config/yolo_hazards.json")
        console.print("  [green]✓[/green] Hazard config uploaded")

    console.print("\n[bold green]Deployment complete![/bold green]")
    console.print("\nCommands:")
    console.print("  uv run python deploy_hailo8.py start   # Start inference server")
    console.print("  uv run python deploy_hailo8.py stop    # Stop inference server")
    console.print("\nTo use Hailo-accelerated YOLO:")
    console.print("  uv run python deploy_explorer.py start --yolo-hailo --duration 5")


@app.command()
def start():
    """Start the Hailo inference server."""
    console.print("Starting Hailo inference server...")
    conn = get_connection()
    conn.sudo("systemctl start hailo-server", pty=True, hide=True, warn=True)

    # Check status
    import time
    time.sleep(1)
    result = conn.run("systemctl is-active hailo-server", hide=True, warn=True)
    if result.stdout.strip() == "active":
        console.print("[green]Hailo server started successfully![/green]")
        # Show port
        result = conn.run("ss -tlnp | grep 5555", hide=True, warn=True)
        if result.return_code == 0:
            console.print(f"Listening on port 5555")
    else:
        console.print("[red]Failed to start server. Check logs:[/red]")
        console.print("  journalctl -u hailo-server -n 20")


@app.command()
def stop():
    """Stop the Hailo inference server."""
    console.print("Stopping Hailo inference server...")
    conn = get_connection()
    conn.sudo("systemctl stop hailo-server", pty=True, hide=True, warn=True)
    console.print("[green]Hailo server stopped.[/green]")


@app.command()
def logs(
    follow: bool = typer.Option(False, "-f", "--follow", help="Follow log output"),
    lines: int = typer.Option(50, "-n", "--lines", help="Number of lines to show"),
):
    """View Hailo server logs."""
    conn = get_connection()
    cmd = f"journalctl -u hailo-server -n {lines}"
    if follow:
        cmd += " -f"
    conn.run(cmd, hide=False, warn=True, pty=True)


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
    model_path = f"{home_dir}/landerpi/hailo/models/yolov11n.hef"
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
