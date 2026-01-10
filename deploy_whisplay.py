#!/usr/bin/env python3
"""
Deploy Whisplay HAT driver to LanderPi Robot.

Installs the PiSugar Whisplay HAT driver which provides:
- LCD screen display
- Physical buttons
- LED indicators
- Audio functions (WM8960 codec)

Usage:
    uv run python deploy_whisplay.py check     # Check Whisplay HAT status
    uv run python deploy_whisplay.py install   # Install Whisplay HAT driver
    uv run python deploy_whisplay.py test      # Run hardware test
    uv run python deploy_whisplay.py status    # Show audio device info

Notes:
    - Requires reboot after installation
    - Audio uses WM8960 codec (I2S)
    - GPIO pins used: I2C, SPI, I2S
"""

import json
import sys
from pathlib import Path

import typer
from fabric import Connection
from rich.console import Console
from rich.panel import Panel
from rich.prompt import Confirm
from rich.table import Table

app = typer.Typer(help="Deploy Whisplay HAT driver to LanderPi Robot")
console = Console()

CONFIG_FILE = Path(__file__).parent / "config.json"
WHISPLAY_DIR = Path(__file__).parent / "whisplay"
REMOTE_WHISPLAY_DIR = "~/whisplay"


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
    """Check Whisplay HAT hardware status (read-only)."""
    console.print(Panel("Checking Whisplay HAT Status", style="bold blue"))

    conn = get_connection()

    # Check I2C devices (WM8960 is at 0x1a)
    console.print("\n[bold]I2C Devices:[/bold]")
    result = conn.run("i2cdetect -y 1 2>/dev/null || echo 'i2c-tools not installed'", hide=True, warn=True)
    if "i2c-tools not installed" not in result.stdout:
        console.print(result.stdout)
        if "1a" in result.stdout.lower():
            console.print("  [green]WM8960 codec detected at 0x1a[/green]")
        else:
            console.print("  [yellow]WM8960 codec (0x1a) not detected - HAT may not be connected[/yellow]")
    else:
        console.print("  [yellow]i2c-tools not installed[/yellow]")

    # Check audio devices
    console.print("\n[bold]Audio Devices:[/bold]")
    result = conn.run("aplay -l 2>/dev/null", hide=True, warn=True)
    if result.return_code == 0:
        console.print(result.stdout)
        if "wm8960" in result.stdout.lower():
            console.print("  [green]WM8960 audio device found[/green]")
        else:
            console.print("  [yellow]WM8960 not in audio devices - driver may not be installed[/yellow]")
    else:
        console.print("  [red]No audio devices found[/red]")

    # Check if driver is installed
    console.print("\n[bold]Driver Installation:[/bold]")
    result = conn.run("test -d ~/Whisplay && echo 'installed' || echo 'not installed'", hide=True, warn=True)
    if "installed" in result.stdout and "not" not in result.stdout:
        console.print("  [green]Whisplay repository cloned[/green]")
    else:
        console.print("  [yellow]Whisplay repository not found[/yellow]")

    # Check kernel modules
    console.print("\n[bold]Kernel Modules:[/bold]")
    result = conn.run("lsmod | grep -i snd_soc_wm8960", hide=True, warn=True)
    if result.return_code == 0:
        console.print(f"  [green]WM8960 driver loaded[/green]")
        console.print(f"  {result.stdout.strip()}")
    else:
        console.print("  [yellow]WM8960 kernel module not loaded[/yellow]")

    # Check SPI (for LCD)
    console.print("\n[bold]SPI Interface:[/bold]")
    result = conn.run("ls /dev/spidev* 2>/dev/null || echo 'none'", hide=True, warn=True)
    if "none" not in result.stdout:
        console.print(f"  [green]SPI enabled:[/green] {result.stdout.strip()}")
    else:
        console.print("  [yellow]SPI not enabled[/yellow]")


@app.command()
def install(
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval prompt"),
):
    """Install Whisplay HAT driver on the robot."""
    console.print(Panel("Installing Whisplay HAT Driver", style="bold blue"))

    if not skip_approval:
        console.print("\n[bold yellow]This will:[/bold yellow]")
        console.print("  1. Install prerequisites (git, i2c-tools, swig, liblgpio-dev)")
        console.print("  2. Clone Whisplay repository from GitHub")
        console.print("  3. Install WM8960 audio driver (I2S/I2C)")
        console.print("  4. Install Python dependencies (pillow, pygame, spidev, rpi-lgpio)")
        console.print("  5. Fix test scripts for Ubuntu (python3)")
        console.print("  6. [bold red]Require a REBOOT to complete[/bold red]")
        if not Confirm.ask("\nContinue with installation?", default=False):
            console.print("[blue]Cancelled[/blue]")
            sys.exit(0)

    conn = get_connection()

    config = load_config()
    password = config.get("password", "")

    # Step 1: Install prerequisites
    console.print("\n[bold]Step 1: Installing prerequisites...[/bold]")
    # Check if prerequisites are already installed
    git_check = conn.run("which git", hide=True, warn=True)
    i2c_check = conn.run("which i2cdetect", hide=True, warn=True)

    if git_check.return_code == 0 and i2c_check.return_code == 0:
        console.print("  [green]Prerequisites already installed[/green]")
    else:
        console.print("  [yellow]Installing git and i2c-tools...[/yellow]")
        result = conn.run(
            f"echo '{password}' | sudo -S apt-get update && echo '{password}' | sudo -S apt-get install -y git i2c-tools swig liblgpio-dev",
            hide=True, warn=True, pty=True
        )
        if result.return_code == 0:
            console.print("  [green]Prerequisites installed[/green]")
        else:
            console.print(f"  [yellow]Warning: Some prerequisites may need manual install[/yellow]")

    # Step 2: Clone Whisplay repository
    console.print("\n[bold]Step 2: Cloning Whisplay repository...[/bold]")
    result = conn.run("test -d ~/Whisplay && echo 'exists'", hide=True, warn=True)
    if "exists" in result.stdout:
        console.print("  [yellow]Repository already exists, updating...[/yellow]")
        result = conn.run("cd ~/Whisplay && git pull", hide=True, warn=True)
    else:
        result = conn.run("git clone https://github.com/PiSugar/Whisplay.git --depth 1 ~/Whisplay", hide=True, warn=True)

    if result.return_code == 0:
        console.print("  [green]Repository ready[/green]")
    else:
        console.print(f"  [red]Failed to clone repository: {result.stderr}[/red]")
        sys.exit(1)

    # Step 3: Upload and run custom Ubuntu installer
    console.print("\n[bold]Step 3: Uploading Ubuntu-compatible installer...[/bold]")
    local_installer = WHISPLAY_DIR / "install_wm8960_ubuntu.sh"
    if not local_installer.exists():
        console.print(f"  [red]Error: {local_installer} not found[/red]")
        sys.exit(1)

    # Get home directory for remote path (~ not expanded by put)
    remote_home = f"/home/{config['user']}"
    remote_installer = f"{remote_home}/Whisplay/Driver/install_wm8960_ubuntu.sh"

    conn.put(str(local_installer), remote_installer)
    conn.run(f"chmod +x {remote_installer}", hide=True)
    console.print("  [green]Installer uploaded[/green]")

    # Step 4: Install WM8960 driver
    console.print("\n[bold]Step 4: Installing WM8960 audio driver...[/bold]")
    result = conn.run(
        "cd ~/Whisplay/Driver && sudo bash install_wm8960_ubuntu.sh",
        hide=False,  # Show output for debugging
        warn=True,
        pty=True
    )

    if result.return_code == 0:
        console.print("  [green]WM8960 driver installed[/green]")
    else:
        console.print(f"  [red]Driver installation failed[/red]")
        console.print(f"  Check output above for errors")
        sys.exit(1)

    # Step 5: Install Python dependencies for LCD/LED tests (system-wide for sudo access)
    # Note: rpi-lgpio is a drop-in replacement for RPi.GPIO that works on Ubuntu 25.10+
    console.print("\n[bold]Step 5: Installing Python dependencies...[/bold]")
    result = conn.run(
        f"echo '{password}' | sudo -S pip3 install pillow pygame spidev rpi-lgpio --break-system-packages",
        hide=True, warn=True, pty=True
    )
    if result.return_code == 0:
        console.print("  [green]Python dependencies installed[/green]")
    else:
        console.print("  [yellow]Warning: Some Python dependencies may need manual install[/yellow]")

    # Step 6: Fix test scripts to use python3 (Ubuntu doesn't have 'python')
    console.print("\n[bold]Step 6: Fixing test scripts for Ubuntu...[/bold]")
    conn.run(
        "sed -i 's/AUDIODEV=hw:\\$card_index,0 python /AUDIODEV=hw:\\$card_index,0 python3 /' ~/Whisplay/example/run_test.sh",
        hide=True, warn=True
    )
    console.print("  [green]Test scripts updated to use python3[/green]")

    # Step 7: Create local whisplay directory marker
    console.print("\n[bold]Step 7: Creating local marker...[/bold]")
    marker_file = WHISPLAY_DIR / ".installed"
    marker_file.write_text(f"Installed on robot: {load_config()['host']}\n")
    console.print("  [green]Marker created[/green]")

    console.print("\n" + "=" * 50)
    console.print("[bold green]Installation complete![/bold green]")
    console.print("[bold yellow]REBOOT REQUIRED[/bold yellow] - Run: ssh into robot and 'sudo reboot'")
    console.print("After reboot, run: [cyan]uv run python deploy_whisplay.py check[/cyan]")


@app.command()
def test(
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval prompt"),
):
    """Run Whisplay HAT hardware test."""
    console.print(Panel("Testing Whisplay HAT", style="bold blue"))

    if not skip_approval:
        console.print("\n[bold yellow]This test will:[/bold yellow]")
        console.print("  - Display test images on LCD")
        console.print("  - Light up RGB LEDs")
        console.print("  - Respond to button presses")
        if not Confirm.ask("\nRun hardware test?", default=False):
            console.print("[blue]Cancelled[/blue]")
            sys.exit(0)

    conn = get_connection()

    # Check if Whisplay is installed
    result = conn.run("test -d ~/Whisplay/example && echo 'ok'", hide=True, warn=True)
    if "ok" not in result.stdout:
        console.print("[red]Whisplay not installed. Run 'install' first.[/red]")
        sys.exit(1)

    console.print("\n[bold]Running LCD/Button/LED test...[/bold]")
    console.print("[yellow]Press buttons on the HAT to see colors change[/yellow]")
    console.print("[yellow]Press Ctrl+C to stop the test[/yellow]\n")

    try:
        conn.run(
            "cd ~/Whisplay/example && sudo bash run_test.sh",
            pty=True,
            warn=True
        )
    except KeyboardInterrupt:
        console.print("\n[blue]Test stopped[/blue]")


@app.command()
def mic_test(
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval prompt"),
):
    """Test microphone and speaker (records 10 seconds then plays back)."""
    console.print(Panel("Testing Microphone and Speaker", style="bold blue"))

    if not skip_approval:
        console.print("\n[bold yellow]This test will:[/bold yellow]")
        console.print("  - Record 10 seconds of audio from microphone")
        console.print("  - Play back the recording through speakers")
        if not Confirm.ask("\nRun microphone test?", default=False):
            console.print("[blue]Cancelled[/blue]")
            sys.exit(0)

    conn = get_connection()

    # Check if Whisplay is installed
    result = conn.run("test -d ~/Whisplay/example && echo 'ok'", hide=True, warn=True)
    if "ok" not in result.stdout:
        console.print("[red]Whisplay not installed. Run 'install' first.[/red]")
        sys.exit(1)

    console.print("\n[bold]Running microphone test...[/bold]")
    console.print("[yellow]Speak into the microphone for 10 seconds[/yellow]\n")

    try:
        conn.run(
            "cd ~/Whisplay/example && sudo bash mic_test.sh",
            pty=True,
            warn=True
        )
    except KeyboardInterrupt:
        console.print("\n[blue]Test stopped[/blue]")


@app.command()
def status():
    """Show detailed audio device status."""
    console.print(Panel("Whisplay HAT Audio Status", style="bold blue"))

    conn = get_connection()

    # Audio playback devices
    console.print("\n[bold]Playback Devices (aplay -l):[/bold]")
    result = conn.run("aplay -l", hide=True, warn=True)
    console.print(result.stdout if result.stdout else "  No playback devices")

    # Audio capture devices
    console.print("\n[bold]Capture Devices (arecord -l):[/bold]")
    result = conn.run("arecord -l", hide=True, warn=True)
    console.print(result.stdout if result.stdout else "  No capture devices")

    # ALSA mixer controls
    console.print("\n[bold]Mixer Controls:[/bold]")
    result = conn.run("amixer -c 1 scontrols 2>/dev/null || amixer scontrols", hide=True, warn=True)
    console.print(result.stdout if result.stdout else "  No mixer controls found")

    # Show table summary
    table = Table(title="Audio Summary")
    table.add_column("Component", style="cyan")
    table.add_column("Status", style="green")

    # Check WM8960
    result = conn.run("aplay -l | grep -i wm8960", hide=True, warn=True)
    wm8960_status = "Found" if result.return_code == 0 else "Not found"
    table.add_row("WM8960 Codec", wm8960_status)

    # Check I2C
    result = conn.run("i2cdetect -y 1 2>/dev/null | grep -q '1a' && echo 'ok'", hide=True, warn=True)
    i2c_status = "Connected" if "ok" in result.stdout else "Not detected"
    table.add_row("I2C (0x1a)", i2c_status)

    # Check kernel module
    result = conn.run("lsmod | grep -q snd_soc_wm8960 && echo 'ok'", hide=True, warn=True)
    module_status = "Loaded" if "ok" in result.stdout else "Not loaded"
    table.add_row("Kernel Module", module_status)

    console.print("\n")
    console.print(table)


@app.command()
def uninstall(
    skip_approval: bool = typer.Option(False, "--yes", "-y", help="Skip approval prompt"),
):
    """Remove Whisplay HAT driver from the robot."""
    console.print(Panel("Uninstalling Whisplay HAT Driver", style="bold red"))

    if not skip_approval:
        console.print("\n[bold yellow]This will:[/bold yellow]")
        console.print("  1. Remove Whisplay repository")
        console.print("  2. Note: Kernel driver may remain until reboot")
        if not Confirm.ask("\nContinue with uninstall?", default=False):
            console.print("[blue]Cancelled[/blue]")
            sys.exit(0)

    conn = get_connection()

    console.print("\n[bold]Removing Whisplay repository...[/bold]")
    result = conn.run("rm -rf ~/Whisplay", hide=True, warn=True)
    if result.return_code == 0:
        console.print("  [green]Repository removed[/green]")
    else:
        console.print(f"  [yellow]Warning: {result.stderr}[/yellow]")

    # Remove local marker
    marker_file = WHISPLAY_DIR / ".installed"
    if marker_file.exists():
        marker_file.unlink()
        console.print("  [green]Local marker removed[/green]")

    console.print("\n[bold green]Uninstall complete[/bold green]")
    console.print("[yellow]Note: Reboot may be required to fully unload driver[/yellow]")


if __name__ == "__main__":
    app()
