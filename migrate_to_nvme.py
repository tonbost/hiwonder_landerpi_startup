#!/usr/bin/env python3
"""
Migrate LanderPi from SD card to NVMe drive.

This script automates:
1. Pre-flight checks (NVMe detection, space requirements)
2. rpi-clone installation and SD-to-NVMe cloning
3. Bootloader configuration for NVMe boot priority
4. PCIe Gen 3 enablement for maximum speed
"""

import json
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

import typer
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich import box
from fabric import Connection
from invoke.exceptions import UnexpectedExit

app = typer.Typer(help="Migrate LanderPi from SD card to NVMe drive")
console = Console()


@dataclass
class NVMeStatus:
    """Status of NVMe migration readiness."""
    nvme_detected: bool = False
    nvme_device: str = ""
    nvme_size_gb: float = 0.0
    nvme_model: str = ""
    sd_used_gb: float = 0.0
    sd_total_gb: float = 0.0
    boot_device: str = ""
    booting_from_nvme: bool = False
    rpi_clone_installed: bool = False
    pcie_gen3_enabled: bool = False
    boot_order: str = ""
    pcie_probe_enabled: bool = False
    bootloader_version: str = ""


def get_config() -> dict:
    """Load connection config from config.json."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        with open(config_path) as f:
            return json.load(f)
    return {}


def get_connection(host: Optional[str], user: Optional[str], password: Optional[str]) -> tuple[Connection, str, str, str]:
    """Create SSH connection from params or config.json."""
    config = get_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not all([host, user, password]):
        console.print("[red]Missing connection details. Use --host, --user, --password or config.json[/red]")
        raise typer.Exit(1)

    conn = Connection(host, user=user, connect_kwargs={"password": password})
    return conn, host, user, password


def run_cmd(conn: Connection, command: str, warn: bool = True) -> tuple[bool, str, str]:
    """Run a command and return (success, stdout, stderr)."""
    try:
        result = conn.run(command, hide=True, warn=warn)
        return result.ok, result.stdout.strip(), result.stderr.strip()
    except UnexpectedExit as e:
        return False, "", str(e)
    except Exception as e:
        return False, "", str(e)


def run_sudo(conn: Connection, command: str, password: str, warn: bool = True) -> tuple[bool, str, str]:
    """Run a sudo command and return (success, stdout, stderr)."""
    try:
        result = conn.sudo(command, hide=True, warn=warn, pty=True, password=password)
        return result.ok, result.stdout.strip(), result.stderr.strip()
    except UnexpectedExit as e:
        return False, "", str(e)
    except Exception as e:
        return False, "", str(e)


def gather_nvme_status(conn: Connection, password: str) -> NVMeStatus:
    """Gather comprehensive NVMe and boot status."""
    status = NVMeStatus()

    # Check for NVMe device
    ok, out, _ = run_cmd(conn, "ls /dev/nvme0n1 2>/dev/null")
    status.nvme_detected = ok and "/dev/nvme0n1" in out
    if status.nvme_detected:
        status.nvme_device = "/dev/nvme0n1"

        # Get NVMe size
        ok, out, _ = run_cmd(conn, "lsblk -b -d -n -o SIZE /dev/nvme0n1 2>/dev/null")
        if ok and out:
            try:
                status.nvme_size_gb = int(out) / (1024**3)
            except ValueError:
                pass

        # Get NVMe model
        ok, out, _ = run_cmd(conn, "lsblk -d -n -o MODEL /dev/nvme0n1 2>/dev/null")
        if ok:
            status.nvme_model = out.strip()

    # Get SD card usage
    ok, out, _ = run_cmd(conn, "df -BG / | tail -1 | awk '{print $3, $2}'")
    if ok and out:
        parts = out.replace("G", "").split()
        if len(parts) >= 2:
            try:
                status.sd_used_gb = float(parts[0])
                status.sd_total_gb = float(parts[1])
            except ValueError:
                pass

    # Check current boot device
    ok, out, _ = run_cmd(conn, "findmnt -n -o SOURCE /")
    if ok:
        status.boot_device = out.strip()
        status.booting_from_nvme = "nvme" in out.lower()

    # Check if rpi-clone is installed
    ok, _, _ = run_cmd(conn, "which rpi-clone")
    status.rpi_clone_installed = ok

    # Check PCIe Gen 3 setting
    ok, out, _ = run_cmd(conn, "grep -q 'pciex1_gen=3' /boot/firmware/config.txt && echo yes")
    status.pcie_gen3_enabled = ok and "yes" in out

    # Get bootloader config
    ok, out, _ = run_cmd(conn, "vcgencmd bootloader_config 2>/dev/null | grep BOOT_ORDER")
    if ok and out:
        status.boot_order = out.split("=")[1] if "=" in out else out

    ok, out, _ = run_cmd(conn, "vcgencmd bootloader_config 2>/dev/null | grep PCIE_PROBE")
    status.pcie_probe_enabled = ok and "PCIE_PROBE=1" in out

    # Get bootloader version
    ok, out, _ = run_cmd(conn, "vcgencmd bootloader_version 2>/dev/null | head -1")
    if ok:
        status.bootloader_version = out.strip()

    return status


def display_status(status: NVMeStatus):
    """Display NVMe migration status in a table."""
    table = Table(title="NVMe Migration Status", box=box.ROUNDED)
    table.add_column("Check", style="cyan")
    table.add_column("Status", style="bold")
    table.add_column("Details")

    # NVMe Detection
    if status.nvme_detected:
        table.add_row(
            "NVMe Detected",
            "[green]✓ Yes[/green]",
            f"{status.nvme_model} ({status.nvme_size_gb:.1f} GB)"
        )
    else:
        table.add_row("NVMe Detected", "[red]✗ No[/red]", "No NVMe drive found at /dev/nvme0n1")

    # SD Card Usage
    if status.sd_total_gb > 0:
        table.add_row(
            "SD Card Usage",
            f"[blue]{status.sd_used_gb:.1f}/{status.sd_total_gb:.1f} GB[/blue]",
            f"{(status.sd_used_gb/status.sd_total_gb)*100:.0f}% used"
        )

    # Space Check
    if status.nvme_detected and status.sd_used_gb > 0:
        if status.nvme_size_gb >= status.sd_used_gb * 1.2:  # 20% headroom
            table.add_row(
                "Space Available",
                "[green]✓ Sufficient[/green]",
                f"NVMe has {status.nvme_size_gb - status.sd_used_gb:.1f} GB free after clone"
            )
        else:
            table.add_row(
                "Space Available",
                "[red]✗ Insufficient[/red]",
                f"Need at least {status.sd_used_gb * 1.2:.1f} GB, have {status.nvme_size_gb:.1f} GB"
            )

    # Current Boot Device
    if status.booting_from_nvme:
        table.add_row("Boot Device", "[green]✓ NVMe[/green]", status.boot_device)
    else:
        table.add_row("Boot Device", "[yellow]SD Card[/yellow]", status.boot_device)

    # rpi-clone
    if status.rpi_clone_installed:
        table.add_row("rpi-clone", "[green]✓ Installed[/green]", "")
    else:
        table.add_row("rpi-clone", "[yellow]Not installed[/yellow]", "Will be installed during clone")

    # PCIe Gen 3
    if status.pcie_gen3_enabled:
        table.add_row("PCIe Gen 3", "[green]✓ Enabled[/green]", "Maximum NVMe speed")
    else:
        table.add_row("PCIe Gen 3", "[yellow]Disabled[/yellow]", "Can be enabled for faster speeds")

    # Boot Order
    boot_order_status = "[green]✓ NVMe first[/green]" if "6" in status.boot_order[:2] else "[yellow]SD first[/yellow]"
    table.add_row("Boot Order", boot_order_status, status.boot_order or "Unknown")

    # PCIE_PROBE
    if status.pcie_probe_enabled:
        table.add_row("PCIE_PROBE", "[green]✓ Enabled[/green]", "NVMe hat detection enabled")
    else:
        table.add_row("PCIE_PROBE", "[yellow]Disabled[/yellow]", "May need enabling for non-official hats")

    console.print(table)

    # Summary
    if status.booting_from_nvme:
        console.print("\n[green bold]✓ System is already booting from NVMe![/green bold]")
    elif status.nvme_detected:
        console.print("\n[yellow]System ready for migration. Run 'clone' to begin.[/yellow]")
    else:
        console.print("\n[red]NVMe drive not detected. Ensure it's properly connected.[/red]")


@app.command()
def status(
    host: str = typer.Option(None, help="Robot IP address"),
    user: str = typer.Option(None, help="SSH username"),
    password: str = typer.Option(None, help="SSH password"),
):
    """Check NVMe migration status and readiness."""
    conn, host, user, password = get_connection(host, user, password)

    console.print(Panel(f"Checking NVMe status on {user}@{host}", title="NVMe Status"))

    try:
        conn.run("echo connected", hide=True)
    except Exception as e:
        console.print(f"[red]Connection failed: {e}[/red]")
        raise typer.Exit(1)

    nvme_status = gather_nvme_status(conn, password)
    display_status(nvme_status)


@app.command()
def clone(
    host: str = typer.Option(None, help="Robot IP address"),
    user: str = typer.Option(None, help="SSH username"),
    password: str = typer.Option(None, help="SSH password"),
    yes: bool = typer.Option(False, "--yes", "-y", help="Skip confirmation prompts"),
):
    """Clone SD card to NVMe drive using rpi-clone."""
    conn, host, user, password = get_connection(host, user, password)

    console.print(Panel(f"Cloning SD to NVMe on {user}@{host}", title="NVMe Clone"))

    # Gather status
    nvme_status = gather_nvme_status(conn, password)

    # Pre-flight checks
    if not nvme_status.nvme_detected:
        console.print("[red]✗ NVMe drive not detected. Cannot proceed.[/red]")
        raise typer.Exit(1)

    if nvme_status.booting_from_nvme:
        console.print("[yellow]Already booting from NVMe. Clone not needed.[/yellow]")
        raise typer.Exit(0)

    if nvme_status.nvme_size_gb < nvme_status.sd_used_gb * 1.1:
        console.print(f"[red]✗ NVMe too small. Need {nvme_status.sd_used_gb * 1.1:.1f} GB, have {nvme_status.nvme_size_gb:.1f} GB[/red]")
        raise typer.Exit(1)

    # Show what we're about to do
    console.print(f"\n[bold]Clone Details:[/bold]")
    console.print(f"  Source: SD card ({nvme_status.sd_used_gb:.1f} GB used)")
    console.print(f"  Target: {nvme_status.nvme_model} ({nvme_status.nvme_size_gb:.1f} GB)")
    console.print(f"\n[yellow]WARNING: This will ERASE all data on the NVMe drive![/yellow]")

    if not yes:
        if not typer.confirm("Proceed with clone?"):
            console.print("Aborted.")
            raise typer.Exit(0)

    # Install rpi-clone if needed
    if not nvme_status.rpi_clone_installed:
        console.print("\n[bold]Installing rpi-clone...[/bold]")

        # Clone rpi-clone repo
        ok, _, err = run_cmd(conn, "test -d ~/rpi-clone || git clone https://github.com/billw2/rpi-clone.git ~/rpi-clone")
        if not ok:
            console.print(f"[red]Failed to clone rpi-clone: {err}[/red]")
            raise typer.Exit(1)

        # Install to /usr/local/sbin
        ok, _, err = run_sudo(conn, "cp ~/rpi-clone/rpi-clone ~/rpi-clone/rpi-clone-setup /usr/local/sbin/", password)
        if not ok:
            console.print(f"[red]Failed to install rpi-clone: {err}[/red]")
            raise typer.Exit(1)

        console.print("[green]✓ rpi-clone installed[/green]")

    # Ensure latest rpi-clone version
    console.print("\n[bold]Updating rpi-clone to latest version...[/bold]")
    run_cmd(conn, "cd ~/rpi-clone && git pull --quiet")
    run_sudo(conn, "cp ~/rpi-clone/rpi-clone ~/rpi-clone/rpi-clone-setup /usr/local/sbin/", password)

    # Create udev rules for NVMe partition naming workaround
    # rpi-clone expects nvme0n11/nvme0n12 but NVMe kernel uses nvme0n1p1/nvme0n1p2
    console.print("[bold]Setting up NVMe partition symlinks (udev rules)...[/bold]")
    run_sudo(conn, """bash -c '
cat > /etc/udev/rules.d/99-nvme-clone.rules << "UDEVRULES"
# Symlinks for rpi-clone NVMe compatibility
# Maps nvme0n1p* to nvme0n1* that rpi-clone expects
KERNEL=="nvme0n1p1", SYMLINK+="nvme0n11"
KERNEL=="nvme0n1p2", SYMLINK+="nvme0n12"
KERNEL=="nvme0n1p3", SYMLINK+="nvme0n13"
KERNEL=="nvme0n1p4", SYMLINK+="nvme0n14"
UDEVRULES
udevadm control --reload-rules
udevadm trigger
'""", password)
    console.print("[green]✓ udev rules installed[/green]")

    # Run rpi-clone
    console.print("\n[bold]Starting clone process...[/bold]")
    console.print("[dim]This may take 10-30 minutes depending on data size.[/dim]")
    console.print("[dim]The clone will partition and sync all data to NVMe.[/dim]\n")

    # rpi-clone flags:
    # -f = force (don't ask for confirmation)
    # -U = unattended mode
    try:
        # Run with timeout of 1 hour
        result = conn.sudo(
            "rpi-clone nvme0n1 -f -U",
            password=password,
            pty=True,
            hide=False,  # Show output
            warn=True,
            timeout=3600  # 1 hour timeout
        )
        if result.ok:
            console.print("\n[green]✓ Clone completed successfully![/green]")
        else:
            console.print(f"\n[red]Clone may have failed. Check output above.[/red]")
            raise typer.Exit(1)
    except Exception as e:
        console.print(f"\n[red]Clone error: {e}[/red]")
        raise typer.Exit(1)

    console.print("\n[bold]Next steps:[/bold]")
    console.print("  1. Run 'configure' to set boot order and PCIe Gen 3")
    console.print("  2. Reboot the Pi")
    console.print("  3. Run 'verify' to confirm NVMe boot")


@app.command()
def configure(
    host: str = typer.Option(None, help="Robot IP address"),
    user: str = typer.Option(None, help="SSH username"),
    password: str = typer.Option(None, help="SSH password"),
    pcie_gen3: bool = typer.Option(True, help="Enable PCIe Gen 3 for maximum speed"),
    yes: bool = typer.Option(False, "--yes", "-y", help="Skip confirmation prompts"),
):
    """Configure bootloader for NVMe boot and enable PCIe Gen 3."""
    conn, host, user, password = get_connection(host, user, password)

    console.print(Panel(f"Configuring NVMe boot on {user}@{host}", title="NVMe Configure"))

    nvme_status = gather_nvme_status(conn, password)

    if not nvme_status.nvme_detected:
        console.print("[red]✗ NVMe drive not detected.[/red]")
        raise typer.Exit(1)

    changes = []

    # Check what needs to be configured
    if "6" not in nvme_status.boot_order[:2]:
        changes.append("Set boot order to NVMe first (0xf416)")

    if not nvme_status.pcie_probe_enabled:
        changes.append("Enable PCIE_PROBE=1 for NVMe hat detection")

    if pcie_gen3 and not nvme_status.pcie_gen3_enabled:
        changes.append("Enable PCIe Gen 3 in config.txt")

    if not changes:
        console.print("[green]✓ All configurations already applied![/green]")
        raise typer.Exit(0)

    console.print("\n[bold]Changes to apply:[/bold]")
    for change in changes:
        console.print(f"  • {change}")

    if not yes:
        if not typer.confirm("\nApply these changes?"):
            console.print("Aborted.")
            raise typer.Exit(0)

    # Update bootloader config
    if "6" not in nvme_status.boot_order[:2] or not nvme_status.pcie_probe_enabled:
        console.print("\n[bold]Updating bootloader configuration...[/bold]")

        # Extract current config
        ok, _, err = run_cmd(conn, "vcgencmd bootloader_config > ~/bootconf.txt")
        if not ok:
            console.print(f"[red]Failed to extract bootloader config: {err}[/red]")
            raise typer.Exit(1)

        # Update BOOT_ORDER
        ok, _, _ = run_cmd(conn, "sed -i 's/^BOOT_ORDER=.*/BOOT_ORDER=0xf416/' ~/bootconf.txt")

        # Ensure BOOT_ORDER exists if it wasn't there
        ok, out, _ = run_cmd(conn, "grep -q '^BOOT_ORDER=' ~/bootconf.txt || echo 'BOOT_ORDER=0xf416' >> ~/bootconf.txt")

        # Add PCIE_PROBE if missing
        ok, _, _ = run_cmd(conn, "grep -q '^PCIE_PROBE=' ~/bootconf.txt || echo 'PCIE_PROBE=1' >> ~/bootconf.txt")
        # Ensure PCIE_PROBE=1
        ok, _, _ = run_cmd(conn, "sed -i 's/^PCIE_PROBE=.*/PCIE_PROBE=1/' ~/bootconf.txt")

        # Apply the config
        ok, out, err = run_sudo(conn, "rpi-eeprom-config --apply ~/bootconf.txt", password)
        if not ok:
            console.print(f"[red]Failed to apply bootloader config: {err}[/red]")
            console.print("[yellow]Try running: sudo rpi-eeprom-config --edit[/yellow]")
            raise typer.Exit(1)

        console.print("[green]✓ Bootloader configuration updated[/green]")

    # Enable PCIe Gen 3
    if pcie_gen3 and not nvme_status.pcie_gen3_enabled:
        console.print("\n[bold]Enabling PCIe Gen 3...[/bold]")

        # Check if line already exists
        ok, out, _ = run_cmd(conn, "grep -q 'pciex1_gen=' /boot/firmware/config.txt && echo exists")

        if "exists" in out:
            # Update existing line
            ok, _, err = run_sudo(conn, "sed -i 's/^dtparam=pciex1_gen=.*/dtparam=pciex1_gen=3/' /boot/firmware/config.txt", password)
        else:
            # Add new line
            ok, _, err = run_sudo(conn, "bash -c 'echo \"\\n# Enable PCIe Gen 3 speeds\\ndtparam=pciex1_gen=3\" >> /boot/firmware/config.txt'", password)

        if ok:
            console.print("[green]✓ PCIe Gen 3 enabled[/green]")
        else:
            console.print(f"[yellow]Warning: Could not enable PCIe Gen 3: {err}[/yellow]")

    console.print("\n[bold green]Configuration complete![/bold green]")
    console.print("\n[bold]Next steps:[/bold]")
    console.print("  1. Reboot: [cyan]sudo reboot[/cyan]")
    console.print("  2. After reboot, run: [cyan]uv run python migrate_to_nvme.py verify[/cyan]")


@app.command()
def verify(
    host: str = typer.Option(None, help="Robot IP address"),
    user: str = typer.Option(None, help="SSH username"),
    password: str = typer.Option(None, help="SSH password"),
):
    """Verify system is booting from NVMe with optimal settings."""
    conn, host, user, password = get_connection(host, user, password)

    console.print(Panel(f"Verifying NVMe boot on {user}@{host}", title="NVMe Verify"))

    nvme_status = gather_nvme_status(conn, password)

    all_ok = True

    # Check boot device
    if nvme_status.booting_from_nvme:
        console.print(f"[green]✓ Booting from NVMe[/green] ({nvme_status.boot_device})")
    else:
        console.print(f"[red]✗ NOT booting from NVMe[/red] (current: {nvme_status.boot_device})")
        all_ok = False

    # Check PCIe Gen 3
    if nvme_status.pcie_gen3_enabled:
        console.print("[green]✓ PCIe Gen 3 enabled[/green]")
    else:
        console.print("[yellow]⚠ PCIe Gen 3 not enabled[/yellow] (using Gen 2 speeds)")

    # Check boot order
    if "6" in nvme_status.boot_order[:2]:
        console.print(f"[green]✓ Boot order correct[/green] ({nvme_status.boot_order})")
    else:
        console.print(f"[yellow]⚠ Boot order not optimal[/yellow] ({nvme_status.boot_order})")

    # Check PCIE_PROBE
    if nvme_status.pcie_probe_enabled:
        console.print("[green]✓ PCIE_PROBE enabled[/green]")
    else:
        console.print("[yellow]⚠ PCIE_PROBE not enabled[/yellow]")

    # Run speed test
    console.print("\n[bold]Running NVMe speed test...[/bold]")

    # Write test
    ok, out, _ = run_cmd(conn, "dd if=/dev/zero of=/tmp/nvme_test bs=1M count=256 conv=fsync 2>&1 | tail -1")
    if ok and out:
        console.print(f"  Write: {out}")

    # Read test
    ok, out, _ = run_cmd(conn, "dd if=/tmp/nvme_test of=/dev/null bs=1M 2>&1 | tail -1")
    if ok and out:
        console.print(f"  Read:  {out}")

    # Cleanup
    run_cmd(conn, "rm -f /tmp/nvme_test")

    # Summary
    if all_ok:
        console.print("\n[bold green]✓ NVMe migration verified successfully![/bold green]")
        console.print("\nYour Pi is now booting from NVMe. The SD card can be removed or kept as backup.")
    else:
        console.print("\n[bold red]✗ NVMe migration incomplete[/bold red]")
        console.print("\nRun 'configure' and reboot to complete the migration.")


@app.command()
def update_eeprom(
    host: str = typer.Option(None, help="Robot IP address"),
    user: str = typer.Option(None, help="SSH username"),
    password: str = typer.Option(None, help="SSH password"),
    yes: bool = typer.Option(False, "--yes", "-y", help="Skip confirmation prompts"),
):
    """Update bootloader EEPROM to latest version."""
    conn, host, user, password = get_connection(host, user, password)

    console.print(Panel(f"Updating EEPROM on {user}@{host}", title="EEPROM Update"))

    # Check for updates
    console.print("[bold]Checking for EEPROM updates...[/bold]")
    ok, out, _ = run_sudo(conn, "rpi-eeprom-update", password)
    console.print(out)

    if "UPDATE AVAILABLE" not in out:
        console.print("\n[green]✓ EEPROM is up to date[/green]")
        return

    if not yes:
        if not typer.confirm("\nApply EEPROM update? (requires reboot)"):
            console.print("Aborted.")
            raise typer.Exit(0)

    # Apply update
    console.print("\n[bold]Applying EEPROM update...[/bold]")
    ok, out, err = run_sudo(conn, "rpi-eeprom-update -a", password)
    if ok:
        console.print("[green]✓ EEPROM update applied[/green]")
        console.print("\n[yellow]Reboot required to complete update:[/yellow]")
        console.print("  [cyan]sudo reboot[/cyan]")
    else:
        console.print(f"[red]Failed to apply EEPROM update: {err}[/red]")
        raise typer.Exit(1)


if __name__ == "__main__":
    app()
