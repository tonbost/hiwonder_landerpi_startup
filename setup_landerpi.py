#!/usr/bin/env python3
import sys
import os
import time
from typing import Optional
from pathlib import Path

import typer
from rich.console import Console
from rich.prompt import Prompt, Confirm
from rich.panel import Panel
from rich.progress import Progress, SpinnerColumn, TextColumn
from fabric import Connection
from invoke.exceptions import UnexpectedExit

app = typer.Typer(help="Hiwonder LanderPi Remote Setup Assistant")
console = Console()

REMOTE_STATE_DIR = ".landerpi_setup"

class RemoteSetupManager:
    def __init__(self, host: str, user: str, password: Optional[str] = None, key_filename: Optional[str] = None, dry_run: bool = False):
        self.host = host
        self.user = user
        self.dry_run = dry_run
        self.console = console
        
        connect_kwargs = {}
        if password:
            connect_kwargs['password'] = password
        if key_filename:
            connect_kwargs['key_filename'] = key_filename
            
        self.conn = Connection(host=host, user=user, connect_kwargs=connect_kwargs)

    def verify_connection(self):
        """Check if we can connect to the host."""
        if self.dry_run:
            self.console.print(f"[yellow][DRY RUN] Would connect to {self.user}@{self.host}[/yellow]")
            return True
            
        try:
            self.console.print(f"[bold blue]Connecting to {self.user}@{self.host}...[/bold blue]")
            self.conn.run("echo 'Connection successful'", hide=True)
            self.console.print("[green]✓ Connected successfully[/green]")
            return True
        except Exception as e:
            self.console.print(f"[red]✗ Connection failed:[/red] {e}")
            return False

    def run_remote(self, command: str, description: str, sudo: bool = False, check: bool = True, warn: bool = False):
        """Run a command on the remote host."""
        if sudo:
            # We use sudo=True in fabric or prepend sudo. Fabric's run(sudo=True) is deprecated in favor of sudo()
            # But conn.sudo() handles password prompts better if configured.
            pass

        if self.dry_run:
            prefix = "sudo " if sudo else ""
            self.console.print(f"[yellow][DRY RUN] Remote exec:[/yellow] {prefix}{command}")
            return True

        self.console.print(f"[bold blue]Running:[/bold blue] {description}...")
        try:
            if sudo:
                result = self.conn.sudo(command, hide=True, warn=warn, pty=True)
            else:
                result = self.conn.run(command, hide=True, warn=warn)
            
            if result.failed and not warn:
                raise UnexpectedExit(result)
                
            self.console.print(f"[green]✓ Success:[/green] {description}")
            return True
        except UnexpectedExit as e:
            self.console.print(f"[red]✗ Failed:[/red] {description}")
            self.console.print(f"[red]Stderr:[/red] {e.result.stderr}")
            if not check:
                return False
            if not warn:
                raise e

    def mark_step_done(self, step_name: str):
        """Create a marker file on remote to indicate step completion."""
        if self.dry_run:
            return
        self.conn.run(f"mkdir -p {REMOTE_STATE_DIR}", hide=True)
        self.conn.run(f"touch {REMOTE_STATE_DIR}/{step_name}.done", hide=True)

    def is_step_done(self, step_name: str) -> bool:
        """Check if step is already completed."""
        if self.dry_run:
            return False
        try:
            self.conn.run(f"test -f {REMOTE_STATE_DIR}/{step_name}.done", hide=True)
            return True
        except UnexpectedExit:
            return False

    def append_to_remote_file(self, filepath: str, content: str, sudo: bool = True):
        """Append content to a remote file if not exists."""
        if self.dry_run:
            self.console.print(f"[yellow][DRY RUN] Append to {filepath}:[/yellow] {content}")
            return

        # Check existence
        grep_cmd = f"grep -F '{content.strip()}' {filepath}"
        try:
            if sudo:
                self.conn.sudo(grep_cmd, hide=True)
            else:
                self.conn.run(grep_cmd, hide=True)
            self.console.print(f"[dim]Content present in {filepath}, skipping.[/dim]")
            return
        except UnexpectedExit:
            pass # Content not found

        # Append
        cmd = f"echo '{content}' | tee -a {filepath}"
        self.run_remote(cmd, f"Appending to {filepath}", sudo=sudo)

    def upload_file(self, local_path: str, remote_path: str, description: str):
        """Upload a file to the remote host."""
        if self.dry_run:
            self.console.print(f"[yellow][DRY RUN] Upload {local_path} -> {remote_path}[/yellow]")
            return
        
        self.console.print(f"[bold blue]Uploading:[/bold blue] {description}...")
        try:
            self.conn.put(local_path, remote_path)
            self.console.print(f"[green]✓ Success:[/green] {description}")
        except Exception as e:
            self.console.print(f"[red]✗ Failed via put:[/red] {e}")
            raise e


@app.command()
def connect(
    host: str = typer.Option(..., help="Raspberry Pi IP address"),
    user: str = typer.Option("pi", help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    key_path: Optional[str] = typer.Option(None, help="Path to SSH private key"),
):
    """Test connection to the Raspberry Pi."""
    mgr = RemoteSetupManager(host, user, password, key_path)
    if mgr.verify_connection():
        # Check for state dir
        try:
            result = mgr.conn.run(f"ls {REMOTE_STATE_DIR}", hide=True, warn=True)
            if result.ok and result.stdout.strip():
                console.print(f"[blue]Found existing setup state:[/blue] {result.stdout.strip()}")
        except Exception:
            pass

@app.command()
def deploy(
    host: str = typer.Option(..., help="Raspberry Pi IP address"),
    user: str = typer.Option("pi", help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
    key_path: Optional[str] = typer.Option(None, help="Path to SSH private key"),
    dry_run: bool = typer.Option(False, "--dry-run", help="Simulate commands"),
    skip_docker: bool = typer.Option(False, help="Skip Docker installation"),
    reset: bool = typer.Option(False, help="Clear previous state and start over")
):
    """Run the setup process on the remote Pi."""
    mgr = RemoteSetupManager(host, user, password, key_path, dry_run)
    
    console.print(Panel.fit(f"Deploying to {user}@{host}", style="bold magenta"))

    if not mgr.verify_connection():
        sys.exit(1)

    # Reset State
    if reset and not dry_run:
        if Confirm.ask("Are you sure you want to reset setup state? This will re-run all steps.", default=False):
            mgr.conn.run(f"rm -rf {REMOTE_STATE_DIR}", hide=True)
            console.print("[yellow]State cleared.[/yellow]")

    # 1. System Updates
    step = "system_update"
    if mgr.is_step_done(step):
        console.print(f"[dim]✓ Skipping {step} (already done)[/dim]")
    else:
        console.rule("[bold]Step 1: System Updates[/bold]")
        mgr.run_remote("apt update", "Updating package lists", sudo=True)
        mgr.run_remote("apt upgrade -y", "Upgrading packages", sudo=True)
        mgr.run_remote("apt install -y build-essential git curl wget vim nano htop net-tools", "Installing tools", sudo=True)
        mgr.mark_step_done(step)

    # 2. Hardware Config
    step = "hardware_config"
    if mgr.is_step_done(step):
         console.print(f"[dim]✓ Skipping {step}[/dim]")
    else:
        console.rule("[bold]Step 2: Hardware Configuration[/bold]")
        config_file = "/boot/firmware/config.txt"
        # Check if file exists, if not try older path
        if not dry_run:
            try:
                mgr.conn.run(f"test -f {config_file}", hide=True)
            except UnexpectedExit:
                config_file = "/boot/config.txt"
        
        mgr.append_to_remote_file(config_file, "dtparam=i2c_arm=on", sudo=True)
        mgr.append_to_remote_file(config_file, "dtparam=spi=on", sudo=True)
        mgr.mark_step_done(step)

    # 2b. SDK Upload
    step = "sdk_upload"
    if mgr.is_step_done(step):
        console.print(f"[dim]✓ Skipping {step}[/dim]")
    else:
        console.rule("[bold]Step 2b: SDK Upload[/bold]")
        # Upload ros_robot_controller SDK for direct motor control
        script_dir = Path(__file__).parent
        sdk_file = script_dir / "drivers" / "ros_robot_controller-ros2" / "src" / "ros_robot_controller" / "ros_robot_controller" / "ros_robot_controller_sdk.py"
        sdk_dest_dir = f"/home/{user}/ros_robot_controller"
        sdk_dest = f"{sdk_dest_dir}/ros_robot_controller_sdk.py"

        if sdk_file.exists() or dry_run:
            # Create destination directory
            mgr.run_remote(f"mkdir -p {sdk_dest_dir}", "Creating motion controller directory")

            # Upload SDK
            mgr.upload_file(str(sdk_file), sdk_dest, "Motion controller SDK")

            # Set ownership
            mgr.run_remote(
                f"chown -R {user}:{user} {sdk_dest_dir}",
                "Setting SDK ownership",
                sudo=True,
                warn=True
            )

            console.print(f"[green]✓ Motion controller SDK installed to {sdk_dest_dir}[/green]")
        else:
            console.print(f"[yellow]Warning: SDK not found at {sdk_file}[/yellow]")
            console.print("[blue]Motion test will upload SDK on-the-fly to /tmp instead.[/blue]")

        mgr.mark_step_done(step)

    # 2d. Aurora 930 Depth Camera Setup
    step = "camera_setup"
    if mgr.is_step_done(step):
        console.print(f"[dim]✓ Skipping {step}[/dim]")
    else:
        console.rule("[bold]Step 2d: Depth Camera Setup[/bold]")

        # Install USB libraries needed for camera
        mgr.run_remote(
            "apt install -y libusb-1.0-0-dev libudev-dev",
            "Installing USB libraries for depth camera",
            sudo=True
        )

        # Create Deptrum udev rules for Aurora 930 camera
        # Vendor ID 3251 is used by Deptrum cameras
        deptrum_udev_rule = 'SUBSYSTEM=="usb", ATTRS{idVendor}=="3251", MODE:="0666", TAG+="uaccess", TAG+="udev-acl"'
        mgr.run_remote(
            f"echo '{deptrum_udev_rule}' > /tmp/99-deptrum-libusb.rules",
            "Creating temp udev rule"
        )
        mgr.run_remote(
            "mv /tmp/99-deptrum-libusb.rules /etc/udev/rules.d/99-deptrum-libusb.rules",
            "Installing udev rule",
            sudo=True
        )

        # Reload udev rules
        mgr.run_remote("udevadm control --reload-rules", "Reloading udev rules", sudo=True)
        mgr.run_remote("udevadm trigger", "Triggering udev rules", sudo=True)

        # Upload and install Deptrum ROS2 driver
        script_dir = Path(__file__).parent
        driver_file = script_dir / "drivers" / "depth-camera" / "deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz"
        driver_name = "deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz"
        deptrum_ws = f"/home/{user}/deptrum_ws"

        if driver_file.exists() or dry_run:
            # Create workspace directory
            mgr.run_remote(f"mkdir -p {deptrum_ws}/src", "Creating Deptrum workspace")

            # Upload driver tarball
            mgr.upload_file(str(driver_file), f"{deptrum_ws}/{driver_name}", "Deptrum ROS2 driver (19MB)")

            # Extract driver
            mgr.run_remote(
                f"tar -xzf {deptrum_ws}/{driver_name} -C {deptrum_ws}/src/",
                "Extracting Deptrum driver"
            )

            # Fix package.xml name mismatch (required for build)
            mgr.run_remote(
                f"sed -i 's/<name>deptrum-ros-driver<\\/name>/<name>deptrum-ros-driver-aurora930<\\/name>/' "
                f"{deptrum_ws}/src/deptrum-ros-driver-aurora930-0.2.1001/package.xml",
                "Fixing package.xml name"
            )

            # Clean up tarball
            mgr.run_remote(f"rm -f {deptrum_ws}/{driver_name}", "Cleaning up tarball")

            console.print("[green]✓ Deptrum driver source installed to ~/deptrum_ws[/green]")
            console.print("[blue]Driver will be built automatically after Docker setup.[/blue]")
        else:
            console.print(f"[yellow]Warning: Driver not found at {driver_file}[/yellow]")
            console.print("[blue]See DepthCameraHowTo.md for manual driver installation.[/blue]")

        mgr.mark_step_done(step)

    # 3. Docker & ROS2
    if not skip_docker:
        step = "docker_ros2"
        if mgr.is_step_done(step):
            console.print(f"[dim]✓ Skipping {step}[/dim]")
        else:
            console.rule("[bold]Step 3: Docker & ROS2[/bold]")
            # Install Docker
            try:
                if dry_run: raise UnexpectedExit(None)
                mgr.conn.run("docker --version", hide=True)
                console.print("[green]Docker already installed.[/green]")
            except UnexpectedExit:
                mgr.run_remote("curl -fsSL https://get.docker.com | sh", "Installing Docker")
                mgr.run_remote(f"usermod -aG docker {user}", f"Adding {user} to docker group", sudo=True)
            
            # Pull Image
            mgr.run_remote("docker pull ros:humble-ros-base", "Pulling ROS2 Humble image", sudo=True)
            
            # Create Workspace
            ws_path = f"/home/{user}/landerpi_ros"
            mgr.run_remote(f"mkdir -p {ws_path}/src", "Creating workspace")
            
            # Dockerfile - includes dependencies for Aurora 930 depth camera
            dockerfile = """FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y \\
    python3-pip \\
    python3-serial \\
    python3-smbus \\
    i2c-tools \\
    libusb-1.0-0-dev \\
    libudev-dev \\
    ros-humble-navigation2 \\
    ros-humble-nav2-bringup \\
    ros-humble-slam-toolbox \\
    ros-humble-cv-bridge \\
    ros-humble-image-transport \\
    ros-humble-angles \\
    ros-humble-tf2-ros \\
    ros-humble-tf2-geometry-msgs \\
    ros-humble-vision-msgs \\
    ros-humble-diagnostic-updater \\
    && rm -rf /var/lib/apt/lists/*

# Install YOLOv11 dependencies (numpy<2 required for cv_bridge compatibility)
RUN pip3 install --no-cache-dir "numpy<2" ultralytics onnxruntime
WORKDIR /ros2_ws
CMD ["bash"]
"""
            # Write Dockerfile remotely
            if not dry_run:
                from io import StringIO
                mgr.conn.put(StringIO(dockerfile), f"{ws_path}/Dockerfile")
            else:
                console.print("[yellow][DRY RUN] Upload Dockerfile[/yellow]")

            mgr.run_remote(f"docker build -t landerpi-ros2 {ws_path}", "Building Custom Docker Image", sudo=True)
            mgr.mark_step_done(step)

        # 3b. Build Depth Camera Driver (requires Docker)
        step = "camera_driver_build"
        deptrum_ws = f"/home/{user}/deptrum_ws"
        if mgr.is_step_done(step):
            console.print(f"[dim]✓ Skipping {step}[/dim]")
        else:
            # Check if deptrum workspace exists (was set up in camera_setup step)
            try:
                if not dry_run:
                    mgr.conn.run(f"test -d {deptrum_ws}/src/deptrum-ros-driver-aurora930-0.2.1001", hide=True)
                console.rule("[bold]Step 3b: Building Depth Camera Driver[/bold]")

                # Build the driver using Docker
                build_cmd = (
                    f"docker run --rm "
                    f"-v {deptrum_ws}:/deptrum_ws "
                    f"-w /deptrum_ws "
                    f"landerpi-ros2:latest "
                    f"bash -c 'source /opt/ros/humble/setup.bash && "
                    f"colcon build --cmake-args -DSTREAM_SDK_TYPE=AURORA930'"
                )
                mgr.run_remote(build_cmd, "Building Deptrum ROS2 driver in Docker", sudo=True)

                # Fix ownership of build files (Docker creates as root)
                mgr.run_remote(
                    f"chown -R {user}:{user} {deptrum_ws}",
                    "Fixing ownership of build files",
                    sudo=True
                )

                # Verify build succeeded
                try:
                    if not dry_run:
                        mgr.conn.run(f"test -d {deptrum_ws}/install/deptrum-ros-driver-aurora930", hide=True)
                    console.print("[green]✓ Depth camera driver built successfully[/green]")
                except UnexpectedExit:
                    console.print("[yellow]Warning: Build may have failed - install directory not found[/yellow]")

                mgr.mark_step_done(step)
            except UnexpectedExit:
                console.print("[dim]Skipping camera driver build - workspace not found[/dim]")

    # 4. Remote Access
    step = "remote_access"
    if mgr.is_step_done(step):
        console.print(f"[dim]✓ Skipping {step}[/dim]")
    else:
        console.rule("[bold]Step 4: Remote Access[/bold]")
        mgr.run_remote("systemctl enable ssh", "Enabling SSH", sudo=True)
        if Confirm.ask("Install VNC Server?", default=False) or dry_run:
             mgr.run_remote("apt install -y tightvncserver", "Installing VNC", sudo=True)
        mgr.mark_step_done(step)

    console.rule("[bold]Setup Complete[/bold]")
    console.print(f"[green]Setup successfully deployed to {host}.[/green]")
    console.print("[yellow]A reboot is recommended.[/yellow]")
    if Confirm.ask("Reboot remote host now?", default=False):
        mgr.run_remote("reboot", "Rebooting remote host", sudo=True, warn=True)

if __name__ == "__main__":
    app()
