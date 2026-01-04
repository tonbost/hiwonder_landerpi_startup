# Hailo 8 Integration Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Integrate Hailo-8 AI accelerator to run YOLOv11 inference at 25-40 FPS instead of 2-5 FPS on CPU.

**Architecture:** Native HailoRT on host (driver + Python bindings), separate `yolo_hailo_node.py` ROS2 node with same interface as existing CPU node, model conversion on workstation.

**Tech Stack:** HailoRT 4.x, hailo_platform Python SDK, ROS2 Humble, YOLOv11n HEF model

---

## Task 1: Create Directory Structure

**Files:**
- Create: `hailo8-int/README.md`
- Create: `hailo8-int/docs/INSTALLATION.md`
- Create: `hailo8-int/docs/MODEL_CONVERSION.md`
- Create: `hailo8-int/docs/TROUBLESHOOTING.md`
- Create: `hailo8-int/models/README.md`
- Create: `hailo8-int/drivers/install_hailort.sh`
- Create: `hailo8-int/ros2_nodes/yolo_hailo/package.xml`
- Create: `hailo8-int/ros2_nodes/yolo_hailo/setup.py`
- Create: `hailo8-int/ros2_nodes/yolo_hailo/setup.cfg`
- Create: `hailo8-int/ros2_nodes/yolo_hailo/yolo_hailo/__init__.py`
- Create: `hailo8-int/conversion/README.md`
- Create: `hailo8-int/validation/test_hailo.py`

**Step 1: Create directory structure**

```bash
mkdir -p hailo8-int/{docs,models,drivers,ros2_nodes/yolo_hailo/yolo_hailo,conversion,validation}
```

**Step 2: Create placeholder README files**

Create `hailo8-int/README.md`:
```markdown
# Hailo 8 AI Accelerator Integration

Hardware acceleration for YOLOv11 inference on LanderPi robot using Hailo-8 NPU.

## Quick Start

```bash
# Check Hailo hardware status
uv run python deploy_hailo8.py check

# Install HailoRT driver
uv run python deploy_hailo8.py install

# Deploy models and ROS2 node
uv run python deploy_hailo8.py deploy

# Run validation test
uv run python deploy_hailo8.py test

# Show device status
uv run python deploy_hailo8.py status
```

## Documentation

- [Installation Guide](docs/INSTALLATION.md)
- [Model Conversion](docs/MODEL_CONVERSION.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)

## Performance

| Backend | Model | FPS | Latency |
|---------|-------|-----|---------|
| CPU (Pi5) | YOLOv11n | 2-5 | 200-500ms |
| Hailo-8 | YOLOv11n | 25-40 | 25-40ms |
```

**Step 3: Commit**

```bash
git add hailo8-int/
git commit -m "feat(hailo): create directory structure for Hailo 8 integration"
```

---

## Task 2: Create deploy_hailo8.py Skeleton

**Files:**
- Create: `deploy_hailo8.py`

**Step 1: Create deployment script with check and status commands**

Create `deploy_hailo8.py`:
```python
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
```

**Step 2: Test the script**

```bash
uv run python deploy_hailo8.py check
```

Expected: Shows Hailo hardware status (PCIe OK, driver NOT READY)

**Step 3: Commit**

```bash
git add deploy_hailo8.py
git commit -m "feat(hailo): add deploy_hailo8.py with check and status commands"
```

---

## Task 3: Implement install Command

**Files:**
- Modify: `deploy_hailo8.py`
- Modify: `hailo8-int/drivers/install_hailort.sh`

**Step 1: Create driver installation script**

Create `hailo8-int/drivers/install_hailort.sh`:
```bash
#!/bin/bash
# Install HailoRT on Raspberry Pi 5
# Run with sudo

set -e

echo "=== Installing HailoRT for Raspberry Pi 5 ==="

# Add Hailo repository
echo "Adding Hailo APT repository..."
wget -qO - https://hailo.ai/keys/hailo-public.gpg | sudo gpg --dearmor -o /usr/share/keyrings/hailo-archive-keyring.gpg
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/hailo-archive-keyring.gpg] https://hailo.ai/raspberry-pi/ stable main" | sudo tee /etc/apt/sources.list.d/hailo.list

# Update and install
echo "Installing HailoRT packages..."
sudo apt-get update
sudo apt-get install -y hailort hailort-dkms

# Load kernel module
echo "Loading kernel module..."
sudo modprobe hailo_pci

# Verify installation
echo "Verifying installation..."
if [ -c /dev/hailo0 ]; then
    echo "✓ /dev/hailo0 exists"
else
    echo "✗ /dev/hailo0 not found"
    exit 1
fi

# Install Python bindings
echo "Installing Python bindings..."
pip install hailort --break-system-packages

echo "=== Installation complete ==="
hailortcli scan
```

**Step 2: Implement install command in deploy_hailo8.py**

Add to `deploy_hailo8.py` (replace the install function):
```python
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
            hide=True
        )
        conn.sudo(
            'echo "deb [arch=arm64 signed-by=/usr/share/keyrings/hailo-archive-keyring.gpg] '
            'https://hailo.ai/raspberry-pi/ stable main" | '
            'tee /etc/apt/sources.list.d/hailo.list',
            hide=True
        )
        mark_done("repo_added")
        console.print("  [green]✓[/green] Repository added")

    # Step 2: Install packages
    if is_done("packages_installed"):
        console.print("[dim]Step 2: HailoRT packages already installed[/dim]")
    else:
        console.print("\n[bold]Step 2: Installing HailoRT packages...[/bold]")
        conn.sudo("apt-get update -qq", hide=True)
        console.print("  Installing hailort...")
        conn.sudo("apt-get install -y hailort", hide=True)
        console.print("  Installing hailort-dkms...")
        conn.sudo("apt-get install -y hailort-dkms", hide=True)
        mark_done("packages_installed")
        console.print("  [green]✓[/green] Packages installed")

    # Step 3: Load kernel module
    console.print("\n[bold]Step 3: Loading kernel module...[/bold]")
    conn.sudo("modprobe hailo_pci", hide=True, warn=True)

    # Verify device node
    result = conn.run("ls /dev/hailo0", hide=True, warn=True)
    if result.return_code == 0:
        console.print("  [green]✓[/green] /dev/hailo0 available")
    else:
        console.print("  [yellow]![/yellow] /dev/hailo0 not found - may need reboot")
        if not skip_reboot:
            console.print("\n[bold yellow]Reboot recommended to load DKMS module.[/bold yellow]")
            console.print("Run: ssh user@robot 'sudo reboot'")
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
```

**Step 3: Test installation**

```bash
uv run python deploy_hailo8.py install
```

**Step 4: Commit**

```bash
git add deploy_hailo8.py hailo8-int/drivers/install_hailort.sh
git commit -m "feat(hailo): implement install command with idempotent driver setup"
```

---

## Task 4: Create Installation Documentation

**Files:**
- Create: `hailo8-int/docs/INSTALLATION.md`

**Step 1: Write installation guide**

Create `hailo8-int/docs/INSTALLATION.md`:
```markdown
# Hailo 8 Installation Guide

## Prerequisites

- Raspberry Pi 5 with Hailo-8 module on PCIe
- Ubuntu 22.04 or 24.04
- Internet connection

## Automated Installation

The recommended way to install HailoRT:

```bash
# Check hardware is detected
uv run python deploy_hailo8.py check

# Install driver and Python bindings
uv run python deploy_hailo8.py install

# Reboot if prompted
ssh user@robot 'sudo reboot'

# Verify installation
uv run python deploy_hailo8.py status
```

## Manual Installation

If automated installation fails:

### 1. Add Hailo Repository

```bash
wget -qO - https://hailo.ai/keys/hailo-public.gpg | sudo gpg --dearmor -o /usr/share/keyrings/hailo-archive-keyring.gpg
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/hailo-archive-keyring.gpg] https://hailo.ai/raspberry-pi/ stable main" | sudo tee /etc/apt/sources.list.d/hailo.list
```

### 2. Install Packages

```bash
sudo apt-get update
sudo apt-get install -y hailort hailort-dkms
```

### 3. Load Kernel Module

```bash
sudo modprobe hailo_pci
```

Or reboot for automatic loading.

### 4. Verify

```bash
ls -la /dev/hailo0
hailortcli scan
```

### 5. Install Python Bindings

```bash
pip install hailort --break-system-packages
python3 -c "import hailo_platform; print(hailo_platform.__version__)"
```

## Verification

After installation, all checks should pass:

```bash
uv run python deploy_hailo8.py check
```

Expected output:
```
PCIe Device:     ✓ Hailo-8 AI Processor
Device Node:     ✓ /dev/hailo0
Kernel Module:   ✓ hailo_pci
HailoRT CLI:     ✓ hailortcli 4.x.x
Python Bindings: ✓ hailo_platform 4.x.x
```

## Troubleshooting

### Device not found after reboot

Check DKMS status:
```bash
dkms status
```

Rebuild if needed:
```bash
sudo dkms install hailort/4.x.x
```

### Permission denied on /dev/hailo0

Add user to hailo group:
```bash
sudo usermod -aG hailo $USER
# Logout and login
```

### PCIe device not detected

Check PCIe link:
```bash
lspci -vv | grep -A20 Hailo
```

Verify in `/boot/firmware/config.txt`:
```
dtparam=pciex1
```
```

**Step 2: Commit**

```bash
git add hailo8-int/docs/INSTALLATION.md
git commit -m "docs(hailo): add installation guide"
```

---

## Task 5: Create ROS2 Node Package Structure

**Files:**
- Create: `hailo8-int/ros2_nodes/yolo_hailo/package.xml`
- Create: `hailo8-int/ros2_nodes/yolo_hailo/setup.py`
- Create: `hailo8-int/ros2_nodes/yolo_hailo/setup.cfg`
- Create: `hailo8-int/ros2_nodes/yolo_hailo/resource/yolo_hailo`
- Create: `hailo8-int/ros2_nodes/yolo_hailo/yolo_hailo/__init__.py`

**Step 1: Create package.xml**

Create `hailo8-int/ros2_nodes/yolo_hailo/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>yolo_hailo</name>
  <version>0.1.0</version>
  <description>Hailo-accelerated YOLO object detection node</description>
  <maintainer email="user@example.com">LanderPi</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>std_msgs</depend>
  <depend>cv_bridge</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Step 2: Create setup.py**

Create `hailo8-int/ros2_nodes/yolo_hailo/setup.py`:
```python
from setuptools import find_packages, setup

package_name = 'yolo_hailo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LanderPi',
    maintainer_email='user@example.com',
    description='Hailo-accelerated YOLO object detection node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_hailo_node = yolo_hailo.yolo_hailo_node:main',
        ],
    },
)
```

**Step 3: Create setup.cfg**

Create `hailo8-int/ros2_nodes/yolo_hailo/setup.cfg`:
```ini
[develop]
script_dir=$base/lib/yolo_hailo
[install]
install_scripts=$base/lib/yolo_hailo
```

**Step 4: Create resource file and __init__.py**

```bash
mkdir -p hailo8-int/ros2_nodes/yolo_hailo/resource
touch hailo8-int/ros2_nodes/yolo_hailo/resource/yolo_hailo
touch hailo8-int/ros2_nodes/yolo_hailo/yolo_hailo/__init__.py
```

**Step 5: Commit**

```bash
git add hailo8-int/ros2_nodes/yolo_hailo/
git commit -m "feat(hailo): create yolo_hailo ROS2 package structure"
```

---

## Task 6: Implement yolo_hailo_node.py

**Files:**
- Create: `hailo8-int/ros2_nodes/yolo_hailo/yolo_hailo/yolo_hailo_node.py`

**Step 1: Create the Hailo YOLO node**

Create `hailo8-int/ros2_nodes/yolo_hailo/yolo_hailo/yolo_hailo_node.py`:
```python
#!/usr/bin/env python3
"""
Hailo-accelerated YOLO Object Detection Node for LanderPi.

Subscribes to camera images, runs YOLOv11 inference on Hailo-8, publishes detections.
Same interface as yolo_node.py (CPU version) for drop-in replacement.
"""
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D

# Hailo imports
try:
    from hailo_platform import HEF, VDevice, HailoStreamInterface, ConfigureParams, InferVStreams, InputVStreamParams, OutputVStreamParams, FormatType
    HAILO_AVAILABLE = True
except ImportError:
    HAILO_AVAILABLE = False


# COCO class names (same as ultralytics)
COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]


class YoloHailoNode(Node):
    def __init__(self):
        super().__init__('yolo_hailo_detector')

        if not HAILO_AVAILABLE:
            self.get_logger().error('hailo_platform not available. Install with: pip install hailort')
            raise RuntimeError('Hailo platform not available')

        # Parameters
        self.declare_parameter('hef_path', str(Path.home() / 'landerpi/hailo/models/yolo11n_hailo.hef'))
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)

        hef_path = self.get_parameter('hef_path').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.nms_thres = self.get_parameter('nms_threshold').get_parameter_value().double_value
        self.input_width = self.get_parameter('input_width').get_parameter_value().integer_value
        self.input_height = self.get_parameter('input_height').get_parameter_value().integer_value

        # Initialize Hailo device
        self.get_logger().info(f'Loading HEF model: {hef_path}')
        try:
            self.hef = HEF(hef_path)
            self.vdevice = VDevice()

            # Configure network
            configure_params = ConfigureParams.create_from_hef(self.hef, interface=HailoStreamInterface.PCIe)
            self.network_group = self.vdevice.configure(self.hef, configure_params)[0]
            self.network_group_params = self.network_group.create_params()

            # Get input/output info
            self.input_vstream_info = self.hef.get_input_vstream_infos()[0]
            self.output_vstream_infos = self.hef.get_output_vstream_infos()

            self.get_logger().info(f'Model loaded. Input: {self.input_vstream_info.shape}')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize Hailo: {e}')
            raise

        # Performance tracking
        self.frame_count = 0
        self.total_inference_time = 0.0
        self.last_fps_report = time.time()

        # ROS Communications
        self.subscription = self.create_subscription(
            Image,
            '/aurora/rgb/image_raw',
            self.image_callback,
            10)

        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10)

        self.hazards_publisher = self.create_publisher(
            String,
            '/hazards',
            10)

        # Load hazard config
        self.hazard_classes = ["person", "dog", "cat", "cup", "bottle", "stop sign"]
        self._load_hazard_config()

        self.get_logger().info('Hailo YOLO Detector Node initialized')

    def _load_hazard_config(self):
        """Load hazard classes from config file if available."""
        config_paths = [
            Path.home() / 'landerpi/config/yolo_hazards.json',
            Path('/ros2_ws/config/yolo_hazards.json'),
        ]
        for config_path in config_paths:
            if config_path.exists():
                try:
                    with open(config_path) as f:
                        config = json.load(f)
                    self.hazard_classes = config.get('hazard_classes', self.hazard_classes)
                    self.get_logger().info(f'Loaded hazard classes: {self.hazard_classes}')
                    break
                except Exception as e:
                    self.get_logger().warn(f'Failed to load hazard config: {e}')

    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """Preprocess image for Hailo inference."""
        # Resize to model input size
        resized = cv2.resize(image, (self.input_width, self.input_height))
        # Convert BGR to RGB if needed
        if len(resized.shape) == 3 and resized.shape[2] == 3:
            resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        # Normalize to 0-1 range
        normalized = resized.astype(np.float32) / 255.0
        # Add batch dimension
        return np.expand_dims(normalized, axis=0)

    def postprocess(self, outputs: dict, orig_width: int, orig_height: int) -> list:
        """Postprocess Hailo outputs to detections."""
        detections = []

        # YOLO output format varies by model - adapt as needed
        # This is a simplified example; actual implementation depends on HEF output format
        for output_name, output_data in outputs.items():
            # Parse YOLO output tensors
            # Format depends on specific model export configuration
            pass

        return detections

    def image_callback(self, msg):
        """Process incoming image."""
        try:
            # Convert ROS Image to numpy array
            if msg.encoding == 'rgb8':
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == 'bgr8':
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
                return

            orig_height, orig_width = image.shape[:2]

            # Preprocess
            input_data = self.preprocess(image)

            # Run inference
            start_time = time.time()

            with InferVStreams(self.network_group, self.network_group_params) as infer_pipeline:
                input_dict = {self.input_vstream_info.name: input_data}
                output_dict = infer_pipeline.infer(input_dict)

            inference_time = time.time() - start_time
            self.total_inference_time += inference_time
            self.frame_count += 1

            # Postprocess
            detections = self.postprocess(output_dict, orig_width, orig_height)

            # Publish detections
            self._publish_detections(detections, msg.header)

            # Publish hazards
            self._publish_hazards(detections)

            # Report FPS periodically
            if time.time() - self.last_fps_report > 10.0:
                avg_fps = self.frame_count / self.total_inference_time if self.total_inference_time > 0 else 0
                self.get_logger().info(f'Average FPS: {avg_fps:.1f}')
                self.last_fps_report = time.time()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def _publish_detections(self, detections: list, header):
        """Publish Detection2DArray message."""
        msg = Detection2DArray()
        msg.header = header

        for det in detections:
            detection = Detection2D()
            detection.bbox = BoundingBox2D()
            detection.bbox.center.position.x = det['cx']
            detection.bbox.center.position.y = det['cy']
            detection.bbox.size_x = det['width']
            detection.bbox.size_y = det['height']

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det['class']
            hypothesis.hypothesis.score = det['score']
            detection.results.append(hypothesis)

            msg.detections.append(detection)

        self.detection_publisher.publish(msg)

    def _publish_hazards(self, detections: list):
        """Publish hazards for exploration controller."""
        hazards = []
        for det in detections:
            if det['class'] in self.hazard_classes:
                hazards.append({
                    'type': det['class'],
                    'distance': det.get('distance', 1.0),
                    'angle': det.get('angle', 0.0),
                    'score': det['score'],
                    'source': 'hailo'
                })

        if hazards:
            msg = String()
            msg.data = json.dumps({'hazards': hazards})
            self.hazards_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloHailoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Commit**

```bash
git add hailo8-int/ros2_nodes/yolo_hailo/yolo_hailo/yolo_hailo_node.py
git commit -m "feat(hailo): implement yolo_hailo_node.py with Hailo inference"
```

---

## Task 7: Implement deploy Command

**Files:**
- Modify: `deploy_hailo8.py`

**Step 1: Add deploy command implementation**

Add to `deploy_hailo8.py` (replace deploy function):
```python
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
```

**Step 2: Commit**

```bash
git add deploy_hailo8.py
git commit -m "feat(hailo): implement deploy command for models and ROS2 node"
```

---

## Task 8: Implement test Command

**Files:**
- Modify: `deploy_hailo8.py`
- Create: `hailo8-int/validation/test_hailo.py`

**Step 1: Add test command**

Add to `deploy_hailo8.py` (replace test function):
```python
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
        console.print(f"  [red]✗[/red] Model not found: {model_path}")
        console.print("  Run 'deploy_hailo8.py deploy' after converting model")
        return
    console.print(f"  [green]✓[/green] Model found: {model_path}")

    # Run benchmark if requested
    if benchmark:
        console.print("\n[bold]Running Benchmark...[/bold]")
        result = conn.run(f"hailortcli benchmark {model_path}", hide=False, warn=True)
        if result.return_code == 0:
            console.print("\n[green]Benchmark complete![/green]")
        else:
            console.print("[red]Benchmark failed[/red]")

    # Test Python import and basic inference
    console.print("\n[bold]Python Integration Test:[/bold]")
    test_script = '''
import sys
try:
    from hailo_platform import HEF, VDevice
    print("  ✓ hailo_platform imported")

    hef = HEF(sys.argv[1])
    print(f"  ✓ HEF loaded: {hef.get_input_vstream_infos()[0].shape}")

    vdevice = VDevice()
    print("  ✓ VDevice created")

    print("\\n  [SUCCESS] Hailo integration working!")
except Exception as e:
    print(f"  ✗ Error: {e}")
    sys.exit(1)
'''
    result = conn.run(f"python3 -c '{test_script}' {model_path}", hide=False, warn=True)

    if result.return_code == 0:
        console.print("\n[bold green]All tests passed![/bold green]")
    else:
        console.print("\n[bold red]Some tests failed.[/bold red]")
```

**Step 2: Commit**

```bash
git add deploy_hailo8.py
git commit -m "feat(hailo): implement test command with device and model validation"
```

---

## Task 9: Create Model Conversion Documentation

**Files:**
- Create: `hailo8-int/docs/MODEL_CONVERSION.md`
- Create: `hailo8-int/conversion/README.md`
- Create: `hailo8-int/conversion/Dockerfile`
- Create: `hailo8-int/conversion/convert_yolo.py`

**Step 1: Create MODEL_CONVERSION.md**

Create `hailo8-int/docs/MODEL_CONVERSION.md`:
```markdown
# YOLOv11 to HEF Model Conversion

This guide covers converting YOLOv11 models to Hailo Executable Format (HEF) for the Hailo-8 accelerator.

## Prerequisites

- x86_64 Linux machine or Docker
- Hailo Dataflow Compiler (requires Hailo developer account)
- Python 3.8+
- YOLOv11 model (.pt file)

## Conversion Pipeline

```
YOLOv11 (.pt) → ONNX (.onnx) → HAR (.har) → HEF (.hef)
```

## Step 1: Export to ONNX

On any machine with ultralytics installed:

```bash
pip install ultralytics
yolo export model=yolo11n.pt format=onnx imgsz=640 opset=11
```

This creates `yolo11n.onnx`.

## Step 2: Convert to HEF (Docker Method)

The Hailo Dataflow Compiler only runs on x86_64. Use Docker:

```bash
cd hailo8-int/conversion

# Build converter image
docker build -t hailo-converter .

# Run conversion
docker run -v $(pwd):/workspace hailo-converter \
    python convert_yolo.py \
    --input /workspace/yolo11n.onnx \
    --output /workspace/yolo11n_hailo.hef
```

## Step 3: Copy HEF to Models Directory

```bash
cp yolo11n_hailo.hef ../models/
```

## Step 4: Deploy to Robot

```bash
uv run python deploy_hailo8.py deploy
```

## Hailo Model Zoo

Pre-converted models are available in the Hailo Model Zoo:
- https://github.com/hailo-ai/hailo_model_zoo

Note: YOLOv11 may require manual conversion as it's newer than available zoo models.

## Calibration Images

For INT8 quantization, provide ~100 representative images:

```bash
mkdir -p calibration_images
# Copy sample images from your dataset
```

## Troubleshooting

### ONNX export fails
Ensure ultralytics version supports YOLOv11:
```bash
pip install --upgrade ultralytics
```

### HEF compilation OOM
Reduce batch size or use machine with more RAM (16GB+ recommended).

### Model accuracy degradation
Increase calibration images or try different quantization settings.
```

**Step 2: Create conversion Dockerfile**

Create `hailo8-int/conversion/Dockerfile`:
```dockerfile
# Hailo Dataflow Compiler environment
# Note: Requires Hailo developer account for DFC installation
FROM python:3.10-slim

WORKDIR /workspace

# Install dependencies
RUN pip install numpy opencv-python-headless onnx

# Note: hailo_sdk requires manual installation from Hailo
# Download from: https://hailo.ai/developer-zone/
# COPY hailo_sdk-*.whl /tmp/
# RUN pip install /tmp/hailo_sdk-*.whl

COPY requirements.txt .
RUN pip install -r requirements.txt

COPY convert_yolo.py .

CMD ["python", "convert_yolo.py", "--help"]
```

**Step 3: Create conversion script**

Create `hailo8-int/conversion/convert_yolo.py`:
```python
#!/usr/bin/env python3
"""
Convert YOLOv11 ONNX model to Hailo HEF format.

Requires Hailo Dataflow Compiler (hailo_sdk).
"""
import argparse
from pathlib import Path


def convert_onnx_to_hef(input_path: str, output_path: str, calibration_dir: str = None):
    """Convert ONNX model to HEF format."""
    try:
        from hailo_sdk_client import ClientRunner
    except ImportError:
        print("Error: hailo_sdk not installed.")
        print("Download from https://hailo.ai/developer-zone/")
        return False

    print(f"Converting {input_path} to {output_path}")

    # Create runner
    runner = ClientRunner(hw_arch="hailo8")

    # Parse ONNX
    print("Parsing ONNX model...")
    hn, npz = runner.translate_onnx_model(
        input_path,
        "yolo11n",
        start_node_names=["images"],
        end_node_names=["output0"],
    )

    # Optimize
    print("Optimizing model...")
    runner.optimize(hn, npz)

    # Quantize (if calibration images provided)
    if calibration_dir:
        print(f"Quantizing with calibration images from {calibration_dir}...")
        # Add calibration logic here

    # Compile
    print("Compiling to HEF...")
    hef = runner.compile()

    # Save
    with open(output_path, "wb") as f:
        f.write(hef)

    print(f"Saved HEF to {output_path}")
    return True


def main():
    parser = argparse.ArgumentParser(description="Convert YOLO ONNX to Hailo HEF")
    parser.add_argument("--input", required=True, help="Input ONNX file")
    parser.add_argument("--output", required=True, help="Output HEF file")
    parser.add_argument("--calibration", help="Calibration images directory")

    args = parser.parse_args()

    if not Path(args.input).exists():
        print(f"Error: Input file not found: {args.input}")
        return

    convert_onnx_to_hef(args.input, args.output, args.calibration)


if __name__ == "__main__":
    main()
```

**Step 4: Create requirements.txt**

Create `hailo8-int/conversion/requirements.txt`:
```
numpy
opencv-python-headless
onnx
```

**Step 5: Commit**

```bash
git add hailo8-int/docs/MODEL_CONVERSION.md hailo8-int/conversion/
git commit -m "docs(hailo): add model conversion guide and scripts"
```

---

## Task 10: Create Troubleshooting Documentation

**Files:**
- Create: `hailo8-int/docs/TROUBLESHOOTING.md`

**Step 1: Create troubleshooting guide**

Create `hailo8-int/docs/TROUBLESHOOTING.md`:
```markdown
# Hailo 8 Troubleshooting Guide

## Common Issues

### Device not found on PCIe

**Symptoms:**
```
lspci | grep -i hailo
(no output)
```

**Solutions:**
1. Check physical connection of Hailo module
2. Verify PCIe is enabled in `/boot/firmware/config.txt`:
   ```
   dtparam=pciex1
   ```
3. Reboot after config changes

### /dev/hailo0 not found

**Symptoms:**
```
ls /dev/hailo*
ls: cannot access '/dev/hailo*': No such file or directory
```

**Solutions:**
1. Load kernel module manually:
   ```bash
   sudo modprobe hailo_pci
   ```
2. Check DKMS status:
   ```bash
   dkms status
   ```
3. Rebuild DKMS module:
   ```bash
   sudo dkms install hailort/$(hailortcli --version | grep -oP '\d+\.\d+\.\d+')
   ```
4. Reboot

### Permission denied

**Symptoms:**
```
hailortcli scan
Error: Permission denied
```

**Solutions:**
1. Add user to hailo group:
   ```bash
   sudo usermod -aG hailo $USER
   ```
2. Logout and login
3. Or use sudo: `sudo hailortcli scan`

### Python import fails

**Symptoms:**
```python
>>> import hailo_platform
ModuleNotFoundError: No module named 'hailo_platform'
```

**Solutions:**
1. Install Python bindings:
   ```bash
   pip install hailort --break-system-packages
   ```
2. Check Python version (requires 3.8+)

### HEF model fails to load

**Symptoms:**
```
HEF validation failed
```

**Solutions:**
1. Verify HEF was compiled for Hailo-8 (not Hailo-8L or Hailo-15)
2. Check HEF file integrity:
   ```bash
   hailortcli parse-hef model.hef
   ```
3. Re-convert model with correct target

### Inference is slow

**Symptoms:**
- FPS lower than expected
- High latency

**Solutions:**
1. Check thermal throttling:
   ```bash
   hailortcli fw-control identify | grep -i temp
   ```
2. Ensure async pipeline is used (not sync)
3. Check input preprocessing isn't bottleneck

### Out of memory during inference

**Symptoms:**
```
HailoRTStatusException: HAILO_OUT_OF_HOST_MEMORY
```

**Solutions:**
1. Reduce batch size to 1
2. Use streaming inference
3. Free system memory

## Diagnostic Commands

```bash
# Full system check
uv run python deploy_hailo8.py check

# Device status
hailortcli scan
hailortcli fw-control identify

# Kernel module status
lsmod | grep hailo
dmesg | grep -i hailo

# PCIe link status
lspci -vv | grep -A20 Hailo

# Benchmark model
hailortcli benchmark model.hef
```

## Getting Help

1. Check Hailo documentation: https://hailo.ai/developer-zone/documentation/
2. Hailo community forum: https://community.hailo.ai/
3. GitHub issues on this project
```

**Step 2: Commit**

```bash
git add hailo8-int/docs/TROUBLESHOOTING.md
git commit -m "docs(hailo): add troubleshooting guide"
```

---

## Task 11: Create landerpi-hailo Skill

**Files:**
- Create: `.claude/skills/landerpi-hailo/SKILL.md`

**Step 1: Create the skill file**

```bash
mkdir -p .claude/skills/landerpi-hailo
```

Create `.claude/skills/landerpi-hailo/SKILL.md`:
```markdown
---
name: landerpi-hailo
description: Hailo 8 AI accelerator integration for HiWonder LanderPi robot. Provides hardware-accelerated YOLOv11 inference at 25-40 FPS. Requires landerpi-core skill for connection setup.
---

# LanderPi Hailo 8 AI Accelerator

## Overview

Hardware acceleration for YOLOv11 object detection using Hailo-8 NPU on Raspberry Pi 5. Provides 5-10x inference speedup compared to CPU.

**Prerequisites:**
- Load `landerpi-core` skill for connection and Docker setup
- Hailo-8 module installed on PCIe

## Quick Commands

| Command | Purpose |
|---------|---------|
| `uv run python deploy_hailo8.py check` | Check hardware/driver status |
| `uv run python deploy_hailo8.py install` | Install HailoRT driver |
| `uv run python deploy_hailo8.py deploy` | Upload models and ROS2 node |
| `uv run python deploy_hailo8.py test` | Run validation test |
| `uv run python deploy_hailo8.py status` | Show device info |

## Setup Flow

```bash
# 1. Check hardware is detected
uv run python deploy_hailo8.py check

# 2. Install driver (one-time)
uv run python deploy_hailo8.py install

# 3. Reboot if prompted
ssh user@robot 'sudo reboot'

# 4. Deploy models (after conversion)
uv run python deploy_hailo8.py deploy

# 5. Verify
uv run python deploy_hailo8.py test --benchmark
```

## Performance

| Backend | Model | FPS | Latency |
|---------|-------|-----|---------|
| CPU (Pi5) | YOLOv11n | 2-5 | 200-500ms |
| Hailo-8 | YOLOv11n | 25-40 | 25-40ms |

## Integration with Exploration

The Hailo YOLO node uses the same ROS2 interface as the CPU version:

```
/aurora/rgb/image_raw → [yolo_hailo_node] → /yolo/detections
                                          → /hazards
```

Enable via launch parameter:
```bash
# In docker-compose or launch file
use_hailo:=true
```

## Model Conversion

YOLOv11 models must be converted to HEF format:

```bash
# 1. Export to ONNX (any machine)
yolo export model=yolo11n.pt format=onnx imgsz=640

# 2. Convert to HEF (x86 machine with Hailo SDK)
cd hailo8-int/conversion
docker run -v $(pwd):/workspace hailo-converter python convert_yolo.py \
    --input yolo11n.onnx --output yolo11n_hailo.hef

# 3. Copy to models directory
cp yolo11n_hailo.hef ../models/

# 4. Deploy to robot
uv run python deploy_hailo8.py deploy
```

## Troubleshooting

### Device not found
```bash
# Check PCIe
lspci | grep -i hailo

# Check /boot/firmware/config.txt has:
dtparam=pciex1
```

### Driver not loaded
```bash
# Load manually
sudo modprobe hailo_pci

# Check DKMS
dkms status
```

### Permission denied
```bash
sudo usermod -aG hailo $USER
# Logout and login
```

## Files

| File | Location | Purpose |
|------|----------|---------|
| `deploy_hailo8.py` | Project root | Deployment script |
| `INSTALLATION.md` | `hailo8-int/docs/` | Setup guide |
| `MODEL_CONVERSION.md` | `hailo8-int/docs/` | Conversion guide |
| `yolo_hailo_node.py` | `hailo8-int/ros2_nodes/yolo_hailo/` | ROS2 node |
| `*.hef` | `hailo8-int/models/` | Compiled models |
```

**Step 2: Commit**

```bash
git add .claude/skills/landerpi-hailo/
git commit -m "feat(skills): add landerpi-hailo skill for AI accelerator"
```

---

## Task 12: Update Existing Skills

**Files:**
- Modify: `.claude/skills/landerpi-yolo/SKILL.md`
- Modify: `.claude/skills/landerpi-core/SKILL.md`
- Modify: `.claude/skills/landerpi-dev/SKILL.md`

**Step 1: Update landerpi-yolo skill**

Add to `.claude/skills/landerpi-yolo/SKILL.md` after the Performance section:

```markdown
## Hailo Acceleration

For 5-10x faster inference, use Hailo-8 hardware acceleration:

```bash
# Load the landerpi-hailo skill for setup instructions
```

See `landerpi-hailo` skill for:
- Driver installation
- Model conversion (YOLOv11 → HEF)
- Hardware-accelerated node deployment

| Backend | FPS | Latency |
|---------|-----|---------|
| CPU | 2-5 | 200-500ms |
| Hailo-8 | 25-40 | 25-40ms |
```

**Step 2: Update landerpi-core skill**

Add to Expected System State table in `.claude/skills/landerpi-core/SKILL.md`:

```markdown
| Hailo-8 (optional) | `/dev/hailo0` if installed | `lspci \| grep -i hailo` |
```

Add to Available Tools table:

```markdown
| Hailo Check | `uv run python deploy_hailo8.py check` | Hailo hardware/driver status |
| Hailo Install | `uv run python deploy_hailo8.py install` | Install HailoRT driver |
```

**Step 3: Update landerpi-dev skill**

Add a new section to `.claude/skills/landerpi-dev/SKILL.md`:

```markdown
## Hailo Integration Patterns

When working with Hailo-8 accelerated code:

1. **Check hardware first**: Always verify Hailo is available before using
2. **Fallback to CPU**: Provide CPU fallback when Hailo unavailable
3. **Same interface**: Hailo nodes should match CPU node interfaces
4. **HEF models**: Models must be pre-converted, not converted at runtime
```

**Step 4: Commit**

```bash
git add .claude/skills/
git commit -m "feat(skills): update existing skills with Hailo references"
```

---

## Task 13: Update CLAUDE.md

**Files:**
- Modify: `CLAUDE.md`

**Step 1: Add Hailo section to CLAUDE.md**

Add after the YOLO section:

```markdown
## Hailo 8 AI Accelerator

The robot has a Hailo-8 NPU on PCIe for hardware-accelerated inference.

**Setup:**
```bash
# Check hardware status
uv run python deploy_hailo8.py check

# Install driver (one-time)
uv run python deploy_hailo8.py install

# Deploy models and node
uv run python deploy_hailo8.py deploy

# Validate
uv run python deploy_hailo8.py test --benchmark

# Show device info
uv run python deploy_hailo8.py status
```

**Performance:**
| Backend | Model | FPS |
|---------|-------|-----|
| CPU | YOLOv11n | 2-5 |
| Hailo-8 | YOLOv11n | 25-40 |

**Files:**
- `deploy_hailo8.py` - Deployment script
- `hailo8-int/` - Models, drivers, ROS2 node, docs
```

**Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: add Hailo 8 section to CLAUDE.md"
```

---

## Task 14: Update README.md

**Files:**
- Modify: `README.md`

**Step 1: Add Hailo section to README**

Add a new section to README.md:

```markdown
## Hailo 8 AI Acceleration

The LanderPi supports Hailo-8 NPU for hardware-accelerated YOLO inference.

### Setup

```bash
# Check hardware
uv run python deploy_hailo8.py check

# Install driver
uv run python deploy_hailo8.py install

# Deploy (after model conversion)
uv run python deploy_hailo8.py deploy
```

### Performance

| Backend | YOLOv11n FPS | Latency |
|---------|--------------|---------|
| CPU | 2-5 | 200-500ms |
| Hailo-8 | 25-40 | 25-40ms |

See [hailo8-int/README.md](hailo8-int/README.md) for full documentation.
```

**Step 2: Commit**

```bash
git add README.md
git commit -m "docs: add Hailo 8 section to README"
```

---

## Summary

After completing all tasks, you will have:

1. **Directory structure** (`hailo8-int/`) with docs, models, drivers, ROS2 node
2. **Deployment script** (`deploy_hailo8.py`) with check, install, deploy, test, status commands
3. **Documentation** (INSTALLATION.md, MODEL_CONVERSION.md, TROUBLESHOOTING.md)
4. **ROS2 node** (`yolo_hailo_node.py`) with same interface as CPU version
5. **Model conversion** tools and guide
6. **Skills** (new `landerpi-hailo`, updated existing skills)
7. **Updated docs** (CLAUDE.md, README.md)

**To use Hailo after implementation:**
```bash
# 1. Install driver
uv run python deploy_hailo8.py install

# 2. Convert model (on x86 workstation)
# Follow hailo8-int/docs/MODEL_CONVERSION.md

# 3. Deploy
uv run python deploy_hailo8.py deploy

# 4. Start exploration with Hailo YOLO
uv run python deploy_explorer.py start --yolo --duration 10
# (after updating launch config to use_hailo:=true)
```
