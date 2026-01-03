# Hailo 8 Installation Guide

This guide covers installing the HailoRT 4.23 driver stack on Raspberry Pi 5 with Hailo-8 AI accelerator.

## Prerequisites

Before starting, ensure you have:

- **Raspberry Pi 5** with Hailo-8 module installed on PCIe slot
- **Ubuntu 22.04, 24.04, or 25.10** (uses Raspberry Pi Trixie APT repository)
- **Internet connection** for downloading packages
- **SSH access** to the robot configured in `config.json`

Hardware requirements:
- Hailo-8 module properly seated in PCIe slot
- PCIe enabled in boot configuration
- Adequate cooling (Hailo-8 generates heat during inference)

## Important Notes

### HailoRT 4.23 Release (January 2025)
This installer uses **HailoRT 4.23** from the Raspberry Pi **Trixie** repository. Key improvements:
- **Python 3.13 support** - Python bindings now work on Ubuntu 25.10
- Improved DKMS compatibility with modern kernels
- Better stability and error handling

### Repository
Uses the **Raspberry Pi Trixie APT repository** (`archive.raspberrypi.com/debian/ trixie main`).

### Firmware Version
M.2 Hailo-8 modules have firmware 4.19.0 which is not field-upgradeable. This is normal - the 4.23 runtime works correctly with 4.19.0 firmware (you'll see a warning message, but inference works fine).

### TAPPAS Core
The full `hailo-all` metapackage and `hailo-tappas-core` have Raspberry Pi OS specific dependencies that don't work on Ubuntu. The core HailoRT packages (hailort, hailort-pcie-driver, python3-hailort) work perfectly.

## Automated Installation (Recommended)

The automated installation script handles repository setup, package installation, kernel module loading, and verification.

### Step 1: Check Hardware Status

First, verify the Hailo-8 hardware is detected:

```bash
uv run python deploy_hailo8.py check
```

Expected output when hardware is present but driver not installed:
```
PCIe Device:     ✓ Hailo-8 AI Processor
Device Node:     ✗ /dev/hailo* not found (driver not loaded)
Kernel Module:   ✗ hailo kernel module not loaded
HailoRT CLI:     ✗ hailortcli not installed
Python Bindings: ✗ hailo_platform not installed

Hardware detected. Run 'deploy_hailo8.py install' to set up driver.
```

If PCIe device is not detected, see [Troubleshooting](#troubleshooting) section.

### Step 2: Install HailoRT 4.23 Driver

Run the automated installation:

```bash
uv run python deploy_hailo8.py install
```

The script performs these steps:
1. Adds Raspberry Pi Trixie APT repository with GPG key
2. Installs DKMS (required before Hailo packages)
3. Installs kernel headers matching your running kernel
4. Removes any old Hailo packages
5. Installs `hailort` 4.23.0 (runtime and CLI tools)
6. Installs `hailort-pcie-driver` (kernel driver with DKMS)
7. Loads `hailo_pci` kernel module
8. Installs `python3-hailort` (Python API)

The installation is **idempotent** - you can safely re-run it if interrupted.

### Step 3: Reboot (If Required)

If you see this message:

```
/dev/hailo0 not found - may need reboot
Reboot recommended to load DKMS module.
```

Reboot the robot to load the DKMS-built kernel module:

```bash
ssh <user>@<robot-ip> 'sudo reboot'
```

Wait 30-60 seconds for reboot, then continue to Step 4.

If `/dev/hailo0` appeared immediately, you can skip the reboot.

### Step 4: Verify Installation

After reboot (if needed), verify everything is working:

```bash
uv run python deploy_hailo8.py check
```

Expected output when installation is successful:
```
PCIe Device:     ✓ 01:00.0 Co-processor: Hailo Technologies Ltd. Hailo-8 AI Processor
Device Node:     ✓ crw-rw-rw- 1 root root 510, 0 Jan  2 12:00 /dev/hailo0
Kernel Module:   ✓ hailo_pci 118784 0
HailoRT CLI:     ✓ HailoRT-CLI version 4.23.0
Python Bindings: ✓ hailo_platform 4.23.0

Summary:
  PCIe Device      OK
  Device Node      OK
  Kernel Module    OK
  HailoRT CLI      OK
  Python Bindings  OK

Hailo 8 is fully operational!
```

### Step 5: View Device Information

Get detailed device information including firmware version and temperature:

```bash
uv run python deploy_hailo8.py status
```

This shows:
- Device scan results (board type, serial number)
- Firmware version and build date
- Device temperature (if available)

Note: You may see a warning about firmware version mismatch - this is normal for M.2 modules.

## Manual Installation

If automated installation fails or you prefer manual control, follow these steps directly on the robot.

### 1. Add Raspberry Pi Trixie Repository

SSH to the robot and add the Raspberry Pi repository:

```bash
# Download and install GPG key
curl -fsSL https://archive.raspberrypi.com/debian/raspberrypi.gpg.key | \
    sudo gpg --dearmor -o /usr/share/keyrings/raspberrypi-archive-keyring.gpg

# Add Trixie repository (has HailoRT 4.23)
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/raspberrypi-archive-keyring.gpg] https://archive.raspberrypi.com/debian/ trixie main" | \
    sudo tee /etc/apt/sources.list.d/raspi.list
```

### 2. Install Prerequisites

Install DKMS and kernel headers:

```bash
sudo apt-get update

# IMPORTANT: Install DKMS first (before Hailo packages)
sudo apt-get install -y dkms

# Install kernel headers
KERNEL_VERSION=$(uname -r)
echo "Running kernel: $KERNEL_VERSION"
sudo apt-get install -y "linux-headers-$KERNEL_VERSION"
```

### 3. Install HailoRT 4.23 Packages

```bash
# Install HailoRT runtime
sudo apt-get install -y hailort=4.23.0

# Install PCIe driver (includes DKMS module)
sudo apt-get install -y hailort-pcie-driver

# Install Python bindings
sudo apt-get install -y python3-hailort
```

Package details:
- `hailort` - Runtime libraries and CLI tools (hailortcli)
- `hailort-pcie-driver` - Kernel driver with DKMS for automatic rebuilds
- `python3-hailort` - Python API bindings

Installation takes 2-5 minutes depending on internet speed and DKMS compilation.

### 4. Load Kernel Module

Load the Hailo PCIe kernel module:

```bash
sudo modprobe hailo_pci
```

Check if the device node was created:

```bash
ls -la /dev/hailo0
```

Expected output:
```
crw-rw-rw- 1 root root 510, 0 Jan  2 12:00 /dev/hailo0
```

If `/dev/hailo0` does not exist, reboot the system:

```bash
sudo reboot
```

### 5. Verify Installation

Test the HailoRT CLI:

```bash
hailortcli --version
hailortcli scan
```

Expected output from `hailortcli scan`:
```
Hailo Devices:
[-] Device: 0001:01:00.0
```

Get detailed device information:

```bash
hailortcli fw-control identify
```

Note: You may see a warning about firmware version mismatch - this is expected for M.2 modules.

Verify Python import:

```bash
python3 -c "import hailo_platform; print(hailo_platform.__version__)"
```

Expected output:
```
4.23.0
```

## Performance

| Backend | Model | FPS | Latency |
|---------|-------|-----|---------|
| CPU (Pi5) | YOLOv11n | 2-5 | 200-500ms |
| Hailo-8 | YOLOv11n | 25-40 | 25-40ms |

## Next Steps

After successful installation:

1. **Convert YOLO Model** - See [MODEL_CONVERSION.md](MODEL_CONVERSION.md) for converting YOLOv11 to HEF format
2. **Deploy Models and Node** - Run `uv run python deploy_hailo8.py deploy` to upload models and ROS2 node
3. **Run Validation Test** - Run `uv run python deploy_hailo8.py test --benchmark` to benchmark inference
4. **Integrate with Exploration** - Update launch configuration to use Hailo-accelerated YOLO

## Troubleshooting

### PCIe Device Not Detected

**Symptoms:**
```
lspci | grep -i hailo
(no output)
```

**Solutions:**

1. **Check physical connection** - Ensure Hailo-8 module is properly seated in PCIe slot
2. **Enable PCIe in boot config** - Edit `/boot/firmware/config.txt` and add:
   ```
   dtparam=pciex1
   ```
3. **Reboot** after config changes:
   ```bash
   sudo reboot
   ```
4. **Verify PCIe link status**:
   ```bash
   lspci -vv | grep -A20 Hailo
   ```
   Look for "LnkSta: Speed 5GT/s, Width x1" indicating active PCIe Gen2 x1 link

### Device Node Not Found

**Symptoms:**
```
ls /dev/hailo*
ls: cannot access '/dev/hailo*': No such file or directory
```

**Solutions:**

1. **Load kernel module manually**:
   ```bash
   sudo modprobe hailo_pci
   ```
2. **Check DKMS status**:
   ```bash
   dkms status | grep hailo
   ```
   Expected: `hailo_pci/4.23.0, <kernel>, arm64: installed`

3. **Rebuild DKMS module** if status shows errors:
   ```bash
   sudo dkms install hailo_pci/4.23.0
   ```
4. **Reboot** to ensure module loads at boot:
   ```bash
   sudo reboot
   ```
5. **Check kernel logs** for errors:
   ```bash
   dmesg | grep -i hailo
   ```

### Permission Denied on /dev/hailo0

**Symptoms:**
```
hailortcli scan
Error: Failed to open device: Permission denied
```

**Solutions:**

1. **Add user to hailo group**:
   ```bash
   sudo usermod -aG hailo $USER
   ```
2. **Logout and login** for group changes to take effect
3. **Or use sudo** as temporary workaround:
   ```bash
   sudo hailortcli scan
   ```

### Python Import Fails

**Symptoms:**
```python
>>> import hailo_platform
ModuleNotFoundError: No module named 'hailo_platform'
```

**Solutions:**

1. **Install Python bindings**:
   ```bash
   sudo apt-get install -y python3-hailort
   ```
2. **Check Python version** (4.23 supports 3.8-3.13):
   ```bash
   python3 --version
   ```
3. **Verify installation**:
   ```bash
   pip3 list | grep hailort
   ```

### Firmware Version Warning

**Symptoms:**
```
[HailoRT] [warning] Unsupported firmware operation. Host: 4.23.0, Device: 4.19.0
```

**Solution:**
This is **normal** for M.2 Hailo-8 modules. The firmware is not field-upgradeable, but the 4.23 runtime works correctly with 4.19 firmware. Inference works fine - the warning is cosmetic.

## Optional: hailo-apps-infra

For additional ML tools and applications, you can install hailo-apps-infra:

```bash
# Clone repository
cd ~
git clone --depth 1 https://github.com/hailo-ai/hailo-apps-infra.git

# Install dependencies
sudo apt-get install -y python3-venv pkg-config

# Run installer
cd hailo-apps-infra
sudo ./install.sh
```

Note: hailo-apps-infra has additional dependencies and the full TAPPAS core may not work on Ubuntu due to Raspberry Pi OS specific packages.

## Uninstallation

To remove HailoRT driver (if needed):

```bash
# Remove packages
sudo apt-get remove --purge hailort hailort-pcie-driver python3-hailort

# Remove repository configuration
sudo rm /etc/apt/sources.list.d/raspi.list

# Clean up state markers
rm -f ~/.landerpi_setup/hailo423_*
```

Note: This does not remove the physical Hailo-8 hardware or disable PCIe.
