# Hailo 8 Installation Guide

This guide covers installing the HailoRT driver stack on Raspberry Pi 5 with Hailo-8 AI accelerator.

## Prerequisites

Before starting, ensure you have:

- **Raspberry Pi 5** with Hailo-8 module installed on PCIe slot
- **Ubuntu 22.04 or 24.04** (other Linux distributions may work but are untested)
- **Internet connection** for downloading packages
- **SSH access** to the robot configured in `config.json`

Hardware requirements:
- Hailo-8 module properly seated in PCIe slot
- PCIe enabled in boot configuration
- Adequate cooling (Hailo-8 generates heat during inference)

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

### Step 2: Install HailoRT Driver

Run the automated installation:

```bash
uv run python deploy_hailo8.py install
```

The script performs these steps:
1. Adds Hailo APT repository with GPG key
2. Installs `hailort` package (runtime and CLI tools)
3. Installs `hailort-dkms` package (kernel driver)
4. Loads `hailo_pci` kernel module
5. Installs Python bindings via pip

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
Device Node:     ✓ crw-rw---- 1 root hailo 248, 0 Jan  1 12:00 /dev/hailo0
Kernel Module:   ✓ hailo_pci 131072 0
HailoRT CLI:     ✓ hailortcli 4.18.0
Python Bindings: ✓ hailo_platform 4.18.0

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

## Manual Installation

If automated installation fails or you prefer manual control, follow these steps directly on the robot.

### 1. Add Hailo APT Repository

SSH to the robot and add the Hailo repository:

```bash
# Download and install GPG key
wget -qO - https://hailo.ai/keys/hailo-public.gpg | \
    sudo gpg --dearmor -o /usr/share/keyrings/hailo-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/hailo-archive-keyring.gpg] https://hailo.ai/raspberry-pi/ stable main" | \
    sudo tee /etc/apt/sources.list.d/hailo.list
```

### 2. Install HailoRT Packages

Update package lists and install HailoRT:

```bash
sudo apt-get update
sudo apt-get install -y hailort hailort-dkms
```

Package details:
- `hailort` - Runtime libraries and CLI tools (hailortcli)
- `hailort-dkms` - Kernel driver with DKMS for automatic rebuilds

Installation takes 2-5 minutes depending on internet speed.

### 3. Load Kernel Module

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
crw-rw---- 1 root hailo 248, 0 Jan  1 12:00 /dev/hailo0
```

If `/dev/hailo0` does not exist, reboot the system:

```bash
sudo reboot
```

After reboot, verify again:

```bash
ls -la /dev/hailo0
lsmod | grep hailo
```

### 4. Verify HailoRT Installation

Test the HailoRT CLI:

```bash
hailortcli --version
hailortcli scan
```

Expected output from `hailortcli scan`:
```
Scanning for Hailo devices...
Device 0: Hailo-8
  PCIe BDF: 0000:01:00.0
  Device Architecture: HAILO8
  Firmware Version: 4.18.0
  Control Protocol Version: 2
```

Get detailed device information:

```bash
hailortcli fw-control identify
```

### 5. Install Python Bindings

Install the Python bindings for programmatic access:

```bash
pip install hailort --break-system-packages
```

Note: `--break-system-packages` is required on Ubuntu 23.04+ with PEP 668 enforcement.

Verify Python import:

```bash
python3 -c "import hailo_platform; print(hailo_platform.__version__)"
```

Expected output:
```
4.18.0
```

### 6. Configure User Permissions (Optional)

To run Hailo applications without sudo, add your user to the `hailo` group:

```bash
sudo usermod -aG hailo $USER
```

**Important:** Logout and login (or reboot) for group changes to take effect.

Verify group membership:

```bash
groups | grep hailo
```

## Verification

After installation (automated or manual), verify all components:

### Quick Check

```bash
# From local machine
uv run python deploy_hailo8.py check

# Or directly on robot
hailortcli scan
python3 -c "import hailo_platform; print('OK')"
```

### Detailed Status

```bash
uv run python deploy_hailo8.py status
```

This shows:
- Device scan (board type, firmware version)
- Firmware control info (core clock, temperature)
- Device health status

### Expected Results

All checks should show ✓ (green checkmark):

| Component | Status | Command to Verify |
|-----------|--------|-------------------|
| PCIe Device | ✓ | `lspci \| grep -i hailo` |
| Device Node | ✓ | `ls /dev/hailo0` |
| Kernel Module | ✓ | `lsmod \| grep hailo` |
| HailoRT CLI | ✓ | `hailortcli --version` |
| Python Bindings | ✓ | `python3 -c "import hailo_platform"` |

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
   dkms status | grep hailort
   ```
   Expected: `hailort/<version>, <kernel>, arm64: installed`

3. **Rebuild DKMS module** if status shows errors:
   ```bash
   # Get HailoRT version
   hailortcli --version

   # Rebuild for current kernel
   sudo dkms install hailort/<version>
   ```
4. **Reboot** to ensure module loads at boot:
   ```bash
   sudo reboot
   ```
5. **Check kernel logs** for errors:
   ```bash
   dmesg | grep -i hailo
   ```

### Kernel Module Won't Load

**Symptoms:**
```
sudo modprobe hailo_pci
modprobe: ERROR: could not insert 'hailo_pci': Operation not permitted
```

**Solutions:**

1. **Check kernel version compatibility**:
   ```bash
   uname -r
   dkms status
   ```
2. **Ensure DKMS module is built** for your kernel:
   ```bash
   sudo dkms autoinstall
   ```
3. **Check for Secure Boot** (must be disabled):
   ```bash
   mokutil --sb-state
   ```
   If Secure Boot is enabled, disable in UEFI/BIOS settings
4. **Review kernel logs**:
   ```bash
   sudo dmesg | tail -50
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
4. **Verify group membership**:
   ```bash
   groups | grep hailo
   ```
5. **Check device permissions**:
   ```bash
   ls -la /dev/hailo0
   ```
   Expected: `crw-rw---- 1 root hailo ...`

### Python Import Fails

**Symptoms:**
```python
>>> import hailo_platform
ModuleNotFoundError: No module named 'hailo_platform'
```

**Solutions:**

1. **Install Python bindings**:
   ```bash
   pip install hailort --break-system-packages
   ```
2. **Check Python version** (requires 3.8+):
   ```bash
   python3 --version
   ```
3. **Verify installation**:
   ```bash
   pip list | grep hailort
   ```
4. **Check you're using system Python**, not a venv:
   ```bash
   which python3
   ```

### hailortcli Not Found

**Symptoms:**
```
bash: hailortcli: command not found
```

**Solutions:**

1. **Reinstall hailort package**:
   ```bash
   sudo apt-get install --reinstall hailort
   ```
2. **Check PATH**:
   ```bash
   echo $PATH | grep -o '/usr/bin'
   which hailortcli
   ```
3. **Verify package installation**:
   ```bash
   dpkg -l | grep hailort
   ```

### Installation Hangs During apt-get

**Symptoms:**
- `apt-get install` appears frozen during hailort-dkms installation

**Solutions:**

1. **Wait patiently** - DKMS compilation can take 5-10 minutes on Pi5
2. **Check progress** in another terminal:
   ```bash
   ps aux | grep dkms
   tail -f /var/lib/dkms/hailort/*/build/make.log
   ```
3. **Ensure adequate cooling** - CPU throttling slows compilation
4. **If truly stuck**, cancel (Ctrl+C) and retry:
   ```bash
   sudo apt-get clean
   sudo apt-get install -y hailort-dkms
   ```

### Reboot Required But Robot Unreachable

**Symptoms:**
- Installation script says "reboot recommended"
- SSH connection lost or robot not responding

**Solutions:**

1. **Physical reboot** - Power cycle the robot if SSH is unavailable
2. **Resume installation** after reboot:
   ```bash
   uv run python deploy_hailo8.py install --skip-reboot
   ```
3. The `--skip-reboot` flag continues installation after reboot without prompting again

## Getting Additional Help

If issues persist after trying troubleshooting steps:

1. **Run diagnostic commands** and save output:
   ```bash
   uv run python deploy_hailo8.py check > hailo_check.txt
   lspci -vv > pcie_info.txt
   dmesg > kernel_log.txt
   ```

2. **Collect system information**:
   ```bash
   uname -a
   cat /etc/os-release
   dpkg -l | grep hailo
   ```

3. **Check Hailo documentation**:
   - Official docs: https://hailo.ai/developer-zone/documentation/
   - Community forum: https://community.hailo.ai/

4. **Report issues** with collected logs to project maintainers

## Uninstallation

To remove HailoRT driver (if needed):

```bash
# Remove packages
sudo apt-get remove --purge hailort hailort-dkms

# Remove Python bindings
pip uninstall hailort

# Remove repository configuration
sudo rm /etc/apt/sources.list.d/hailo.list
sudo rm /usr/share/keyrings/hailo-archive-keyring.gpg

# Clean up state markers
rm -f ~/.landerpi_setup/hailo_*
```

Note: This does not remove the physical Hailo-8 hardware or disable PCIe.
