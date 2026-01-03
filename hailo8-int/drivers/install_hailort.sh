#!/bin/bash
# Install HailoRT on Raspberry Pi 5 (Ubuntu)
# Uses Raspberry Pi APT repository which contains Hailo packages
# Run with sudo

set -e

echo "=== Installing HailoRT for Raspberry Pi 5 ==="

# Step 1: Add Raspberry Pi repository (contains Hailo packages)
echo "Adding Raspberry Pi APT repository..."
curl -fsSL https://archive.raspberrypi.com/debian/raspberrypi.gpg.key | \
    sudo gpg --dearmor -o /usr/share/keyrings/raspberrypi-archive-keyring.gpg
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/raspberrypi-archive-keyring.gpg] https://archive.raspberrypi.com/debian/ bookworm main" | \
    sudo tee /etc/apt/sources.list.d/raspi.list

# Step 2: Install kernel headers for running kernel
echo "Installing kernel headers..."
KERNEL_VERSION=$(uname -r)
echo "Running kernel: $KERNEL_VERSION"

if [ ! -d "/lib/modules/$KERNEL_VERSION/build" ]; then
    echo "Installing linux-headers-$KERNEL_VERSION..."
    sudo apt-get update
    sudo apt-get install -y "linux-headers-$KERNEL_VERSION" || \
        sudo apt-get install -y linux-headers-generic
fi

# Step 3: Install HailoRT packages (version 4.19.0 for DKMS compatibility)
echo "Installing HailoRT packages..."
sudo apt-get update
sudo apt-get install -y --allow-downgrades hailort=4.19.0-3

# Install DKMS - may need patching for kernel 6.4+
echo "Installing hailo-dkms..."
if ! sudo apt-get install -y hailo-dkms; then
    echo "DKMS build failed - applying kernel 6.4+ compatibility patch..."

    # Patch MODULE_IMPORT_NS for kernel 6.4+ compatibility
    # The macro changed from MODULE_IMPORT_NS(ns) to MODULE_IMPORT_NS("ns")
    sudo sed -i 's/MODULE_IMPORT_NS(DMA_BUF)/MODULE_IMPORT_NS("DMA_BUF")/g' \
        /var/lib/dkms/hailo_pci/*/source/linux/vdma/memory.c 2>/dev/null || true

    # Rebuild DKMS module
    echo "Rebuilding DKMS for kernel $KERNEL_VERSION..."
    sudo dkms build hailo_pci/4.19.0 -k "$KERNEL_VERSION"
    sudo dkms install hailo_pci/4.19.0 -k "$KERNEL_VERSION" --force
fi

# Step 4: Load kernel module
echo "Loading kernel module..."
sudo modprobe hailo_pci

# Verify installation
echo "Verifying installation..."
if [ -c /dev/hailo0 ]; then
    echo "✓ /dev/hailo0 exists"
else
    echo "✗ /dev/hailo0 not found"
    echo "Note: May require reboot to load DKMS module"
    echo "Run: sudo reboot"
    exit 1
fi

# Step 5: Install Python bindings (best effort)
echo "Installing Python bindings..."
PY_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
echo "Python version: $PY_VERSION"

if sudo apt-get install -y python3-hailort 2>/dev/null; then
    echo "✓ Python bindings installed via apt"
elif pip install hailort --break-system-packages 2>/dev/null; then
    echo "✓ Python bindings installed via pip"
else
    echo "! Python bindings not available for Python $PY_VERSION"
    echo "  Note: Hailo SDK requires Python ≤3.11. Use Docker for Python API."
fi

echo "=== Installation complete ==="
hailortcli scan
hailortcli fw-control identify
