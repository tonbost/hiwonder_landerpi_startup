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
    echo "Note: May require reboot to load DKMS module"
    exit 1
fi

# Install Python bindings
echo "Installing Python bindings..."
pip install hailort --break-system-packages

echo "=== Installation complete ==="
hailortcli scan
