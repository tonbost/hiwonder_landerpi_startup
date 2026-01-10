#!/bin/bash
# WM8960 Audio HAT driver installer for Ubuntu on Raspberry Pi
# Modified from PiSugar/Whisplay for systems without raspi-config

set -e

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (use sudo)" 1>&2
   exit 1
fi

# Check we're on a Raspberry Pi
is_Raspberry=$(cat /proc/device-tree/model | awk '{print $1}')
if [ "x${is_Raspberry}" != "xRaspberry" ]; then
  echo "Sorry, this driver only works on Raspberry Pi"
  exit 1
fi

echo "Installing WM8960 Audio HAT driver for Ubuntu..."

# Get the actual user's home directory (not root's)
if [ -n "$SUDO_USER" ]; then
    USER_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
    USER_HOME="$HOME"
fi
echo "User home directory: $USER_HOME"

# Check if SPI is already enabled
if ls /dev/spidev* > /dev/null 2>&1; then
    echo "SPI interface already enabled."
else
    echo "Warning: SPI interface not detected at /dev/spidev*"
    echo "Please ensure SPI is enabled in /boot/firmware/config.txt"
fi

# Navigate to Driver directory
cd "$USER_HOME/Whisplay/Driver"

# Unzip the driver package
echo "Extracting WM8960-Audio-HAT.zip..."
unzip -o WM8960-Audio-HAT.zip
cd WM8960-Audio-HAT

# Set kernel modules
echo "Configuring kernel modules..."
grep -q "i2c-dev" /etc/modules || echo "i2c-dev" >> /etc/modules
grep -q "snd-soc-wm8960" /etc/modules || echo "snd-soc-wm8960" >> /etc/modules
grep -q "snd-soc-wm8960-soundcard" /etc/modules || echo "snd-soc-wm8960-soundcard" >> /etc/modules

# Configure device tree overlays in config.txt
CONFIG_FILE="/boot/firmware/config.txt"
echo "Configuring device tree overlays in $CONFIG_FILE..."

# Enable I2S
sed -i -e 's:#dtparam=i2s=on:dtparam=i2s=on:g' $CONFIG_FILE || true
grep -q "dtparam=i2s=on" $CONFIG_FILE || echo "dtparam=i2s=on" >> $CONFIG_FILE

# Enable I2C
sed -i -e 's:#dtparam=i2c_arm=on:dtparam=i2c_arm=on:g' $CONFIG_FILE || true

# Add I2S mmap overlay
grep -q "dtoverlay=i2s-mmap" $CONFIG_FILE || echo "dtoverlay=i2s-mmap" >> $CONFIG_FILE

# Add WM8960 soundcard overlay
grep -q "dtoverlay=wm8960-soundcard" $CONFIG_FILE || echo "dtoverlay=wm8960-soundcard" >> $CONFIG_FILE

# Install config files
echo "Installing ALSA config files..."
mkdir -p /etc/wm8960-soundcard
cp *.conf /etc/wm8960-soundcard/ 2>/dev/null || true
cp *.state /etc/wm8960-soundcard/ 2>/dev/null || true

# Install service
echo "Installing systemd service..."
cp wm8960-soundcard /usr/bin/
chmod u=rwx,go=rx /usr/bin/wm8960-soundcard
cp wm8960-soundcard.service /lib/systemd/system/

systemctl daemon-reload
systemctl enable wm8960-soundcard.service
systemctl start wm8960-soundcard || echo "Service start deferred until reboot"

echo ""
echo "------------------------------------------------------"
echo "Installation complete!"
echo "Please reboot your Raspberry Pi to apply all settings:"
echo "  sudo reboot"
echo "------------------------------------------------------"
