# HiWonder LanderPi Robot - Complete Ubuntu Setup Guide for Raspberry Pi 5

This comprehensive guide walks you through setting up Ubuntu on your Raspberry Pi 5 for the HiWonder LanderPi robot, compiled from official LanderPi tutorials and documentation.

---

## Table of Contents

1. [Prerequisites and Requirements](#1-prerequisites-and-requirements)
2. [Downloading and Flashing the System Image](#2-downloading-and-flashing-the-system-image)
3. [First Boot and Initial Configuration](#3-first-boot-and-initial-configuration)
4. [Network Configuration](#4-network-configuration)
5. [System Updates and Mirror Configuration](#5-system-updates-and-mirror-configuration)
6. [Essential Software Installation](#6-essential-software-installation)
7. [Remote Access Setup](#7-remote-access-setup)
8. [Robot Software Setup](#8-robot-software-setup)
9. [Troubleshooting](#9-troubleshooting)
10. [Quick Reference Commands](#10-quick-reference-commands)

---

## 1. Prerequisites and Requirements

### Hardware Requirements

| Component | Specification | Notes |
|-----------|--------------|-------|
| **Raspberry Pi 5** | 4GB or 8GB RAM recommended | 8GB for ROS2 workloads |
| **MicroSD Card** | 32GB minimum, 64GB+ recommended | Class 10 / UHS-I or better |
| **Power Supply** | USB-C, 5V/5A (27W) | Official Pi 5 PSU recommended |
| **Ethernet Cable** | Cat5e or better | For initial setup (optional) |
| **Monitor + HDMI** | Micro-HDMI to HDMI cable | For initial setup (optional) |
| **Keyboard/Mouse** | USB | For initial setup (optional) |

### Software Requirements (on your computer)

- **Raspberry Pi Imager** - [Download here](https://www.raspberrypi.com/software/)
- **Or** Win32DiskImager (Windows) / balenaEtcher (cross-platform)
- **VNC Viewer** - Included in `/Users/ZDZ/Downloads/LanderPi/2. Software/02 Remote Desktop Software/`
- **SSH Client** - MobaXterm (Windows) or Terminal (Mac/Linux)

### Files Included with LanderPi Package

```
LanderPi/
├── 1. Tutorials/              # Documentation and lessons
├── 2. Software/               # Tools for setup
│   ├── 01 App Installation Package/   # WonderPi mobile app
│   ├── 02 Remote Desktop Software/    # VNC Viewer
│   ├── 03 SD Card Formatting Tool/    # SD Card Formatter
│   ├── 04 Image Flashing Tool/        # Win32DiskImager
│   └── 05 File Transmitting Tool/     # File transfer utilities
└── 3. System Image & Source Code/     # System image info
```

---

## 2. Downloading and Flashing the System Image

### Step 2.1: Obtain the System Image

**Option A: Pre-configured LanderPi Image (Recommended)**
1. Check the `Important Notice.pdf` in `/3. System Image & Source Code/` for download links
2. Visit HiWonder's official website: https://www.hiwonder.com
3. Navigate to LanderPi product page → Downloads section
4. Download the official LanderPi Ubuntu image (includes ROS2 and robot software)

**Option B: Fresh Ubuntu Installation**
1. Visit https://ubuntu.com/download/raspberry-pi
2. Download **Ubuntu Server 22.04.x LTS (64-bit)** for Raspberry Pi
3. Note: This requires manual ROS2 and robot software installation

### Step 2.2: Prepare the SD Card

1. Insert the microSD card into your computer
2. **Backup any existing data** - the card will be completely erased

### Step 2.3: Flash the Image

**Using Raspberry Pi Imager (Recommended):**

```
1. Download and install Raspberry Pi Imager
2. Launch the application
3. Click "Choose Device" → Select "Raspberry Pi 5"
4. Click "Choose OS":
   - For LanderPi image: "Use Custom" → Select downloaded .img file
   - For fresh Ubuntu: "Other general-purpose OS" → "Ubuntu" → Select version
5. Click "Choose Storage" → Select your SD card
6. Click the gear icon (⚙️) for Advanced Options:
   ✓ Set hostname: landerpi
   ✓ Enable SSH (Use password authentication)
   ✓ Set username: pi (or your preferred username)
   ✓ Set password: (choose a secure password)
   ✓ Configure wireless LAN (optional)
   ✓ Set locale settings: Your timezone
7. Click "Save" then "Write"
8. Wait for the process to complete (10-20 minutes)
9. Click "Continue" when done and safely eject the SD card
```

**Using Win32DiskImager (Windows):**
```
1. Extract the .img file if compressed (.zip or .xz)
2. Launch Win32DiskImager (from /2. Software/04 Image Flashing Tool/)
3. Select the image file
4. Select the correct drive letter for your SD card
5. Click "Write"
6. Wait for completion and safely eject
```

---

## 3. First Boot and Initial Configuration

### Step 3.1: Assemble and Power On

1. Insert the flashed SD card into Raspberry Pi 5
2. Connect peripherals (if using direct setup):
   - Monitor via micro-HDMI
   - USB keyboard and mouse
   - Ethernet cable (recommended for first boot)
3. Connect power supply **last**

### Step 3.2: First Boot Process

```
First boot takes 2-5 minutes. The system will:
1. Resize the filesystem to use full SD card
2. Generate SSH host keys
3. Configure initial settings
4. May reboot automatically once
```

### Step 3.3: Initial Login

**Default Credentials (if using LanderPi image):**
```
Username: pi (or hiwonder, check documentation)
Password: (as set during imaging, or check documentation)
```

**Default Credentials (if using fresh Ubuntu):**
```
Username: ubuntu
Password: ubuntu (will be prompted to change on first login)
```

### Step 3.4: Essential First Commands

```bash
# Change password (if using default)
passwd

# Set hostname
sudo hostnamectl set-hostname landerpi

# Configure timezone
sudo timedatectl set-timezone America/New_York  # Change to your timezone
# Or interactive: sudo dpkg-reconfigure tzdata

# Update the system immediately
sudo apt update && sudo apt upgrade -y
```

---

## 4. Network Configuration

### Step 4.1: Check Network Status

```bash
# Check all network interfaces
ip addr show

# Check WiFi status
nmcli device status

# List available WiFi networks
nmcli device wifi list
```

### Step 4.2: Configure WiFi

**Method A: Using nmcli (command line)**
```bash
# Connect to WiFi
sudo nmcli device wifi connect "YOUR_WIFI_SSID" password "YOUR_WIFI_PASSWORD"

# Verify connection
ip addr show wlan0
ping -c 4 google.com
```

**Method B: Using netplan (Ubuntu's default)**
```bash
# Edit netplan configuration
sudo nano /etc/netplan/50-cloud-init.yaml
```

Add WiFi configuration:
```yaml
network:
  version: 2
  renderer: networkd
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "YOUR_WIFI_SSID":
          password: "YOUR_WIFI_PASSWORD"
```

Apply the configuration:
```bash
sudo netplan apply
```

### Step 4.3: Configure Static IP (Optional but Recommended)

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

For static IP:
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: false
      addresses:
        - 192.168.1.100/24
      routes:
        - to: default
          via: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
  wifis:
    wlan0:
      dhcp4: false
      addresses:
        - 192.168.1.101/24
      routes:
        - to: default
          via: 192.168.1.1
          metric: 100
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      access-points:
        "YOUR_WIFI_SSID":
          password: "YOUR_WIFI_PASSWORD"
```

```bash
sudo netplan apply
```

### Step 4.4: Find Your IP Address

```bash
# Show IP addresses
hostname -I

# More detailed view
ip addr show | grep "inet "
```

---

## 5. System Updates and Mirror Configuration

### Step 5.1: Backup Original Sources

```bash
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup
```

### Step 5.2: Configure Faster Mirrors (Optional)

For faster downloads, you can use regional mirrors.

**Edit sources file:**
```bash
sudo nano /etc/apt/sources.list
```

**Default Ubuntu Ports Configuration (for ARM64):**
```
deb http://ports.ubuntu.com/ubuntu-ports jammy main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports jammy-updates main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports jammy-backports main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports jammy-security main restricted universe multiverse
```

**For Users in China - Use Tsinghua Mirror:**
```
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports jammy main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports jammy-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports jammy-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports jammy-security main restricted universe multiverse
```

### Step 5.3: Update System

```bash
# Update package lists
sudo apt update

# Upgrade all packages
sudo apt upgrade -y

# Full system upgrade (handles dependencies)
sudo apt full-upgrade -y

# Clean up
sudo apt autoremove -y
sudo apt autoclean
```

### Step 5.4: Install Essential Packages

```bash
# Development essentials
sudo apt install -y build-essential git curl wget vim nano

# System utilities
sudo apt install -y htop tmux screen tree net-tools

# Python development
sudo apt install -y python3-pip python3-venv python3-dev

# Hardware interface tools
sudo apt install -y i2c-tools libraspberrypi-bin
```

---

## 6. Essential Software Installation

### Step 6.1: Enable Hardware Interfaces

```bash
# Install raspi-config if not present
sudo apt install -y raspi-config

# Run configuration tool
sudo raspi-config
```

In raspi-config, enable:
- **Interface Options → SSH** (if not already enabled)
- **Interface Options → I2C** (for sensors)
- **Interface Options → SPI** (if needed)
- **Interface Options → Serial Port** (for robot communication)
- **Interface Options → Camera** (if using camera)

### Step 6.2: Install Robot Dependencies

```bash
# Robot control libraries
sudo apt install -y python3-serial python3-smbus

# OpenCV for computer vision
sudo apt install -y python3-opencv

# GPIO libraries
pip3 install RPi.GPIO gpiozero

# Serial communication
pip3 install pyserial
```

### Step 6.3: Install ROS2 (if not pre-installed)

You have two options for installing ROS2: **Docker (Recommended)** or **Native Installation**.

---

#### Option A: Docker-Based ROS2 (Recommended)

Docker provides pre-built ROS2 binaries with Tier 1 support for ARM64, making it the easiest and most reliable way to run ROS2 on Raspberry Pi 5.

**Benefits of Docker:**
- Pre-built binaries (no compilation needed)
- Isolated environment (won't affect host system)
- Easy version switching and updates
- Consistent behavior across different hosts
- Quick setup (~5 minutes vs ~30+ minutes for native)

**Step 1: Install Docker**

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add your user to docker group (avoids needing sudo)
sudo usermod -aG docker $USER

# Apply group changes (or logout/login)
newgrp docker

# Verify installation
docker --version
docker run hello-world
```

**Step 2: Pull Official ROS2 Docker Images**

```bash
# ROS2 Humble - Minimal (ros-core)
docker pull ros:humble-ros-core

# ROS2 Humble - Base (includes common tools)
docker pull ros:humble-ros-base

# ROS2 Humble - Desktop (includes RViz, demos) - Larger image
docker pull ros:humble-desktop

# Alternative: OSRF images with more tools
docker pull osrf/ros:humble-desktop
```

**Available ROS2 Docker Image Tags:**

| Tag | Size | Description |
|-----|------|-------------|
| `ros:humble-ros-core` | ~500MB | Minimal ROS2 runtime |
| `ros:humble-ros-base` | ~700MB | Core + common packages |
| `ros:humble-desktop` | ~3GB | Full desktop with RViz |
| `osrf/ros:humble-desktop` | ~3GB | OSRF maintained, includes dev tools |

**Step 3: Run ROS2 in Docker**

```bash
# Basic interactive shell
docker run -it --rm ros:humble-ros-base

# With network access (for multi-machine ROS2)
docker run -it --rm --network host ros:humble-ros-base

# With hardware access (GPIO, I2C, serial ports)
docker run -it --rm \
  --privileged \
  --network host \
  -v /dev:/dev \
  ros:humble-ros-base

# With display support (for RViz/GUI apps)
docker run -it --rm \
  --privileged \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  ros:humble-desktop
```

**Step 4: Create a Custom Dockerfile for LanderPi**

Create `~/landerpi_ros/Dockerfile`:

```dockerfile
FROM ros:humble-ros-base

# Install additional ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for robot control
RUN pip3 install \
    pyserial \
    RPi.GPIO \
    smbus2

# Set up workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Source ROS2 in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["bash"]
```

Build and run:
```bash
cd ~/landerpi_ros
docker build -t landerpi-ros2 .
docker run -it --rm --privileged --network host -v /dev:/dev landerpi-ros2
```

**Step 5: Docker Compose for Multi-Container Setup**

Create `~/landerpi_ros/docker-compose.yml`:

```yaml
version: '3.8'

services:
  ros2-base:
    image: ros:humble-ros-base
    container_name: landerpi-ros2
    privileged: true
    network_mode: host
    volumes:
      - /dev:/dev
      - ./workspace:/ros2_ws
      - /tmp/.X11-unix:/tmp/.X11-unix
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
    command: bash -c "source /opt/ros/humble/setup.bash && bash"
    stdin_open: true
    tty: true

  talker:
    image: ros:humble-ros-base
    container_name: ros2-talker
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
    command: bash -c "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker"
    depends_on:
      - ros2-base

  listener:
    image: ros:humble-ros-base
    container_name: ros2-listener
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
    command: bash -c "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp listener"
    depends_on:
      - ros2-base
```

Run with:
```bash
docker compose up -d
docker compose logs -f
docker compose down
```

**Step 6: Useful Docker Commands for ROS2**

```bash
# List running containers
docker ps

# Execute command in running container
docker exec -it landerpi-ros2 bash

# Stop all ROS2 containers
docker stop $(docker ps -q --filter "ancestor=ros:humble-ros-base")

# Remove unused images
docker image prune

# View container logs
docker logs landerpi-ros2 -f
```

---

#### Option B: Native ROS2 Installation

If you prefer a native installation (without Docker), follow these steps.

> **Note:** ARM64 receives Tier 1 support with pre-built binaries. This installation takes longer (~30 minutes) but integrates directly with the host system.

```bash
# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install -y ros-humble-desktop  # Full desktop install (~3GB)
# Or: sudo apt install -y ros-humble-ros-base  # Minimal install (~700MB)

# Install development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Add to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

#### Comparison: Docker vs Native Installation

| Aspect | Docker | Native |
|--------|--------|--------|
| **Setup Time** | ~5 minutes | ~30+ minutes |
| **Disk Space** | Images can be shared | Full install per system |
| **Updates** | Pull new image | apt upgrade |
| **Isolation** | Fully isolated | System-wide |
| **Hardware Access** | Requires --privileged | Direct access |
| **Performance** | Slight overhead | Native performance |
| **Recommended For** | Development, testing | Production, embedded |

---

## 7. Remote Access Setup

### Step 7.1: SSH Configuration

```bash
# Ensure SSH is installed and running
sudo apt install -y openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh

# Check SSH status
sudo systemctl status ssh

# Configure firewall (if enabled)
sudo ufw allow 22/tcp
sudo ufw enable
```

**Connect from your computer:**
```bash
ssh pi@<RASPBERRY_PI_IP>
# Example: ssh pi@192.168.1.100
```

### Step 7.2: VNC Remote Desktop Setup

```bash
# Install VNC server
sudo apt install -y realvnc-vnc-server realvnc-vnc-viewer

# Or use TightVNC
sudo apt install -y tightvncserver

# Enable VNC
sudo raspi-config
# Interface Options → VNC → Enable

# Start VNC server
vncserver :1 -geometry 1920x1080 -depth 24
```

**Connect using VNC Viewer:**
1. Install VNC Viewer from `/2. Software/02 Remote Desktop Software/`
2. Open VNC Viewer and enter: `<RASPBERRY_PI_IP>:1`
3. Enter credentials when prompted

### Step 7.3: WonderPi Mobile App Setup

1. Install the WonderPi app:
   - **Android**: Install from `/2. Software/01 App Installation Package/1. Android/WonderPi-V5.5.2.apk`
   - **iOS**: Search "WonderPi" in App Store
2. Connect your phone to the same WiFi network as the robot
3. Open the app and it should auto-discover the LanderPi robot
4. Follow in-app instructions for pairing

---

## 8. Robot Software Setup

### Step 8.1: Clone LanderPi Software (if needed)

```bash
# Create workspace
mkdir -p ~/landerpi_ws/src
cd ~/landerpi_ws/src

# Clone LanderPi packages (check HiWonder documentation for exact repository)
# git clone <LANDERPI_REPO_URL>

# Build workspace
cd ~/landerpi_ws
colcon build
source install/setup.bash
```

### Step 8.2: Configure Auto-Start Services

```bash
# Create service file
sudo nano /etc/systemd/system/landerpi.service
```

Example service file:
```ini
[Unit]
Description=LanderPi Robot Service
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/landerpi_ws
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/pi/landerpi_ws/install/setup.bash && ros2 launch landerpi_bringup robot.launch.py"
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable the service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable landerpi.service
sudo systemctl start landerpi.service

# Check status
sudo systemctl status landerpi.service
```

### Step 8.3: Test Robot Hardware

```bash
# Test I2C devices
sudo i2cdetect -y 1

# Test serial ports
ls /dev/ttyUSB* /dev/ttyAMA* /dev/serial*

# Check camera
ls /dev/video*
```

---

## 9. Troubleshooting

### Common Issues and Solutions

#### Issue: Cannot Connect to WiFi
```bash
# Check WiFi interface
ip link show wlan0

# Restart NetworkManager
sudo systemctl restart NetworkManager

# Scan for networks
sudo nmcli device wifi rescan
nmcli device wifi list

# Check logs
journalctl -u NetworkManager -f
```

#### Issue: SSH Connection Refused
```bash
# Check SSH service
sudo systemctl status ssh

# Restart SSH
sudo systemctl restart ssh

# Check firewall
sudo ufw status
sudo ufw allow 22/tcp

# Check if port is listening
sudo ss -tlnp | grep 22
```

#### Issue: No Display Output
```bash
# Check connected displays
tvservice -s

# Force HDMI output (edit config)
sudo nano /boot/config.txt
# Add: hdmi_force_hotplug=1
```

#### Issue: Permission Denied for Hardware
```bash
# Add user to required groups
sudo usermod -aG gpio,i2c,spi,dialout $USER

# Apply changes (requires logout/login)
# Or reboot: sudo reboot
```

#### Issue: System Running Slow
```bash
# Check CPU temperature
vcgencmd measure_temp

# Check memory usage
free -h

# Check disk space
df -h

# Check running processes
top
```

#### Issue: Package Installation Fails
```bash
# Fix broken packages
sudo apt --fix-broken install
sudo dpkg --configure -a

# Clear cache and retry
sudo apt clean
sudo apt update
sudo apt upgrade
```

---

## 10. Quick Reference Commands

### System Management
```bash
sudo reboot                    # Reboot system
sudo poweroff                  # Shutdown system
vcgencmd measure_temp          # Check CPU temperature
free -h                        # Memory usage
df -h                          # Disk usage
htop                           # Process monitor
```

### Network
```bash
ip addr show                   # Show IP addresses
nmcli device wifi list         # List WiFi networks
ping -c 4 google.com           # Test connectivity
ss -tlnp                       # Show listening ports
```

### Service Management
```bash
sudo systemctl status <service>   # Check service status
sudo systemctl start <service>    # Start service
sudo systemctl stop <service>     # Stop service
sudo systemctl restart <service>  # Restart service
sudo systemctl enable <service>   # Enable on boot
journalctl -u <service> -f        # Follow service logs
```

### File Operations
```bash
ls -la                         # List files with details
cd /path/to/directory          # Change directory
pwd                            # Print working directory
cp source dest                 # Copy file
mv source dest                 # Move/rename file
rm filename                    # Delete file
chmod 755 script.sh            # Make executable
```

### Package Management
```bash
sudo apt update                # Update package lists
sudo apt upgrade -y            # Upgrade packages
sudo apt install <package>     # Install package
sudo apt remove <package>      # Remove package
sudo apt autoremove            # Clean unused packages
```

### ROS2 Commands
```bash
ros2 topic list                # List active topics
ros2 topic echo /topic_name    # Monitor topic data
ros2 node list                 # List active nodes
ros2 launch <pkg> <launch.py>  # Launch file
ros2 run <pkg> <executable>    # Run node
```

### Docker Commands (for ROS2)
```bash
docker pull ros:humble-ros-base       # Pull ROS2 image
docker run -it --rm ros:humble-ros-base  # Run interactive container
docker run -it --rm --privileged --network host -v /dev:/dev ros:humble-ros-base  # With hardware access
docker ps                             # List running containers
docker exec -it <container> bash      # Enter running container
docker compose up -d                  # Start compose services
docker compose down                   # Stop compose services
docker logs <container> -f            # Follow container logs
docker image prune                    # Clean unused images
```

---

## Additional Resources

### Files Created by This Setup Process

| File | Purpose |
|------|---------|
| `/Users/ZDZ/Documents/gitrepo/personalproject/hiwonderSetup/LanderPi_Ubuntu_Setup_Guide.md` | This guide |
| `/Users/ZDZ/Documents/gitrepo/personalproject/hiwonderSetup/LanderPi_Linux_Command_Reference.md` | Linux command reference |

### HiWonder LanderPi Documentation Structure

```
/Users/ZDZ/Downloads/LanderPi/1. Tutorials/
├── 1. LanderPi User Manual/          # Main user manual
├── 2. Linux Basic Lesson/            # Linux fundamentals
├── 3. Chassis Motion Control/        # Movement control
├── 4. Lidar Course/                  # LiDAR setup and usage
├── 5. Depth Camera Basic Course/     # Depth camera tutorials
├── 6. Mapping & Navigation Course/   # SLAM and navigation
├── 7. ROS+OpenCV Course/             # Computer vision
├── 8. ROS+ Machine Learning Course/  # ML applications
├── 9. Robotic Arm Control/           # Arm manipulation
├── 10. MoveIt & Gazebo Simulation/   # Simulation
├── 11. Voice Control Course/         # Voice commands
├── 12. Large AI Model Course/        # AI integration
└── 13. Group Control/                # Multi-robot control
```

### Useful Links

- HiWonder Official: https://www.hiwonder.com
- Ubuntu Downloads: https://ubuntu.com/download/raspberry-pi
- Raspberry Pi Imager: https://www.raspberrypi.com/software/
- ROS2 Documentation: https://docs.ros.org/en/humble/
- ROS2 Docker Images: https://hub.docker.com/_/ros
- Docker Installation: https://docs.docker.com/engine/install/

---

## Summary Checklist

Use this checklist to track your setup progress:

- [ ] **Hardware prepared** - SD card, power supply, cables
- [ ] **System image downloaded** and flashed to SD card
- [ ] **First boot completed** - System resized and configured
- [ ] **Password changed** from default
- [ ] **Network configured** - WiFi or Ethernet connected
- [ ] **System updated** - `apt update && apt upgrade`
- [ ] **Essential packages installed**
- [ ] **SSH enabled** and accessible remotely
- [ ] **VNC configured** (optional) for remote desktop
- [ ] **Hardware interfaces enabled** - I2C, SPI, Serial
- [ ] **Docker installed** (recommended for ROS2)
- [ ] **ROS2 installed** - via Docker or native
- [ ] **Robot software installed** and tested
- [ ] **Auto-start services configured** (optional)
- [ ] **WonderPi app connected** (optional)

---

*Guide compiled from HiWonder LanderPi tutorials and documentation.*
*Last updated: December 2024*
