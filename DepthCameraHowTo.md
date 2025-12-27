# Aurora 930 Depth Camera - Setup Guide for Raspberry Pi 5

This guide documents how to set up and use the Deptrum Aurora 930 depth camera on a Raspberry Pi 5 running Ubuntu, based on HiWonder's official documentation.

## Camera Specifications

| Category | Parameter | Value |
|----------|-----------|-------|
| **Dimensions** | Overall | 76.5 × 20.7 × 21.8 mm |
| **Interface** | Connection | USB 2.0 Wafer Connector |
| **Depth** | Resolution | 640 × 400 (or 320 × 240) |
| **Depth** | Frame Rate | 5-15 fps |
| **Depth** | Range | 30 - 300 cm |
| **Depth** | Accuracy | 8mm @ 1m |
| **Depth** | Precision | 3mm @ 0.5m, 7mm @ 1m |
| **RGB** | Resolution | 640 × 400 (or 320 × 240) |
| **RGB** | Frame Rate | 5-15 fps |
| **RGB** | Format | NV12 |
| **IR** | Resolution | 640 × 400 (or 320 × 240) |
| **IR** | Format | Raw 8 |
| **FOV** | Field of View | H71° × V46° |
| **Power** | Supply | 5V ± 10%, 1.5A |
| **Power** | Consumption | < 1.6W average |
| **Safety** | Laser | Class 1 |
| **Environment** | Temperature | -10°C to 55°C |
| **Environment** | Humidity | 0-95% non-condensing |
| **Environment** | Illumination | 3-80000 Lux |

### Camera Components
1. Fixed Wing (Steel Plate, detachable)
2. IR Camera (Infrared Camera Module)
3. RGB Camera (RGB Camera Module)
4. Infrared Illuminator
5. Silicone Sleeve (detachable, replaceable)
6. Dot Projector
7. USB Wafer Connector

## Required Files

The ROS2 driver is included in this repository at `drivers/depth-camera/`.

### For Raspberry Pi 5 (ARM64):
| File | Purpose |
|------|---------|
| `drivers/depth-camera/deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz` | ROS2 Humble driver (ARM64) |

### ROS2 Driver Contents:
- `launch_aurora930/` - ROS2 launch files
- `ext/deptrum-stream-aurora900-linux-aarch64-v1.1.19-18.04/` - Core SDK
- `scripts/setup_udev_rules.sh` - udev rules installer
- `scripts/install_dependency.sh` - Dependency installer
- `scripts/99-deptrum-libusb.rules` - USB permission rules

## Installation Steps (Docker-Based - Recommended)

Since the Pi is running Ubuntu 25.10 and ROS2 Humble requires Ubuntu 22.04, we use Docker for the ROS2 environment.

### Step 1: Transfer Files to Raspberry Pi

```bash
# From this repository directory, copy driver to Pi
scp drivers/depth-camera/deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz user@<PI_IP>:~/
```

### Step 2: Create ROS2 Workspace

```bash
# On Raspberry Pi
mkdir -p ~/deptrum_ws/src
```

### Step 3: Extract ROS2 Driver

```bash
tar -xzvf deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz -C ~/deptrum_ws/src/
```

### Step 4: Install System Dependencies

```bash
# Update system
sudo apt update

# Install USB libraries (needed on host for udev rules)
sudo apt install -y libusb-1.0-0-dev libudev-dev
```

### Step 5: Setup udev Rules

```bash
# Install udev rules for camera USB access
cd ~/deptrum_ws/src/deptrum-ros-driver-aurora930-0.2.1001/ext/deptrum-stream-aurora900-linux-aarch64-v1.1.19-18.04/scripts
sudo bash setup_udev_rules.sh

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The udev rules file (`99-deptrum-libusb.rules`) grants USB access to the Deptrum cameras.

### Step 6: Fix Package Name (Required)

The package.xml has a name mismatch that must be fixed before building:

```bash
# Fix the package name to match CMake project name
sed -i 's/<name>deptrum-ros-driver<\/name>/<name>deptrum-ros-driver-aurora930<\/name>/' \
    ~/deptrum_ws/src/deptrum-ros-driver-aurora930-0.2.1001/package.xml
```

### Step 7: Build ROS2 Package in Docker

```bash
# Build using the landerpi-ros2 Docker image
docker run --rm \
    -v ~/deptrum_ws:/deptrum_ws \
    -w /deptrum_ws \
    landerpi-ros2:latest \
    bash -c "source /opt/ros/humble/setup.bash && \
             colcon build --cmake-args -DSTREAM_SDK_TYPE=AURORA930"
```

The build should complete in ~30 seconds with only deprecation warnings.

### Step 8: Connect Camera and Verify

1. Connect Aurora 930 to Raspberry Pi USB port
2. Verify detection:
   ```bash
   lsusb | grep 3251
   # Should show: ID 3251:1930 Linux Foundation Aurora 930
   ```
3. Verify USB permissions:
   ```bash
   ls -la /dev/bus/usb/002/  # Adjust bus number as needed
   # Camera device should have 0666 permissions
   ```

## Usage

### Launch Camera Driver (Docker)

```bash
# Launch camera driver in Docker
docker run --rm -it \
    --privileged \
    -v /dev:/dev \
    -v ~/deptrum_ws:/deptrum_ws \
    landerpi-ros2:latest \
    bash -c "source /opt/ros/humble/setup.bash && \
             source /deptrum_ws/install/local_setup.bash && \
             ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py"
```

Expected output on successful startup:
```
device_count==========1
BOOT_ORDER_1::Found 1 deptrum device
Device information:
    sdk version: 1.1.19
    device name: Aurora 930
    serial number: HY400516001015B03G00151
    firmware version: 1.7.2
Camera Parameters:
    Resolution: 640x400
```

### View Camera Output (RViz)

In a second terminal:
```bash
ros2 launch deptrum-ros-driver-aurora930 viewer930_launch.py
```

### Available Launch Files

| Launch File | Description |
|-------------|-------------|
| `aurora930_launch.py` | Main camera driver |
| `viewer930_launch.py` | RViz visualization |
| `aurora930_multi_launch.py` | Multi-camera support |
| `sub_node_ci_aurora930_launch.py` | CI/testing node |

### ROS2 Topics (Expected)

Based on the driver, these topics should be published:
- `/camera/depth/image_raw` - Depth image
- `/camera/rgb/image_raw` - RGB image
- `/camera/ir/image_raw` - Infrared image
- `/camera/depth/points` - Point cloud
- `/camera/camera_info` - Camera calibration info

## Docker Alternative

If native installation has issues, use Docker:

```bash
# Build Docker image with ROS2 and camera driver
docker build -t landerpi-ros2-camera -f Dockerfile.camera .

# Run with USB device access
docker run -it --rm \
  --device=/dev/bus/usb \
  --privileged \
  -v /dev:/dev \
  landerpi-ros2-camera
```

## Troubleshooting

### Camera Not Detected

1. **Check USB connection:**
   ```bash
   lsusb | grep 3251
   ```

2. **Verify udev rules:**
   ```bash
   cat /etc/udev/rules.d/99-deptrum-libusb.rules
   ```

3. **Check USB permissions:**
   ```bash
   ls -la /dev/bus/usb/002/  # Adjust bus number
   ```

4. **Re-run udev setup:**
   ```bash
   sudo bash setup_udev_rules.sh
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

### Build Errors

1. **Missing ROS2:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Missing ament_cmake:**
   ```bash
   sudo apt install ros-humble-ament-cmake
   ```

3. **Wrong SDK type:**
   ```bash
   colcon build --cmake-args -DSTREAM_SDK_TYPE=AURORA930
   ```

### Runtime Errors

1. **Library not found:**
   ```bash
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/deptrum_ws/install/lib
   ```

2. **Permission denied:**
   - Ensure user is in `video` and `plugdev` groups
   - Check udev rules are installed

### RViz Not Showing Data

1. Check if driver is publishing:
   ```bash
   ros2 topic list
   ros2 topic echo /camera/depth/image_raw
   ```

2. Check RViz display settings match topic names

## Important Notes

1. **NOT Orbbec SDK Compatible**: The Aurora 930 uses Deptrum's proprietary SDK, not the standard Orbbec SDK (pyorbbecsdk).

2. **USB 2.0 Limitation**: The camera uses USB 2.0 (480Mbps), which limits bandwidth for high-resolution streaming.

3. **ARM64 Specific**: Use the `aarch64` versions of all packages for Raspberry Pi 5.

4. **ROS2 Humble Required**: The driver is built for ROS2 Humble on Ubuntu 22.04.

## File Locations Reference

### Repository Structure
```
hiwonderSetup/
├── drivers/
│   └── depth-camera/
│       ├── deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz  <-- ROS2 Driver
│       └── README.md
├── DepthCameraHowTo.md  <-- This file
├── CameraCheck.md       <-- Camera status report
├── setup_landerpi.py    <-- Setup script (includes camera udev rules)
└── checkLanderPi.py     <-- Health check (includes camera checks)
```

### Original HiWonder Documentation
The driver was sourced from HiWonder's LanderPi documentation:
`1. Tutorials/5. Depth Camera Basic Course/4 Appendix/2 Source Code/ROS2/`
