# Aurora 930 Depth Camera - Complete Setup Guide for Raspberry Pi 5

This guide documents how to set up, configure, and validate the Deptrum Aurora 930 depth camera on a Raspberry Pi 5 running Ubuntu, based on HiWonder's official documentation and driver source analysis.

## Table of Contents

1. [Camera Specifications](#camera-specifications)
2. [Required Files](#required-files)
3. [Installation Steps](#installation-steps-docker-based---recommended)
4. [ROS2 Interface](#ros2-interface)
5. [Configuration Parameters](#configuration-parameters)
6. [Usage Examples](#usage-examples)
7. [Validation Script Guide](#validation-script-guide)
8. [Troubleshooting](#troubleshooting)
9. [Performance Optimization](#performance-optimization)

---

## Camera Specifications

### Physical Specifications

| Category | Parameter | Value |
|----------|-----------|-------|
| **Dimensions** | Overall | 76.5 x 20.7 x 21.8 mm |
| **Weight** | Approximate | ~30g |
| **Interface** | Connection | USB 2.0 Wafer Connector |
| **USB ID** | VID:PID | 3251:1930 |

### Optical Specifications

| Category | Parameter | Value |
|----------|-----------|-------|
| **FOV** | Field of View | H71 x V46 |
| **Baseline** | Stereo Distance | 40 mm |

### Depth Performance

| Parameter | Value |
|-----------|-------|
| **Resolution** | 640 x 400, 480 x 300, or 320 x 200 |
| **Frame Rate** | 5-15 fps |
| **Operating Range** | 30 - 300 cm (0.3m - 3.0m) |
| **Accuracy** | 8mm @ 1m |
| **Precision** | 3mm @ 0.5m, 7mm @ 1m |

### RGB Performance

| Parameter | Value |
|-----------|-------|
| **Resolution** | 640 x 400, 480 x 300, or 320 x 200 |
| **Frame Rate** | 5-15 fps |
| **Format** | NV12 (converted to BGR8 by driver) |

### Infrared Performance

| Parameter | Value |
|-----------|-------|
| **Resolution** | 640 x 400, 480 x 300, or 320 x 200 |
| **Frame Rate** | 5-15 fps |
| **Format** | Raw 8 (MONO8) |

### Power & Environment

| Parameter | Value |
|-----------|-------|
| **Power Supply** | 5V +/- 10%, 1.5A |
| **Power Consumption** | < 1.6W average |
| **Laser Safety** | Class 1 |
| **Temperature** | -10C to 55C |
| **Humidity** | 0-95% non-condensing |
| **Illumination** | 3-80000 Lux |

### Camera Components

1. **Fixed Wing** - Steel plate (detachable)
2. **IR Camera** - Infrared camera module
3. **RGB Camera** - Color camera module
4. **Infrared Illuminator** - Active IR light source
5. **Silicone Sleeve** - Detachable, replaceable
6. **Dot Projector** - Structured light pattern emitter
7. **USB Wafer Connector** - Data and power interface

---

## Required Files

The ROS2 driver is included in this repository at `drivers/depth-camera/`.

### For Raspberry Pi 5 (ARM64)

| File | Purpose |
|------|---------|
| `drivers/depth-camera/deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz` | ROS2 Humble driver (ARM64) |

### ROS2 Driver Contents

```
deptrum-ros-driver-aurora930-0.2.1001/
├── CMakeLists.txt
├── package.xml
├── include/deptrum_ros_driver/
│   ├── aurora900_ros2_device.h
│   ├── ros2_device.h
│   └── ros2_types.h
├── src/
│   ├── aurora900_ros2_device.cc
│   └── ros2_device.cc
├── launch_aurora930/launch/
│   ├── aurora930_launch.py          # Main camera driver
│   ├── aurora930_multi_launch.py    # Multi-camera support
│   ├── viewer930_launch.py          # RViz visualization
│   └── sub_node_ci_aurora930_launch.py
├── ext/deptrum-stream-aurora900-linux-aarch64-v1.1.19-18.04/
│   ├── lib/                         # Core SDK libraries
│   └── scripts/
│       ├── setup_udev_rules.sh      # udev rules installer
│       ├── install_dependency.sh    # Dependency installer
│       └── 99-deptrum-libusb.rules  # USB permission rules
└── docs/
    └── usage_aurora930.md
```

### Driver Version Information

- **Driver Version**: 0.2.1001
- **SDK Version**: 1.1.19
- **ROS2 Support**: Foxy, Galactic, Humble
- **Target Platform**: ARM64 (aarch64)

---

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

The udev rules file (`99-deptrum-libusb.rules`) grants USB access to cameras with VID 3251.

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

---

## ROS2 Interface

### ROS2 Topics Published

The driver publishes the following topics under the default namespace `aurora`:

#### Image Topics

| Topic | Message Type | Description | Format |
|-------|-------------|-------------|--------|
| `/aurora/rgb/image_raw` | `sensor_msgs/msg/Image` | Raw RGB color image | BGR8 |
| `/aurora/ir/image_raw` | `sensor_msgs/msg/Image` | Raw infrared image | MONO8 |
| `/aurora/depth/image_raw` | `sensor_msgs/msg/Image` | Raw depth image | MONO16 (mm) |

#### Point Cloud Topics

| Topic | Message Type | Description | Format |
|-------|-------------|-------------|--------|
| `/aurora/points2` | `sensor_msgs/msg/PointCloud2` | 3D point cloud | XYZ (meters) or XYZRGB |

#### Camera Info Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/aurora/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB camera calibration |
| `/aurora/ir/camera_info` | `sensor_msgs/msg/CameraInfo` | IR/depth camera calibration |

### ROS2 Services

**None** - This driver is a pure publisher node with no services.

### ROS2 Subscriptions

**None** - This driver operates autonomously without subscribing to any topics.

### Coordinate Frames

```
camera_base (root frame)
├── rgb_camera_link (RGB camera optical frame)
└── depth_camera_link (depth/IR camera optical frame)
```

### Message Details

**Depth Image:**
- Encoding: `mono16` (16-bit unsigned integer)
- Units: Millimeters (mm)
- Valid range: 150mm to 4000mm (configurable)
- Invalid pixels: 0 value

**RGB Image:**
- Encoding: `bgr8` (OpenCV convention)
- Converted from camera's NV12 format

**Point Cloud:**
- Coordinate system: X=right, Y=down, Z=forward
- Units: Meters (converted from mm)
- Points with z <= 0 are excluded

---

## Configuration Parameters

### Resolution Modes

The `resolution_mode_index` parameter selects from supported resolutions:

| Index | IR Resolution | RGB Resolution | Depth Resolution | Recommended Use |
|-------|--------------|----------------|------------------|-----------------|
| 0 | 640x400 | 640x400 | 640x400 | High quality |
| 1 | 480x300 | 480x300 | 480x300 | Balanced |
| 2 | 320x200 | 320x200 | 320x200 | Low latency |

### Core Enable/Disable Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rgb_enable` | bool | `true` | Enable RGB color image stream |
| `ir_enable` | bool | `true` | Enable infrared (IR) image stream |
| `depth_enable` | bool | `true` | Enable depth image stream |
| `point_cloud_enable` | bool | `true` | Enable 3D point cloud generation |
| `rgbd_enable` | bool | `false` | Enable RGBD mode (colored point cloud) |

### Frame Rate Parameters

| Parameter | Type | Default | Valid Range | Description |
|-----------|------|---------|-------------|-------------|
| `ir_fps` | int | `15` | 5, 10, 12, 15 | IR/depth frame rate |
| `rgb_fps` | int | `15` | 5-30 | RGB frame rate (must be <= ir_fps) |

### Camera Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `resolution_mode_index` | int | `2` | Resolution mode (0-2) |
| `usb_port_number` | string | `""` | Select camera by USB port |
| `serial_number` | string | `""` | Select camera by serial number |
| `boot_order` | int | `1` | Device boot order (multi-camera) |

### Exposure & Gain

| Parameter | Type | Default | Valid Range | Description |
|-----------|------|---------|-------------|-------------|
| `exposure_enable` | bool | `false` | - | Enable manual exposure (false=auto) |
| `exposure_time` | int | `1` | 1-31 | Exposure time (0.1ms units) |
| `gain_enable` | bool | `false` | - | Enable manual gain control |
| `gain_value` | int | `20` | 10-160 | Manual gain value |

### Depth Processing

| Parameter | Type | Default | Valid Range | Description |
|-----------|------|---------|-------------|-------------|
| `depth_correction` | bool | `true` | - | Enable depth correction |
| `align_mode` | bool | `true` | - | Align depth to RGB coordinates |
| `threshold_size` | int | `110` | 30-400 | Noise filter size (smaller=more filtering) |
| `minimum_filter_depth_value` | int | `150` | 150-300 | Min valid depth (mm) |
| `maximum_filter_depth_value` | int | `4000` | - | Max valid depth (mm) |

### Laser Power

| Parameter | Type | Default | Valid Range | Description |
|-----------|------|---------|-------------|-------------|
| `laser_power` | float | `1.0` | 1.0-3.0 | 1=auto, 2=indoor, 3=outdoor |

### Logging

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `log_dir` | string | `"/tmp/"` | Log file directory |
| `stream_sdk_log_enable` | bool | `true` | Enable SDK logging |

---

## Usage Examples

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

```bash
ros2 launch deptrum-ros-driver-aurora930 viewer930_launch.py
```

### Basic ROS2 Commands

```bash
# List all topics
ros2 topic list

# Check topic publishing rate
ros2 topic hz /aurora/depth/image_raw

# View single message
ros2 topic echo /aurora/rgb/camera_info --once

# View depth image stats
ros2 topic echo /aurora/depth/image_raw --field header
```

### Available Launch Files

| Launch File | Description |
|-------------|-------------|
| `aurora930_launch.py` | Main camera driver (namespace: `aurora`) |
| `viewer930_launch.py` | Camera + RViz visualization |
| `aurora930_multi_launch.py` | Multi-camera support (`aurora_1`, `aurora_2`) |
| `sub_node_ci_aurora930_launch.py` | CI/testing node |

---

## Validation Script Guide

The following information supports creating a `test_cameradepth.py` validation script.

### Required Dependencies

```python
# Python packages
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
import numpy as np
```

### Topics to Validate

```python
CAMERA_TOPICS = {
    'rgb': '/aurora/rgb/image_raw',
    'ir': '/aurora/ir/image_raw',
    'depth': '/aurora/depth/image_raw',
    'points': '/aurora/points2',
    'rgb_info': '/aurora/rgb/camera_info',
    'ir_info': '/aurora/ir/camera_info',
}
```

### Expected Message Properties

```python
# Depth image validation
def validate_depth_image(msg):
    """Validate depth image message."""
    assert msg.encoding == 'mono16', f"Expected mono16, got {msg.encoding}"
    assert msg.width in [640, 480, 320], f"Unexpected width: {msg.width}"
    assert msg.height in [400, 300, 200], f"Unexpected height: {msg.height}"

    # Convert to numpy array
    bridge = CvBridge()
    depth = bridge.imgmsg_to_cv2(msg, "mono16")

    # Check for valid depth values (150-4000mm)
    valid_mask = (depth > 150) & (depth < 4000)
    valid_percentage = np.sum(valid_mask) / depth.size * 100

    return {
        'width': msg.width,
        'height': msg.height,
        'valid_percentage': valid_percentage,
        'min_depth_mm': np.min(depth[valid_mask]) if np.any(valid_mask) else 0,
        'max_depth_mm': np.max(depth[valid_mask]) if np.any(valid_mask) else 0,
    }

# RGB image validation
def validate_rgb_image(msg):
    """Validate RGB image message."""
    assert msg.encoding == 'bgr8', f"Expected bgr8, got {msg.encoding}"
    return {
        'width': msg.width,
        'height': msg.height,
        'channels': 3,
    }

# IR image validation
def validate_ir_image(msg):
    """Validate IR image message."""
    assert msg.encoding == 'mono8', f"Expected mono8, got {msg.encoding}"
    return {
        'width': msg.width,
        'height': msg.height,
    }
```

### Sample Validation Node

```python
#!/usr/bin/env python3
"""Aurora 930 Depth Camera Validation Script."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
import numpy as np

class CameraValidator(Node):
    def __init__(self):
        super().__init__('camera_validator')
        self.bridge = CvBridge()
        self.results = {}

        # Subscribe to all camera topics
        self.rgb_sub = self.create_subscription(
            Image, '/aurora/rgb/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/aurora/depth/image_raw', self.depth_callback, 10)
        self.ir_sub = self.create_subscription(
            Image, '/aurora/ir/image_raw', self.ir_callback, 10)
        self.points_sub = self.create_subscription(
            PointCloud2, '/aurora/points2', self.points_callback, 10)
        self.rgb_info_sub = self.create_subscription(
            CameraInfo, '/aurora/rgb/camera_info', self.rgb_info_callback, 10)

    def rgb_callback(self, msg):
        self.results['rgb'] = {
            'status': 'OK',
            'encoding': msg.encoding,
            'resolution': f"{msg.width}x{msg.height}",
        }
        self.get_logger().info(f"RGB: {msg.width}x{msg.height} {msg.encoding}")

    def depth_callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, "mono16")
        valid = (depth > 150) & (depth < 4000)
        self.results['depth'] = {
            'status': 'OK',
            'encoding': msg.encoding,
            'resolution': f"{msg.width}x{msg.height}",
            'valid_percentage': f"{np.sum(valid)/depth.size*100:.1f}%",
            'range_mm': f"{np.min(depth[valid])}-{np.max(depth[valid])}" if np.any(valid) else "N/A",
        }
        self.get_logger().info(f"Depth: {msg.width}x{msg.height} valid={np.sum(valid)/depth.size*100:.1f}%")

    def ir_callback(self, msg):
        self.results['ir'] = {
            'status': 'OK',
            'encoding': msg.encoding,
            'resolution': f"{msg.width}x{msg.height}",
        }
        self.get_logger().info(f"IR: {msg.width}x{msg.height} {msg.encoding}")

    def points_callback(self, msg):
        self.results['points'] = {
            'status': 'OK',
            'width': msg.width,
            'height': msg.height,
            'point_step': msg.point_step,
            'fields': [f.name for f in msg.fields],
        }
        self.get_logger().info(f"PointCloud: {msg.width}x{msg.height} points")

    def rgb_info_callback(self, msg):
        self.results['camera_info'] = {
            'status': 'OK',
            'frame_id': msg.header.frame_id,
            'resolution': f"{msg.width}x{msg.height}",
        }
        self.get_logger().info(f"CameraInfo: {msg.width}x{msg.height}")

def main():
    rclpy.init()
    validator = CameraValidator()

    # Spin for a few seconds to collect data
    import time
    start = time.time()
    while time.time() - start < 5.0:
        rclpy.spin_once(validator, timeout_sec=0.1)

    # Print results
    print("\n" + "="*50)
    print("CAMERA VALIDATION RESULTS")
    print("="*50)
    for topic, result in validator.results.items():
        print(f"\n{topic.upper()}:")
        for key, value in result.items():
            print(f"  {key}: {value}")

    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Troubleshooting

### Camera Not Detected

**Symptoms:** `lsusb` doesn't show device ID 3251:1930

**Solutions:**
1. Check USB cable connection - try different port
2. Verify power - camera needs 1.5A @ 5V
3. Check USB hub - try direct connection to Pi

### Camera Detected but Driver Fails

**Symptoms:** "No deptrum device connected! It's going to quit..."

**Solutions:**
1. Verify udev rules:
   ```bash
   cat /etc/udev/rules.d/99-deptrum-libusb.rules
   ```
2. Reload udev rules:
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
3. Check USB permissions:
   ```bash
   ls -la /dev/bus/usb/*/
   ```
4. Ensure user is in required groups:
   ```bash
   groups $USER
   # Should include: dialout video plugdev
   ```

### Build Errors

**Symptoms:** colcon build fails

**Solutions:**
1. Ensure ROS2 is sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Use correct SDK type:
   ```bash
   colcon build --cmake-args -DSTREAM_SDK_TYPE=AURORA930
   ```
3. Fix package name in package.xml (see Step 6)

### Frame Timeout

**Symptoms:** "Get frames timeout!" warnings

**Solutions:**
1. Reduce frame rate: Set `ir_fps=5`
2. Disable unused streams: Set `point_cloud_enable=false`
3. Use lower resolution: Set `resolution_mode_index=2`

### Depth Artifacts / Noise

**Symptoms:** Noisy depth values, holes in depth image

**Solutions:**
1. Adjust filter size: Increase `threshold_size` (30-400)
2. Set depth range: Adjust `minimum_filter_depth_value` and `maximum_filter_depth_value`
3. Adjust laser power: Set `laser_power=2` for indoor, `3` for outdoor
4. Enable depth correction: Set `depth_correction=true`

### RViz Not Showing Data

**Solutions:**
1. Check topic list:
   ```bash
   ros2 topic list
   ```
2. Check publishing rate:
   ```bash
   ros2 topic hz /aurora/depth/image_raw
   ```
3. Verify RViz display topics match driver topics

### Log Files

- **ROS2 Driver Logs:** `/tmp/deptrum_ros_driver_*.INFO`
- **SDK Logs:** `/tmp/deptrum-stream.log`
- **Latest Log Symlink:** `/tmp/node.INFO`

---

## Performance Optimization

### Reduce Latency

1. Increase frame rate: `ir_fps=15`
2. Disable point cloud: `point_cloud_enable=false`
3. Use lower resolution: `resolution_mode_index=2`

### Reduce CPU/Bandwidth

1. Lower frame rate: `ir_fps=5`
2. Disable unused streams: `rgb_enable=false` if not needed
3. Use smaller resolution: `resolution_mode_index=2`

### Multi-Camera Setup

1. Set unique namespaces via launch file
2. Use USB port selection: `usb_port_number`
3. Set `ROS_DOMAIN_ID` environment variable
4. Use separate USB controllers if possible

---

## Important Notes

1. **NOT Orbbec SDK Compatible**: The Aurora 930 uses Deptrum's proprietary SDK, not the standard Orbbec SDK (pyorbbecsdk).

2. **USB 2.0 Limitation**: The camera uses USB 2.0 (480Mbps), which limits bandwidth for high-resolution streaming.

3. **ARM64 Specific**: Use the `aarch64` versions of all packages for Raspberry Pi 5.

4. **ROS2 Humble Required**: The driver is built for ROS2 Humble on Ubuntu 22.04.

5. **Docker Required**: Ubuntu 25.10 on Pi requires Docker to run ROS2 Humble.

---

## File Locations Reference

### Repository Structure

```
hiwonderSetup/
├── drivers/
│   └── depth-camera/
│       ├── deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz
│       └── README.md
├── DepthCameraHowTo.md          <-- This file
├── Aurora930_ROS2_Interface_Analysis.md  <-- Detailed ROS2 API reference
├── CameraCheck.md               <-- Camera status report
├── setup_landerpi.py            <-- Setup script
└── checkLanderPi.py             <-- Health check
```

### Original HiWonder Documentation

The driver was sourced from HiWonder's LanderPi documentation:
- Path: `1. Tutorials/5. Depth Camera Basic Course/4 Appendix/2 Source Code/ROS2/`

Additional documentation:
- Camera Overview: `1 Deptrum Aurora 930 Depth Camera Overview.pdf`
- ROS2 Setup: `1 Depth Camera Setup and Usage in ROS2.pdf`
- General Setup: `1 Depth Camera Setup and Usage.pdf`
