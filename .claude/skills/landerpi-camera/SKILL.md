---
name: landerpi-camera
description: Depth camera control for HiWonder LanderPi robot. Provides Aurora 930 camera setup, ROS2 driver configuration, image/depth/point cloud streaming, and camera validation. Requires landerpi-core skill to be loaded first for connection and Docker setup.
---

# LanderPi Depth Camera Control

## Overview

Depth camera control skill for HiWonder LanderPi with Deptrum Aurora 930 camera. Provides driver management, multi-stream data access (RGB, IR, depth, point cloud), and camera validation tools.

**Prerequisites:**
- Load `landerpi-core` skill for connection and Docker setup
- Camera requires Docker-based ROS2 environment

## Camera Commands

| Command | Purpose |
|---------|---------|
| `uv run python test_cameradepth.py check` | Check camera + Docker + ROS2 |
| `uv run python test_cameradepth.py start-driver` | Start camera driver in Docker |
| `uv run python test_cameradepth.py stream --samples 5` | Read camera streams |
| `uv run python test_cameradepth.py stop-driver` | Stop camera driver container |
| `uv run python test_cameradepth.py validate` | Full camera validation |

### ROS2 Stack Testing

For ROS2-based testing (requires deployed stack):
```bash
uv run python test_cameradepth_ros2.py check   # Check camera topics
uv run python test_cameradepth_ros2.py stream  # Read stream samples
```

## Hardware Configuration

| Property | Value |
|----------|-------|
| Camera Model | Deptrum Aurora 930 |
| USB ID | VID:PID 3251:1930 |
| Interface | USB 2.0 Wafer Connector |
| Docker Image | `landerpi-ros2:latest` |

### Resolution Modes

| Index | Resolution | Frame Rate | Use Case |
|-------|------------|------------|----------|
| 0 | 640x400 | 5-15 fps | High quality |
| 1 | 480x300 | 5-15 fps | Balanced |
| 2 | 320x200 | 5-15 fps | Low latency |

### Depth Performance

| Parameter | Value |
|-----------|-------|
| Operating Range | 30 - 300 cm |
| Accuracy | 8mm @ 1m |
| Precision | 3mm @ 0.5m, 7mm @ 1m |
| FOV | H71 x V46 |
| Baseline | 40mm |

## ROS2 Topics

Default namespace: `aurora`

### Image Topics

| Topic | Message Type | Format | Description |
|-------|--------------|--------|-------------|
| `/aurora/rgb/image_raw` | `sensor_msgs/msg/Image` | BGR8 | RGB color image |
| `/aurora/ir/image_raw` | `sensor_msgs/msg/Image` | MONO8 | Infrared image |
| `/aurora/depth/image_raw` | `sensor_msgs/msg/Image` | MONO16 (mm) | Depth image |

### Point Cloud Topic

| Topic | Message Type | Format | Description |
|-------|--------------|--------|-------------|
| `/aurora/points2` | `sensor_msgs/msg/PointCloud2` | XYZ (meters) | 3D point cloud |

### Camera Info Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/aurora/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB calibration |
| `/aurora/ir/camera_info` | `sensor_msgs/msg/CameraInfo` | IR/depth calibration |

## ROS2 Parameters

### Stream Control

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rgb_enable` | bool | `true` | Enable RGB stream |
| `ir_enable` | bool | `true` | Enable IR stream |
| `depth_enable` | bool | `true` | Enable depth stream |
| `point_cloud_enable` | bool | `true` | Enable point cloud |
| `rgbd_enable` | bool | `false` | Enable colored point cloud |

### Frame Rate

| Parameter | Type | Default | Valid Range | Description |
|-----------|------|---------|-------------|-------------|
| `ir_fps` | int | `15` | 5, 10, 12, 15 | IR/depth frame rate |
| `rgb_fps` | int | `15` | 5-30 | RGB frame rate |
| `resolution_mode_index` | int | `2` | 0-2 | Resolution selection |

### Depth Processing

| Parameter | Type | Default | Valid Range | Description |
|-----------|------|---------|-------------|-------------|
| `depth_correction` | bool | `true` | - | Enable depth correction |
| `align_mode` | bool | `true` | - | Align depth to RGB |
| `threshold_size` | int | `110` | 30-400 | Noise filter size |
| `minimum_filter_depth_value` | int | `150` | 150-300 | Min depth (mm) |
| `maximum_filter_depth_value` | int | `4000` | - | Max depth (mm) |

### Exposure Control

| Parameter | Type | Default | Valid Range | Description |
|-----------|------|---------|-------------|-------------|
| `exposure_enable` | bool | `false` | - | Manual exposure |
| `exposure_time` | int | `1` | 1-31 | Exposure (0.1ms units) |
| `gain_enable` | bool | `false` | - | Manual gain |
| `gain_value` | int | `20` | 10-160 | Gain value |
| `laser_power` | float | `1.0` | 1-3 | 1=auto, 2=indoor, 3=outdoor |

## Launch Camera Driver

```bash
# Start camera driver in Docker
docker run --rm -it \
    --privileged \
    -v /dev:/dev \
    -v ~/deptrum_ws:/deptrum_ws \
    landerpi-ros2:latest \
    bash -c "source /opt/ros/humble/setup.bash && \
             source /deptrum_ws/install/local_setup.bash && \
             ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py"
```

Expected startup output:
```
device_count==========1
BOOT_ORDER_1::Found 1 deptrum device
Device information:
    sdk version: 1.1.19
    device name: Aurora 930
    serial number: HY400516001015B03G00151
    firmware version: 1.7.2
```

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Docker Container                      │
│  ┌─────────────────┐         ┌──────────────────────┐  │
│  │ Aurora930 Driver│ ───────→│ /aurora/rgb/image_raw│  │
│  │    (ROS2)       │ ───────→│ /aurora/depth/image_raw│ │
│  │                 │ ───────→│ /aurora/points2       │  │
│  └─────────────────┘         └──────────────────────┘  │
│          ↑                                             │
└──────────│─────────────────────────────────────────────┘
           │
      USB (3251:1930)
           │
           ↓
    ┌─────────────┐
    │ Aurora 930  │
    │ Depth Camera│
    └─────────────┘
```

## Coordinate Frames

```
camera_base (root)
├── rgb_camera_link (RGB camera optical frame)
└── depth_camera_link (depth/IR camera optical frame)
```

**Point Cloud Coordinates:**
- X: Right (+) / Left (-)
- Y: Down (+) / Up (-)
- Z: Forward (+)

## Validation Guide

### Quick Validation Steps

1. **Check USB detection:**
   ```bash
   lsusb | grep 3251
   # Expected: ID 3251:1930
   ```

2. **Check udev rules:**
   ```bash
   cat /etc/udev/rules.d/99-deptrum-libusb.rules
   ```

3. **List ROS2 topics:**
   ```bash
   ros2 topic list | grep aurora
   ```

4. **Check publishing rate:**
   ```bash
   ros2 topic hz /aurora/depth/image_raw
   ```

5. **Validate depth data:**
   ```bash
   ros2 topic echo /aurora/depth/image_raw --field header
   ```

### Expected Validation Results

| Stream | Encoding | Valid Width | Valid Height |
|--------|----------|-------------|--------------|
| RGB | bgr8 | 640/480/320 | 400/300/200 |
| IR | mono8 | 640/480/320 | 400/300/200 |
| Depth | mono16 | 640/480/320 | 400/300/200 |

**Depth Values:**
- Units: Millimeters
- Valid range: 150-4000mm
- Invalid: 0 value

## Python Validation Example

```python
#!/usr/bin/env python3
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

        self.depth_sub = self.create_subscription(
            Image, '/aurora/depth/image_raw', self.depth_callback, 10)

    def depth_callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, "mono16")
        valid = (depth > 150) & (depth < 4000)
        self.results['depth'] = {
            'encoding': msg.encoding,
            'resolution': f"{msg.width}x{msg.height}",
            'valid_percentage': f"{np.sum(valid)/depth.size*100:.1f}%",
        }
        self.get_logger().info(
            f"Depth: {msg.width}x{msg.height} valid={np.sum(valid)/depth.size*100:.1f}%"
        )
```

## Troubleshooting

### Problem: Camera not detected

**Symptoms:**
- `lsusb | grep 3251` returns nothing

**Solutions:**
1. Check USB cable connection
2. Try different USB port
3. Verify camera power (needs 1.5A @ 5V)
4. Check USB hub - try direct connection

### Problem: Driver fails to start

**Symptoms:**
- "No deptrum device connected!"

**Solutions:**
1. Verify udev rules:
   ```bash
   cat /etc/udev/rules.d/99-deptrum-libusb.rules
   ```
2. Reload rules:
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
3. Check USB permissions:
   ```bash
   ls -la /dev/bus/usb/*/
   ```

### Problem: No depth data

**Symptoms:**
- Driver starts but depth topic has no data
- All depth values are 0

**Solutions:**
1. Check camera is in range (30-300cm)
2. Adjust laser power: `laser_power=2` (indoor) or `3` (outdoor)
3. Check for reflective surfaces
4. Verify depth filtering params

### Problem: Noisy depth

**Symptoms:**
- Many holes in depth image
- Flickering depth values

**Solutions:**
1. Increase `threshold_size` (30-400)
2. Adjust `minimum_filter_depth_value` (150mm min)
3. Enable `depth_correction=true`
4. Reduce frame rate: `ir_fps=5`

### Problem: Frame timeout

**Symptoms:**
- "Get frames timeout!" warnings
- Intermittent data

**Solutions:**
1. Reduce frame rate: `ir_fps=5`
2. Disable unused streams: `point_cloud_enable=false`
3. Use lower resolution: `resolution_mode_index=2`
4. Check USB bandwidth

### Log Files

- **ROS2 Driver:** `/tmp/deptrum_ros_driver_*.INFO`
- **SDK Logs:** `/tmp/deptrum-stream.log`
- **Latest Log:** `/tmp/node.INFO`

## Reference Files

- Full documentation: `DepthCameraHowTo.md`
- ROS2 API reference: `Aurora930_ROS2_Interface_Analysis.md`
- Driver source: `drivers/depth-camera/deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz`
