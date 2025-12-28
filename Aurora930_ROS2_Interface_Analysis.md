# Aurora 930 Depth Camera ROS2 Driver Interface Analysis

**Source**: deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz
**Driver Version**: 0.2.1001
**SDK Version**: Aurora900 v1.1.19
**ROS2 Support**: Foxy, Galactic, Humble
**Camera Model**: Aurora 930 (3D depth camera)

---

## Package Information

**Package Name**: `deptrum-ros-driver-aurora930`
**Node Executable**: `aurora930_node`
**Default Namespace**: `aurora` (configurable)

**Dependencies**:
- `rclcpp` - ROS2 C++ client library
- `std_msgs` - Standard ROS2 messages
- `sensor_msgs` - Sensor data messages (Image, PointCloud2, CameraInfo)
- `geometry_msgs` - Geometry messages
- `cv_bridge` - OpenCV-ROS bridge
- `tf2`, `tf2_ros`, `tf2_geometry_msgs` - Transform library
- `angles` - Angle utilities
- `libopencv-dev` - OpenCV library

---

## ROS2 Topics Published

### Image Topics

| Topic | Message Type | Description | Format | Enabled By |
|-------|-------------|-------------|--------|-----------|
| `rgb/image_raw` | `sensor_msgs/msg/Image` | Raw RGB color image | BGR8 (640x480 or other resolutions) | `rgb_enable=true` |
| `ir/image_raw` | `sensor_msgs/msg/Image` | Raw infrared image | MONO8 (8-bit grayscale) | `ir_enable=true` |
| `depth/image_raw` | `sensor_msgs/msg/Image` | Raw depth image | MONO16 (16-bit, millimeters) | `depth_enable=true` |

### Point Cloud Topics

| Topic | Message Type | Description | Format | Enabled By |
|-------|-------------|-------------|--------|-----------|
| `points2` | `sensor_msgs/msg/PointCloud2` | 3D point cloud from depth | XYZ (meters) or XYZRGB (colored) | `point_cloud_enable=true` |

### Camera Info Topics

| Topic | Message Type | Description | Enabled By |
|-------|-------------|-------------|-----------|
| `rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB camera calibration parameters | `rgb_enable=true` |
| `ir/camera_info` | `sensor_msgs/msg/CameraInfo` | IR/depth camera calibration parameters | `ir_enable=true` or `depth_enable=true` or `point_cloud_enable=true` |

### Frame IDs

- **RGB camera frame**: `rgb_camera_link` (default)
- **Depth/IR camera frame**: `depth_camera_link` (default)
- **Base link**: `camera_base` (default)

---

## ROS2 Topics Subscribed

**None** - This driver is a pure publisher node (no subscriptions)

---

## ROS2 Services

**None** - No services are exposed by this driver

---

## ROS2 Parameters

### Core Enable/Disable Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rgb_enable` | bool | `true` | Enable RGB color image stream |
| `ir_enable` | bool | `true` | Enable infrared (IR) image stream |
| `depth_enable` | bool | `true` | Enable depth image stream |
| `point_cloud_enable` | bool | `true` | Enable 3D point cloud generation |
| `rgbd_enable` | bool | `false` | Enable RGBD mode (all streams from single frame) |

### Camera Configuration

| Parameter | Type | Default | Valid Range | Description |
|-----------|------|---------|-------------|-------------|
| `ir_fps` | int | `15` | 5, 10, 12, 15 | IR camera frame rate (fps) |
| `rgb_fps` | int | `15` | 5-30 | RGB camera frame rate (must be ≤ ir_fps) |
| `resolution_mode_index` | int | `2` | 0-n | Resolution mode index (see supported modes below) |
| `usb_port_number` | string | `""` | USB port path | Select camera by USB port (for multi-camera) |
| `serial_number` | string | `""` | Serial number | Select camera by serial number |

### Exposure & Gain

| Parameter | Type | Default | Valid Range | Description |
|-----------|------|---------|-------------|-------------|
| `exposure_enable` | bool | `false` | - | Enable manual exposure control (false = auto exposure) |
| `exposure_time` | int | `1` | 1-31 | Manual exposure time in 0.1ms units (e.g., 10 = 1ms) |
| `gain_enable` | bool | `false` | - | Enable manual gain control |
| `gain_value` | int | `20` | 10-160 | Manual gain value |

### Depth Processing

| Parameter | Type | Default | Valid Range | Description |
|-----------|------|---------|-------------|-------------|
| `depth_correction` | bool | `true` | - | Enable depth correction (requires align_mode=true) |
| `align_mode` | bool | `true` | - | Align depth to RGB coordinates |
| `threshold_size` | int | `110` | 30-400 | Noise removal filter size (smaller = more aggressive) |
| `minimum_filter_depth_value` | int | `150` | 150-300 | Minimum valid depth in mm (closer points filtered out) |
| `maximum_filter_depth_value` | int | `4000` | - | Maximum valid depth in mm (farther points filtered out) |

### Laser & Power

| Parameter | Type | Default | Valid Range | Description |
|-----------|------|---------|-------------|-------------|
| `laser_power` | float | `1.0` | 1.0-3.0 | Laser driver power: 1=auto, 2=indoor, 3=outdoor |

### System & Logging

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `boot_order` | int | `1` | Device boot order (for multi-camera setups) |
| `log_dir` | string | `"/tmp/"` | Directory for log file storage |
| `stream_sdk_log_enable` | bool | `true` | Enable Deptrum SDK internal logging |
| `heart_enable` | bool | `false` | Enable heartbeat to auto-reboot on timeout |
| `update_file_path` | string | `""` | Path to firmware update file (triggers update) |

### Advanced Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `flag_enable` | bool | `true` | Enable flag data |
| `outlier_point_removal_flag` | bool | `false` | Enable outlier point removal |
| `shuffle_enable` | bool | `false` | Enable shuffle mode |
| `undistortion_enable` | bool | `false` | Enable lens undistortion |
| `device_numbers` | int | `1` | Number of devices to manage |
| `heart_timeout_times` | int | `8` | Heartbeat timeout count |
| `filter_type` | int | `1` | Filter type selection |
| `ir_frequency_fusion_threshold` | int | `16` | IR frequency fusion threshold (0-4095) |
| `depth_frequency_fusion_threshold` | float | `300.0` | Depth frequency fusion threshold (0-7500) |
| `ratio_scatter_filter_threshold` | float | `0.04` | Scatter filter ratio threshold |
| `slam_mode` | int | `0` | SLAM mode configuration |
| `stof_minimum_range` | int | `10` | STOF minimum range |
| `mtof_filter_level` | int | `1` | MTOF filter level (0=low, 1=mid, 2=high) |
| `stof_filter_level` | int | `1` | STOF filter level (0=low, 1=mid, 2=high) |
| `mtof_crop_up` | int | `50` | MTOF crop top pixels |
| `mtof_crop_down` | int | `80` | MTOF crop bottom pixels |
| `ext_rgb_mode` | int | `0` | External RGB mode (0=disable, 1=1920x1080, 2=1280x720) |
| `enable_imu` | bool | `true` | Enable IMU data (if available) |

---

## Resolution Modes

The `resolution_mode_index` parameter selects from supported frame mode combinations. Common modes for Aurora 930:

| Index | IR Resolution | RGB Resolution | Depth Resolution |
|-------|--------------|----------------|------------------|
| 0 | 640x400 @ 8-bit | 640x400 YUV | 640x400 @ 16-bit |
| 1 | 480x300 @ 8-bit | 480x300 YUV | 480x300 @ 16-bit |
| 2 | 320x200 @ 8-bit | 320x200 YUV | 320x200 @ 16-bit |

**Note**: Exact available modes depend on camera firmware version. The driver logs all supported modes at startup.

---

## Message Type Details

### Image Messages

**RGB Image** (`rgb/image_raw`):
- **Encoding**: `bgr8` (8-bit per channel, Blue-Green-Red order)
- **Resolution**: Depends on `resolution_mode_index`
- **Format**: Converted from YUV (NV12) to BGR

**IR Image** (`ir/image_raw`):
- **Encoding**: `mono8` (8-bit grayscale)
- **Resolution**: Same as depth image
- **Format**: Raw infrared intensity

**Depth Image** (`depth/image_raw`):
- **Encoding**: `mono16` (16-bit unsigned integer)
- **Resolution**: Depends on `resolution_mode_index`
- **Units**: Millimeters (mm)
- **Range**: 150mm to 4000mm (configurable with min/max filter)

### Point Cloud Message

**Point Cloud** (`points2`):
- **Type**: `sensor_msgs/msg/PointCloud2`
- **Fields**:
  - `xyz` (float32, meters) - Basic point cloud
  - `xyzrgb` (float32 + uint8[3]) - Colored point cloud (if `rgbd_enable=true`)
- **Coordinate System**:
  - X: Right (+) / Left (-)
  - Y: Down (+) / Up (-)
  - Z: Forward (+) / Backward (-)
- **Conversion**: Points are converted from mm to meters (divided by 1000)
- **Filtering**: Points with `z ≤ 0` are excluded

### Camera Info Messages

**Camera Info** (`rgb/camera_info`, `ir/camera_info`):
- Contains intrinsic calibration parameters (K matrix, D coefficients)
- Distortion model and projection matrix
- Retrieved from Deptrum SDK's camera parameter storage

---

## Launch Files

### Single Camera Launch

**File**: `aurora930_launch.py`

```bash
ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py
```

**Default namespace**: `aurora`

**Published topics**:
- `/aurora/rgb/image_raw`
- `/aurora/ir/image_raw`
- `/aurora/depth/image_raw`
- `/aurora/points2`
- `/aurora/rgb/camera_info`
- `/aurora/ir/camera_info`

### Multi-Camera Launch

**File**: `aurora930_multi_launch.py`

```bash
ros2 launch deptrum-ros-driver-aurora930 aurora930_multi_launch.py
```

Launches two camera nodes with namespaces:
- `aurora_1` (boot_order=1)
- `aurora_2` (boot_order=2)

**Note**: For multi-camera setups, set `ROS_DOMAIN_ID` environment variable to avoid network conflicts.

### Visualization Launch

**File**: `viewer930_launch.py`

```bash
ros2 launch deptrum-ros-driver-aurora930 viewer930_launch.py
```

Launches camera node + RViz2 with pre-configured visualization settings.

---

## Device Selection (Multi-Camera)

When multiple Aurora cameras are connected, select specific camera using:

### Option 1: USB Port Number
```python
parameters=[{"usb_port_number": "1-1.1"}]
```

### Option 2: Serial Number
```python
parameters=[{"serial_number": "ABC123456"}]
```

### Option 3: Boot Order
```python
parameters=[{"boot_order": 1}]  # First device found
```

---

## Coordinate Frames

The driver uses standard ROS coordinate frame conventions:

```
camera_base (root)
├── rgb_camera_link (RGB camera optical frame)
└── depth_camera_link (depth/IR camera optical frame)
```

**Transform broadcasting**: The driver includes `tf2_ros` support, but specific transforms depend on the camera's physical configuration.

---

## Example Usage

### Basic ROS2 Commands

```bash
# List all topics
ros2 topic list

# View RGB image
ros2 topic echo /aurora/rgb/image_raw

# View point cloud stats
ros2 topic echo /aurora/points2 --once

# Check publishing rate
ros2 topic hz /aurora/depth/image_raw

# Get camera info
ros2 topic echo /aurora/rgb/camera_info --once
```

### Python Subscriber Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

class AuroraSubscriber(Node):
    def __init__(self):
        super().__init__('aurora_subscriber')
        self.bridge = CvBridge()

        self.rgb_sub = self.create_subscription(
            Image, '/aurora/rgb/image_raw',
            self.rgb_callback, 10)

        self.depth_sub = self.create_subscription(
            Image, '/aurora/depth/image_raw',
            self.depth_callback, 10)

        self.cloud_sub = self.create_subscription(
            PointCloud2, '/aurora/points2',
            self.cloud_callback, 10)

    def rgb_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Process RGB image (640x400 or other resolution)

    def depth_callback(self, msg):
        cv_depth = self.bridge.imgmsg_to_cv2(msg, "mono16")
        # Process depth image (16-bit, millimeters)

    def cloud_callback(self, msg):
        # Process point cloud
        # Extract XYZ points (in meters)
        pass

def main():
    rclpy.init()
    node = AuroraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## Installation

### From Debian Package

```bash
# Install
sudo dpkg -i ros-humble-deptrum-ros-driver-aurora930_0.2.1001-0jammy_arm64.deb

# Source ROS2
source /opt/ros/humble/setup.bash

# Launch
ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py
```

### From Source (in Docker)

```bash
# Clone/extract source to workspace
cd ~/deptrum_ws/src
tar -xzf deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz

# Build with specific SDK type
cd ~/deptrum_ws
colcon build --cmake-args -DSTREAM_SDK_TYPE=AURORA930

# Source workspace
source ~/deptrum_ws/install/setup.bash

# Launch
ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py
```

---

## Udev Rules

The driver requires USB permissions for the Deptrum camera:

**File**: `/etc/udev/rules.d/99-deptrum-libusb.rules`

```bash
# Install udev rules
sudo cp /usr/share/deptrum-ros-driver-aurora930/99-deptrum-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**VID**: 0x3251 (Deptrum/Stellar)

---

## Logging

### ROS2 Driver Logs

**Location**: `/tmp/deptrum_ros_driver_*.INFO`

**Soft link**: `/tmp/node.INFO` (latest log)

**Content**:
- Device connection/disconnection events
- Frame capture statistics
- Parameter settings
- Error messages

### Deptrum SDK Logs

**Location**: `/tmp/deptrum-stream.log` (if `stream_sdk_log_enable=true`)

**Content**:
- Low-level SDK operations
- USB communication
- Firmware interactions

---

## Troubleshooting

### No Device Found

**Symptoms**: "No deptrum device connected! It's going to quit..."

**Solutions**:
1. Check USB connection: `lsusb | grep 3251`
2. Verify udev rules are installed
3. Check user is in `dialout` group: `groups $USER`
4. Try `sudo` to rule out permissions

### Frame Timeout

**Symptoms**: "Get frames timeout!" warnings

**Solutions**:
1. Reduce frame rate: `ir_fps=5`
2. Disable unused streams: `point_cloud_enable=false`
3. Check USB bandwidth (USB 3.0 recommended)

### Incorrect Colors

**Symptoms**: Colors look wrong in RGB image

**Solutions**:
1. The driver publishes BGR format (OpenCV convention)
2. For RViz2, ensure color scheme is set correctly
3. For OpenCV: image is already in BGR format

### Depth Artifacts

**Symptoms**: Noisy or incorrect depth values

**Solutions**:
1. Adjust `threshold_size` (increase = more filtering)
2. Set appropriate depth range with `minimum_filter_depth_value` and `maximum_filter_depth_value`
3. Adjust `laser_power` based on environment (2=indoor, 3=outdoor)
4. Enable `depth_correction=true` if using aligned mode

---

## Performance Optimization

### Reduce Latency

1. Increase frame rate: `ir_fps=15`
2. Disable point cloud if not needed: `point_cloud_enable=false`
3. Use lower resolution: `resolution_mode_index=2`

### Reduce Bandwidth

1. Lower frame rate: `ir_fps=5`
2. Disable unused streams: `rgb_enable=false`
3. Use smaller resolution: `resolution_mode_index=2`

### Multi-Camera Setup

1. Set unique namespaces: `aurora_1`, `aurora_2`
2. Use USB port selection: `usb_port_number`
3. Set `ROS_DOMAIN_ID` to avoid network interference
4. Use separate USB controllers if possible

---

## Summary

The Aurora 930 ROS2 driver provides a comprehensive interface for accessing depth camera data with:

- **4 image topics** (RGB, IR, depth, point cloud)
- **2 camera info topics** (RGB, IR)
- **50+ configurable parameters**
- **No services** (pure publisher node)
- **No subscriptions** (autonomous operation)
- **Multi-camera support** via namespaces and device selection
- **Standard ROS2 message types** (sensor_msgs)

The driver is production-ready with extensive configurability for exposure control, depth filtering, alignment, and performance tuning. It follows ROS2 best practices with proper coordinate frames, camera calibration, and message stamping.
