# Aurora 930 Depth Camera - Status Report

**Date:** 2025-12-26 (Updated: 2025-12-27)
**Device:** HiWonder LanderPi Robot (Raspberry Pi 5)
**Camera:** Aurora 930 Depth Camera
**Status:** WORKING with Deptrum ROS2 Driver

## Hardware Detection

| Check | Result |
|-------|--------|
| **USB Detection** | Detected |
| **Vendor ID** | 3251 |
| **Product ID** | 1930 |
| **Serial Number** | HY400516001015B03G00151 |
| **USB Speed** | High Speed (480Mbps) |
| **USB Class** | Vendor Specific (255) |
| **V4L2 Device** | Not created (uses proprietary SDK) |
| **ROS2 Driver** | WORKING (Deptrum SDK v1.1.19) |

### lsusb Output
```
Bus 002 Device 003: ID 3251:1930 Linux Foundation Aurora 930
```

### USB Descriptor Details
```
idVendor           0x3251 Linux Foundation
idProduct          0x1930 Aurora 930
bcdDevice            4.19
iManufacturer           1 Linux Foundation
iProduct                2 Aurora 930
iSerial                 3 HY400516001015B03G00151
bDeviceClass          239 Miscellaneous Device
bInterfaceClass       255 Vendor Specific Class
bInterfaceSubClass    255 Vendor Specific Subclass
```

## Software Status

### Orbbec SDK (pyorbbecsdk v2.0.15)

| Component | Status |
|-----------|--------|
| **Installation** | Installed |
| **Version** | 2.0.15 (ARM64) |
| **Device Detection** | NOT DETECTED |

#### SDK Debug Log
```
[debug] DeviceManager init ...
[debug] Enable USB Device Enumerator ...
[debug] queryUsbInterfaces done!
[debug] No matched usb device found!    <-- Aurora 930 not recognized
[info] Current found device(s): (0)
```

### Root Cause
The Aurora 930 uses **Vendor Specific USB Class (255)** instead of standard UVC (USB Video Class). This means:

1. **Not a standard webcam** - Won't work with V4L2/OpenCV directly
2. **Requires proprietary SDK** - Needs manufacturer's driver/library
3. **Not in Orbbec SDK database** - Despite using Orbbec-style VID (3251), this camera is not recognized by the standard Orbbec SDK v2

## Configuration Applied

### udev Rules
File: `/etc/udev/rules.d/99-orbbec.rules`
```
# Orbbec USB Cameras
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666", GROUP="video"
SUBSYSTEM=="usb", ATTR{idVendor}=="3251", MODE="0666", GROUP="video"
# Aurora 930
SUBSYSTEM=="usb", ATTR{idVendor}=="3251", ATTR{idProduct}=="1930", MODE="0666", GROUP="video"
```

### USB Permissions
```
/dev/bus/usb/002/003: crw-rw-rw- 1 root video 189, 130
```
Permissions are correct (0666, video group).

### Installed Packages
- `libusb-1.0-0-dev`
- `libudev-dev`
- `v4l-utils`
- `pyorbbecsdk` 2.0.15 (from GitHub release, ARM64 wheel)

## Working Solution: Deptrum ROS2 Driver

The camera now works with the official Deptrum ROS2 driver from HiWonder's documentation.

### Driver Details
| Component | Value |
|-----------|-------|
| **Driver Package** | `deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz` |
| **SDK Version** | 1.1.19 |
| **Firmware Version** | 1.7.2 |
| **Resolution** | 640×400 (IR & RGB) |
| **ROS2 Distribution** | Humble (via Docker) |

### Verified ROS2 Topics
The driver publishes depth, RGB, and IR image streams.

### Launch Command
```bash
docker run --rm -it --privileged -v /dev:/dev -v ~/deptrum_ws:/deptrum_ws landerpi-ros2:latest \
    bash -c "source /opt/ros/humble/setup.bash && source /deptrum_ws/install/local_setup.bash && \
             ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py"
```

### Why Orbbec SDK Didn't Work
The Aurora 930 uses **Deptrum's proprietary SDK**, not the standard Orbbec SDK. Despite having Vendor ID 3251 (associated with Orbbec), this camera requires Deptrum's specific driver.

## Test Commands

### Verify USB Detection
```bash
lsusb -d 3251:1930
lsusb -v -d 3251:1930 | head -30
```

### Check V4L2 Devices
```bash
v4l2-ctl --list-devices
ls -la /dev/video*
```

### Test Orbbec SDK
```python
from pyorbbecsdk import *
ctx = Context()
device_list = ctx.query_devices()
print(f'Devices found: {device_list.get_count()}')
```

### Check Kernel Messages
```bash
sudo dmesg | grep -i "3251\|aurora\|uvc"
```

## References

- [Orbbec SDK GitHub](https://github.com/orbbec/pyorbbecsdk)
- [HiWonder LanderPi Documentation](https://www.hiwonder.com/)
- [OpenNI2 Documentation](https://structure.io/openni)

## Setup Instructions

For complete setup instructions, see [DepthCameraHowTo.md](DepthCameraHowTo.md).

### Quick Summary
1. Transfer `deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz` to Pi
2. Extract to `~/deptrum_ws/src/`
3. Fix package.xml name: `deptrum-ros-driver` → `deptrum-ros-driver-aurora930`
4. Setup udev rules for Deptrum cameras (VID 3251)
5. Build in Docker with `-DSTREAM_SDK_TYPE=AURORA930`
6. Launch with Docker (requires `--privileged` and USB access)
