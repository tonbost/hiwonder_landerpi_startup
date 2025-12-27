# Aurora 930 Depth Camera Driver

This directory contains the Deptrum ROS2 driver for the Aurora 930 depth camera.

## Contents

| File | Description |
|------|-------------|
| `deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz` | ROS2 Humble driver for ARM64 (Raspberry Pi 5) |

## Driver Details

- **SDK Version**: 1.1.19
- **Target Platform**: ARM64 (aarch64)
- **ROS2 Distribution**: Humble
- **Camera Model**: Aurora 930 (VID:PID 3251:1930)

## Installation

See [DepthCameraHowTo.md](../../DepthCameraHowTo.md) for complete setup instructions.

### Quick Transfer to Pi

```bash
# From this repo directory
scp drivers/depth-camera/deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz user@<PI_IP>:~/
```

## Source

Original location: HiWonder LanderPi documentation
- Path: `1. Tutorials/5. Depth Camera Basic Course/4 Appendix/2 Source Code/ROS2/`
