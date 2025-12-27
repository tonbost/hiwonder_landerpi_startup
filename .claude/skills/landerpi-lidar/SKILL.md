---
name: landerpi-lidar
description: Lidar control for HiWonder LanderPi robot. Provides LD19/MS200 lidar protocol details, driver management, scan data reading, and autonomous modes (obstacle avoidance, tracking, guard). Requires landerpi-core skill; autonomous modes also require landerpi-motion skill.
---

# LanderPi Lidar Control

## Overview

Lidar control skill for HiWonder LanderPi with LD19/MS200 lidar. Provides driver management, scan data, and autonomous navigation modes.

**Prerequisites:**
- Load `landerpi-core` skill for connection and Docker setup
- Load `landerpi-motion` skill for autonomous modes (modes control motors)

## Lidar Commands

| Command | Purpose |
|---------|---------|
| `uv run python test_lidar.py check` | Check lidar + Docker + ROS2 |
| `uv run python test_lidar.py start-driver` | Start lidar driver in Docker |
| `uv run python test_lidar.py scan --samples 5` | Read lidar scan data |
| `uv run python test_lidar.py stop-driver` | Stop lidar driver container |
| `uv run python test_lidar.py test --mode 1 --duration 10 --yes` | Obstacle avoidance mode |
| `uv run python test_lidar.py test --mode 2 --duration 10 --yes` | Tracking mode |
| `uv run python test_lidar.py test --mode 3 --duration 10 --yes` | Guard mode |
| `uv run python test_lidar.py stop` | Stop all lidar functionality |

## Hardware Configuration

| Property | Value |
|----------|-------|
| Lidar Model | LD19 / MS200 |
| Device | `/dev/ttyUSB0` or `/dev/ttyUSB1` |
| Baud Rate | 230400 |
| ROS2 Topic | `/scan` |
| Docker Image | `landerpi-ros2:latest` |
| Scan Rate | ~10 Hz |
| Range | 0.05m - 12m |
| Angular Resolution | 1° (360 points per scan) |

**Note:** The symlink `/dev/ldlidar` usually doesn't exist. The driver automatically uses `/dev/ttyUSB0` or `/dev/ttyUSB1`.

## LD19/MS200 Protocol

The lidar uses a packet-based serial protocol:

### Packet Structure (47 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Header | `0x54` - packet start marker |
| 1 | 1 | Ver/Len | `0x2C` - 12-point packet |
| 2-3 | 2 | Speed | Rotation speed (little-endian) |
| 4-5 | 2 | Start Angle | In 0.01° units (little-endian) |
| 6-41 | 36 | Data | 12 measurements × 3 bytes each |
| 42-43 | 2 | End Angle | In 0.01° units (little-endian) |
| 44-45 | 2 | Timestamp | Milliseconds |
| 46 | 1 | CRC | Checksum |

### Measurement Format (3 bytes each)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0-1 | 2 | Distance | In mm (little-endian) |
| 2 | 1 | Intensity | Signal strength |

### Valid Range

- **Minimum:** 50mm (5cm)
- **Maximum:** 12000mm (12m)
- **Invalid:** 0 or values outside range → `inf`

### Driver Implementation

The custom driver:
1. Reads serial data at 230400 baud
2. Validates packets (header `0x54`, ver `0x2C`)
3. Accumulates 360° of data (multiple packets)
4. Publishes complete LaserScan at 10Hz

## Autonomous Modes

**WARNING: Robot WILL MOVE in these modes!**

All modes run entirely in Docker without requiring HiWonder ROS2 workspace.

### Mode 1: Obstacle Avoidance

Robot turns away from obstacles, moves forward when clear.

**Logic:**
- Scan 120° cone (60° each side)
- Compare minimum distance left vs right
- If obstacle on one side: turn away
- If obstacles both sides: reverse
- If clear (>0.6m): move forward

### Mode 2: Tracking

Robot follows the closest object at 35cm distance.

**Logic:**
- Find closest object in scan
- PID control to maintain 35cm distance
- Turn to face the object
- Move forward/backward to maintain distance

### Mode 3: Guard

Robot rotates to face detected objects.

**Logic:**
- Detect objects in scan
- Calculate angular offset to closest object
- Rotate to face it
- Stay stationary (no forward movement)

## Control Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Speed | 0.2 m/s | Forward movement speed |
| Threshold | 0.6m | Obstacle detection distance |
| Scan Angle | 120° | 60° each side of center |
| Tracking Distance | 0.35m | Target following distance |
| PID Kp | 1.6 | Proportional gain |
| PID Ki | 0.0 | Integral gain |
| PID Kd | 0.16 | Derivative gain |

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Docker Container                      │
│  ┌─────────────┐     /scan      ┌──────────────────┐   │
│  │ LD19 Driver │ ──────────────→│ Mode Controller  │   │
│  │  (ROS2)     │                │    (ROS2)        │   │
│  └─────────────┘                └────────┬─────────┘   │
│        ↑                                 │             │
└────────│─────────────────────────────────│─────────────┘
         │                                 │
    /dev/ttyUSB0                     Motor Commands
         │                                 │
         ↓                                 ↓
    ┌─────────┐                    ┌───────────────┐
    │  Lidar  │                    │ Motor SDK     │
    │  LD19   │                    │ /dev/ttyACM0  │
    └─────────┘                    └───────────────┘
```

## Troubleshooting

### Problem: Lidar driver crashes immediately

**Symptoms:**
- "Driver may have crashed" error
- Container exits immediately

**Diagnosis:**
```bash
docker run --rm landerpi-ros2:latest bash -c 'pip3 install pyserial && python3 -c "import serial"'
```

**Cause:** Using wrong Docker image (base image lacks pip3)

**Solution:** Ensure `landerpi-ros2:latest` is used (has pip3/pyserial)

### Problem: Lidar symlink not found

**Symptoms:**
- `/dev/ldlidar` doesn't exist
- Message: "Lidar symlink not found, but USB serial devices exist"

**Solution:** This is normal. The driver uses `/dev/ttyUSB0` or `/dev/ttyUSB1` automatically.

### Problem: No scan data

**Symptoms:**
- Driver starts but no data on `/scan` topic
- Mode controller shows no distances

**Diagnosis:**
```bash
# Check raw serial data
docker run --rm --privileged -v /dev:/dev landerpi-ros2:latest \
    python3 -c "import serial; s=serial.Serial('/dev/ttyUSB0', 230400); print(s.read(100).hex())"
```

**Expected:** Data starting with `542c` (header 0x54, ver 0x2C)

**Solutions:**
1. Check lidar USB connection
2. Try different USB port (`/dev/ttyUSB1`)
3. Verify lidar is powered (spinning)

### Problem: Mode controller hangs

**Symptoms:**
- Test runs but never completes
- "Command did not complete within X seconds"

**Solution:** This was fixed - the driver now uses `sys.exit(0)` for clean shutdown. Update `test_lidar.py` if using old version.

### Problem: Robot doesn't move in autonomous mode

**Symptoms:**
- Lidar data shows correctly
- No motor movement

**Diagnosis:**
1. Check motor SDK is deployed: `ls ~/ros_robot_controller/`
2. Check serial port: `ls /dev/ttyACM*`
3. Check battery: `uv run python test_chassis_direct.py status`

**Solution:** Ensure motor controller SDK is installed and battery is charged.
