---
name: landerpi-lidar
description: Lidar control for HiWonder LanderPi robot. Provides LD19/MS200 lidar protocol details, driver management, scan data reading, autonomous modes (obstacle avoidance, tracking, guard), and autonomous exploration. Requires landerpi-core skill; autonomous modes and exploration also require landerpi-motion skill.
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
| `uv run python validation/test_lidar.py check` | Check lidar + Docker + ROS2 |
| `uv run python validation/test_lidar.py start-driver` | Start lidar driver in Docker |
| `uv run python validation/test_lidar.py scan --samples 5` | Read lidar scan data |
| `uv run python validation/test_lidar.py stop-driver` | Stop lidar driver container |
| `uv run python validation/test_lidar.py test --mode 1 --duration 10 --yes` | Obstacle avoidance mode |
| `uv run python validation/test_lidar.py test --mode 2 --duration 10 --yes` | Tracking mode |
| `uv run python validation/test_lidar.py test --mode 3 --duration 10 --yes` | Guard mode |
| `uv run python validation/test_lidar.py stop` | Stop all lidar functionality |

### ROS2 Stack Testing

For ROS2-based testing (requires deployed stack):
```bash
uv run python validation/test_lidar_ros2.py check       # Check lidar status
uv run python validation/test_lidar_ros2.py scan --samples 5  # Read scan data
```

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

**Solution:** This was fixed - the driver now uses `sys.exit(0)` for clean shutdown. Update `validation/test_lidar.py` if using old version.

### Problem: Robot doesn't move in autonomous mode

**Symptoms:**
- Lidar data shows correctly
- No motor movement

**Diagnosis:**
1. Check motor SDK is deployed: `ls ~/ros_robot_controller/`
2. Check serial port: `ls /dev/ttyACM*`
3. Check battery: `uv run python validation/test_chassis_direct.py status`

**Solution:** Ensure motor controller SDK is installed and battery is charged.

## Autonomous Exploration

Frontier-based autonomous exploration using sensor fusion (lidar + depth camera). Robot autonomously explores while avoiding obstacles. Includes progressive escape system to handle stuck situations.

**Two Approaches:**
1. **On-Robot (Recommended)** - Exploration runs on the robot for fast 8Hz sensor reads
2. **Remote (Legacy)** - Exploration runs on Mac via SSH (~0.1Hz due to SSH overhead)

### On-Robot Exploration (Recommended)

Exploration code runs directly on the Raspberry Pi with local `docker exec` for fast sensor access.

**Prerequisites:**
- ROS2 stack deployed (`deploy_ros2_stack.py deploy`)
- config.json configured with robot credentials

**Architecture:**
The explorer uses a ROS2 hardware abstraction layer (`ros2_hardware.py`) that:
- Wraps all `docker exec landerpi-ros2` calls
- Provides throttled sensor reading (lidar: 0.5s, depth: 1.0s) with caching
- Returns cached data between reads to avoid blocking the control loop

**Commands:**

| Command | Purpose |
|---------|---------|
| `uv run python deploy_explorer.py deploy` | Upload explorer + exploration module to robot |
| `uv run python deploy_explorer.py start --duration 10` | Start exploration (streams output) |
| `uv run python deploy_explorer.py start --rosbag --duration 10` | With ROS2 bag recording |
| `uv run python deploy_explorer.py stop` | Emergency stop |
| `uv run python deploy_explorer.py status` | Check robot status |
| `uv run python deploy_explorer.py logs` | View exploration logs |
| `uv run python deploy_explorer.py logs -f` | Follow logs in real-time |

**Options:**

| Option | Default | Description |
|--------|---------|-------------|
| `--duration` | 5 | Max runtime in minutes |
| `--speed` | 0.35 | Forward speed (m/s) |
| `--turn-multiplier` | 2.5 | Carpet friction compensation (see below) |
| `--rosbag` | Off | Enable ROS2 bag recording |

**Carpet Friction & Command Overhead Compensation:**

Mecanum wheels slip on carpet, and docker exec commands have latency. Two parameters compensate:

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `turn_time_multiplier` | 2.5 | Carpet friction (in config) |
| `turn_overhead_seconds` | 0.5 | Docker exec latency (in config) |

```bash
# Default settings (good for carpet)
uv run python deploy_explorer.py start --duration 5

# High-pile carpet (more friction)
uv run python deploy_explorer.py start --duration 5 --turn-multiplier 3.0

# Hard floor (no friction compensation needed)
uv run python deploy_explorer.py start --duration 5 --turn-multiplier 1.0
```

See `landerpi-motion` skill for detailed explanation.

### Remote Exploration (Legacy)

Exploration runs on Mac with SSH sensor reads. Slower due to network overhead (~0.1Hz vs 8Hz).

**Prerequisites:**
- Load `landerpi-core`, `landerpi-motion`, `landerpi-camera` skills
- ROS2 stack deployed (`deploy_ros2_stack.py deploy`)

**Commands:**

| Command | Purpose |
|---------|---------|
| `uv run python validation/test_exploration.py status` | Check prerequisites |
| `uv run python validation/test_exploration.py start --yes` | Start exploration (30 min) |
| `uv run python validation/test_exploration.py start --duration 5 --yes` | Custom duration |
| `uv run python validation/test_exploration.py start --rosbag --yes` | With ROS2 bag recording |
| `uv run python validation/test_exploration.py stop` | Emergency stop |

### How It Works

1. **Sensor Reading** - Reads lidar `/scan` and depth camera `/aurora/depth/image_raw`
2. **Sector Analysis** - Divides 360° into 8 sectors, tracks "freshness" (time since visited)
3. **Frontier Selection** - Chooses sector with lowest freshness that is not blocked
4. **Progressive Escape** - Detects oscillation/stuck patterns, escalates through 3 escape levels
5. **Obstacle Avoidance** - Stop at 15cm, slow at 30cm, sector blocked at 40cm
6. **Safety Monitoring** - Battery monitoring, runtime limits, emergency stop

### Exploration Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Forward Speed | 0.35 m/s | Movement speed when clear |
| Turn Speed | 1.5 rad/s | Rotation speed |
| Stop Distance | 0.15 m | Emergency stop threshold |
| Slow Distance | 0.30 m | Speed reduction threshold |
| Blocked Distance | 0.40 m | Sector marked blocked |
| Loop Rate | 10 Hz | Control loop frequency |
| Decision Rate | 2 Hz | Navigation decision frequency |
| Lidar Read Interval | 0.5 s | Throttled to avoid blocking |
| Depth Read Interval | 1.0 s | Throttled to avoid blocking |
| Stuck Timeout | 20 s | Time without forward movement |

**Navigation Logic:**
- ALWAYS prefer Front sector if open (go straight unless blocked)
- Only turn if Front is blocked
- When turning, prefer directions with more open space

### Progressive Escape System

Detects when robot is stuck (oscillating or repeatedly blocked) and escalates through escape levels:

| Level | Trigger | Action | Duration (mult=2.5) |
|-------|---------|--------|---------------------|
| 1 - Wide Turn | Oscillation OR blocked 3x | 90° turn toward open side | ~5.7s |
| 2 - Reverse 180 | Level 1 failed | Back up longer + turn 180° | ~11s |
| 3 - Full Scan | Level 2 failed | 360° rotation with lidar sampling | ~22s |
| Trapped | Level 3 failed | Stop and announce "I'm trapped" | - |

**Oscillation Detection:**
- Tracks turn history: `[(direction, timestamp), ...]` for last 20 seconds
- Triggers when 3+ alternating turns detected (left-right-left or right-left-right)
- Proactively escapes even before "stuck timeout" if oscillation detected

**Key Implementation Details (robot_explorer.py):**
```python
# State tracking in ExplorationState
turn_history: List[tuple]  # [(direction, timestamp), ...]
escape_level: int          # 0=normal, 1=wide, 2=reverse, 3=scan
last_escape_time: float
escape_cooldown: float = 15.0  # seconds before resetting level

# Oscillation detection
def _is_oscillating(self) -> bool:
    recent = [d for d, _ in self.turn_history[-6:]]
    alternations = sum(1 for i in range(1, len(recent)) if recent[i] != recent[i - 1])
    return alternations >= 3

# Progressive escape
def _escape_stuck(self) -> None:
    if self._should_escalate_escape():
        self.state.escape_level = min(self.state.escape_level + 1, 3)
    # Execute level-appropriate escape
```

**Why 90° instead of 45° for escape?**
In corners, both Front-Left (~0.35m) and Front-Right (~0.35m) may have similar clearance. Small 45° turns just oscillate between them. A 90° turn actually changes the robot's facing direction enough to find a clear path.

**Escape Level Cooldown:**
After a successful escape (robot moves forward), the escape level resets to 0 and turn history clears. A 15-second cooldown prevents rapid re-escalation.

### On-Robot Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Raspberry Pi                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              robot_explorer.py                        │  │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐     │  │
│  │  │ Lidar Read │  │  Frontier  │  │  Escape    │     │  │
│  │  │docker exec │  │  Planner   │  │  Handler   │     │  │
│  │  └────────────┘  └────────────┘  └────────────┘     │  │
│  └──────────────────────────────────────────────────────┘  │
│                           │                                  │
│  ┌────────────────────────▼────────────────────────────┐   │
│  │               Docker: landerpi-ros2                   │   │
│  │   /scan topic    /aurora/depth/image_raw topic       │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │   Mac (local)          │
              │   deploy_explorer.py   │
              │   (control + logs)     │
              └───────────────────────┘
```

### Module Files

**On-Robot:**
- `robot_explorer.py` - Self-contained exploration script for robot
- `deploy_explorer.py` - Deployment and control from Mac

**Remote (Legacy):**
- `validation/test_exploration.py` - CLI entry point
- `validation/exploration/explorer.py` - Main controller
- `validation/exploration/sensor_fusion.py` - Lidar + depth camera fusion
- `validation/exploration/frontier_planner.py` - Frontier detection and goal selection
- `validation/exploration/escape_handler.py` - Progressive escape logic
- `validation/exploration/arm_scanner.py` - Arm-mounted depth camera scanning
- `validation/exploration/safety_monitor.py` - Battery and runtime monitoring
- `validation/exploration/data_logger.py` - Local logging

### Exploration Troubleshooting

#### Problem: Turn Under-Rotation on Carpet

**Symptoms:**
- Robot commanded to turn 90° only turns ~30-60°
- Escape maneuvers fail to clear obstacles
- Robot keeps getting stuck in same spot

**Root Causes:**
1. Mecanum wheels slip on carpet (30-40% of expected rotation)
2. Docker exec command latency reduces effective turn time

**Solution:** Two parameters in `ExplorationConfig` (robot_explorer.py):

| Parameter | Default | Adjust if... |
|-----------|---------|--------------|
| `turn_time_multiplier` | 2.5 | Turns consistently short → increase to 3.0 |
| `turn_overhead_seconds` | 0.5 | Turns slightly short → increase to 0.8 |

```bash
# CLI only controls multiplier; overhead is in config
uv run python deploy_explorer.py start --duration 5 --turn-multiplier 3.0
```

**Tuning:**
1. Start with defaults (mult=2.5, overhead=0.5)
2. Watch turn output: `Turning 90° to Front-Left (3.1s, mult=2.5, overhead=0.5s)`
3. If under-rotating: first try multiplier 3.0, then increase overhead to 0.8
4. If over-rotating: decrease multiplier to 2.0

#### Problem: Robot Oscillates Without Progress

**Symptoms:**
- Robot keeps turning left-right-left-right
- Never moves forward
- Eventually triggers escape

**Solution:** This is handled by the turn commitment system. If still occurring:
1. The turn_complete_threshold (25°) may need adjustment
2. Check if lidar is returning stable data
3. Verify min_turn_duration is being respected (log shows turn times)
