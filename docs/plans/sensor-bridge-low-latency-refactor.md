# Plan: Persistent ROS2 Subscriber Architecture for Low-Latency Topic Reading

**Date:** 2026-01-03
**Status:** Approved for Implementation

## Problem Statement

Current architecture uses subprocess calls to `docker exec ros2 topic echo` for reading sensor data:
- Each read takes **400-500ms** due to ROS2 node initialization overhead
- Caching mitigates but introduces **0.5-1.0s latency** for sensor data
- Control loop at 10Hz (100ms period) cannot get real-time sensor data
- Lidar and hazard data is stale, reducing obstacle avoidance responsiveness

**Current Flow:**
```
Python → subprocess → docker exec → ros2 topic echo → parse YAML
         (shell)       (docker)      (ROS2 init ~400ms)
```

**Target Flow:**
```
Docker: sensor_bridge_node → persistent subscriber → writes JSON to shared file
Host Python: reads JSON file (~10ms)
```

## Architecture Design

### Component Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    Docker Container (landerpi-ros2)              │
│                                                                  │
│  ┌──────────────┐    ┌──────────────────────────────────────┐   │
│  │ lidar_driver │───▶│        sensor_bridge_node             │   │
│  │   /scan      │    │                                       │   │
│  └──────────────┘    │  Persistent subscribers:              │   │
│                      │  - /scan → lidar.json                 │   │
│  ┌──────────────┐    │  - /hazards → hazards.json            │   │
│  │obstacle_fusion│──▶│  - /depth_stats → depth_stats.json    │   │
│  │   /hazards   │    │  - /battery → battery.json            │   │
│  └──────────────┘    │                                       │   │
│                      │  Writes to: /landerpi_data/*.json     │   │
│  ┌──────────────┐    │  (every 50-100ms or on new data)      │   │
│  │ depth_stats  │───▶│                                       │   │
│  │ /depth_stats │    └──────────────────────────────────────┘   │
│  └──────────────┘                    │                          │
│                                      ▼                          │
│  ┌──────────────┐          ┌─────────────────┐                  │
│  │battery_monitor│────────▶│  Shared Volume  │                  │
│  │   /battery   │          │ /landerpi_data/ │                  │
│  └──────────────┘          └─────────────────┘                  │
│                                      │                          │
└──────────────────────────────────────│──────────────────────────┘
                                       │
                                       ▼ (volume mount)
┌──────────────────────────────────────│──────────────────────────┐
│                           Host (Robot)                          │
│                                      │                          │
│                          ~/landerpi_data/                       │
│                          ├── lidar.json                         │
│                          ├── hazards.json                       │
│                          ├── depth_stats.json                   │
│                          └── battery.json                       │
│                                      │                          │
│                                      ▼                          │
│                      ┌─────────────────────────┐                │
│                      │    ROS2Hardware class   │                │
│                      │  read_lidar() → reads   │                │
│                      │  lidar.json (~10ms)     │                │
│                      └─────────────────────────┘                │
│                                      │                          │
│                                      ▼                          │
│                      ┌─────────────────────────┐                │
│                      │   robot_explorer.py     │                │
│                      │   10Hz control loop     │                │
│                      └─────────────────────────┘                │
└─────────────────────────────────────────────────────────────────┘
```

### JSON File Format

**lidar.json:**
```json
{
  "timestamp": 1704307200.123,
  "ranges": [0.5, 0.6, ...],
  "angle_min": -3.14159,
  "angle_increment": 0.00873
}
```

**hazards.json:**
```json
{
  "timestamp": 1704307200.123,
  "hazards": [
    {"type": "person", "distance": 1.2, "angle": 15.0, "score": 0.85}
  ]
}
```

**depth_stats.json:**
```json
{
  "timestamp": 1704307200.123,
  "min_depth_m": 0.45,
  "avg_depth_m": 1.2,
  "valid_percent": 85.5
}
```

**battery.json:**
```json
{
  "timestamp": 1704307200.123,
  "voltage": 7.85
}
```

## Files to Create

### 1. `ros2_nodes/sensor_bridge/` (NEW ROS2 Package)

```
ros2_nodes/sensor_bridge/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/sensor_bridge
└── sensor_bridge/
    ├── __init__.py
    └── sensor_bridge_node.py
```

**sensor_bridge_node.py** - Key responsibilities:
- Subscribe to `/scan`, `/hazards`, `/depth_stats`, `/battery`
- On each message, write to corresponding JSON file in `/landerpi_data/`
- Use atomic writes (write to .tmp, then rename) to prevent partial reads
- Throttle writes to ~20Hz max to reduce I/O (lidar publishes at ~10Hz anyway)

## Files to Modify

### 2. `validation/exploration/ros2_hardware.py`

**Changes:**
- Add new `_read_json_file()` method for atomic JSON file reading
- Modify `read_lidar()` to read from `~/landerpi_data/lidar.json`
- Modify `read_hazards()` to read from `~/landerpi_data/hazards.json`
- Modify `read_depth()` to read from `~/landerpi_data/depth_stats.json`
- Modify `get_battery()` to read from `~/landerpi_data/battery.json`
- Keep subprocess fallback for backwards compatibility
- Add configuration option to choose file-based vs subprocess mode

**Functions to modify:**
| Function | Line | Change |
|----------|------|--------|
| `__init__()` | 50 | Add data_dir config, file paths |
| `read_lidar()` | 220 | Read from lidar.json instead of subprocess |
| `read_hazards()` | 370 | Read from hazards.json instead of subprocess |
| `read_depth()` | 314 | Read from depth_stats.json instead of subprocess |
| `get_battery()` | 484 | Read from battery.json instead of subprocess |

### 3. `docker/docker-compose.yml`

**Changes:**
- Add volume mount: `~/landerpi_data:/landerpi_data`
- Add sensor_bridge to PACKAGES list

**Lines to modify:** ~24 (volumes), ~63 (PACKAGES)

### 4. `ros2_nodes/cmd_vel_bridge/launch/landerpi.launch.py`

**Changes:**
- Add `sensor_bridge_node` to launch description
- Enable by default (required for exploration)

**Lines to add:** After line ~170 (battery_monitor_node)

### 5. `deploy_ros2_stack.py`

**Changes:**
- Ensure `~/landerpi_data/` directory exists on robot before starting stack
- Add sensor_bridge to uploaded packages

### 6. `validation/exploration/__init__.py`

**Changes:**
- Export any new config classes if added

## Implementation Steps

### Phase 1: Create sensor_bridge ROS2 Node
1. Create `ros2_nodes/sensor_bridge/` package structure
2. Implement `sensor_bridge_node.py` with:
   - Subscribers for /scan, /hazards, /depth_stats, /battery
   - JSON serialization with timestamps
   - Atomic file writes (tmp + rename)
   - Throttled write rate (20Hz max)
3. Create `setup.py`, `setup.cfg`, `package.xml`

### Phase 2: Update Docker Configuration
1. Add volume mount in `docker-compose.yml`
2. Add sensor_bridge to PACKAGES list
3. Test container can write to shared volume

### Phase 3: Update Launch File
1. Add sensor_bridge_node to `landerpi.launch.py`
2. Add enable_sensor_bridge parameter (default: true)

### Phase 4: Refactor ros2_hardware.py
1. Add `ROS2Config.data_dir` for shared data directory
2. Add `ROS2Config.use_file_bridge` boolean (default: True)
3. Implement `_read_json_file()` helper
4. Refactor `read_lidar()` to use file reading
5. Refactor `read_hazards()` to use file reading
6. Refactor `read_depth()` to use file reading
7. Refactor `get_battery()` to use file reading
8. Keep subprocess as fallback when file not found

### Phase 5: Update Deployment
1. Update `deploy_ros2_stack.py` to create data directory
2. Upload sensor_bridge package

### Phase 6: Testing
1. Deploy to robot
2. Verify JSON files are being written
3. Measure latency improvement
4. Run exploration test

## Performance Expectations

| Metric | Current | Expected |
|--------|---------|----------|
| Topic read latency | 400-500ms | 5-10ms |
| Cache staleness | 0.5-1.0s | 50-100ms |
| Control loop effective rate | ~2-3Hz | 10Hz |
| Obstacle reaction time | >1s | <200ms |

## Backwards Compatibility

- `use_file_bridge=False` config option falls back to subprocess
- File reading gracefully falls back to subprocess if files don't exist
- No changes to `sensor_fusion.py` or `explorer.py` (they use ROS2Hardware interface)

## Risk Mitigation

1. **File I/O errors**: Use atomic writes and catch exceptions
2. **Stale data**: Include timestamp in JSON, check freshness
3. **Volume permissions**: Docker runs privileged, should have access
4. **Disk space**: JSON files are small (<10KB), rotated on each write
