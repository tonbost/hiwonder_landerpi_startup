# Autonomous Exploration Design

## Overview

Autonomous exploration system for HiWonder LanderPi that uses depth camera + lidar sensor fusion to navigate while avoiding obstacles. Uses frontier-based exploration to systematically cover areas rather than random wandering.

**Goals:**
- Robot roams freely avoiding obstacles (Roomba-style)
- Logs sensor data for future SLAM development
- Controllable via CLI script and TARS voice commands

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Exploration Controller                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │ Sensor Fusion │  │   Frontier   │  │  Safety Monitor  │  │
│  │    Module     │  │   Planner    │  │ (battery/timer)  │  │
│  └──────┬───────┘  └──────┬───────┘  └────────┬─────────┘  │
└─────────│─────────────────│────────────────────│────────────┘
          │                 │                    │
    ┌─────┴─────┐     ┌─────┴─────┐        ┌────┴────┐
    │  Sensors  │     │  Motion   │        │ Battery │
    │Depth+Lidar│     │ /cmd_vel  │        │ Monitor │
    └───────────┘     └───────────┘        └─────────┘
```

**Components:**
1. **Sensor Fusion Module** - Combines depth camera (71° FOV, 30-300cm) for close obstacles with lidar (360°, 0.05-12m) for situational awareness
2. **Frontier Planner** - Tracks recently-seen directions, steers toward unexplored areas
3. **Safety Monitor** - Watches battery level and runtime, triggers auto-stop
4. **Data Logger** - Records sensor data for future SLAM work

## Sensor Fusion Module

### Depth Camera (Primary - immediate obstacles)
- Processes `/aurora/depth/image_raw` at ~10-15 Hz
- Scans center region of depth image for obstacles
- Effective range: 30-300cm (camera's sweet spot)
- Detects small obstacles lidar misses: table legs, chair legs, cables
- Triggers **immediate stop** if obstacle < 20cm in front

### Lidar (Secondary - situational awareness)
- Processes `/scan` at ~10 Hz
- Full 360° coverage, divided into 8 sectors (45° each)
- Tracks minimum distance per sector
- Used for: side/rear awareness, frontier detection, path preference

### Fusion Logic

```python
Every 100ms:
  1. Read depth camera center region → closest_depth_obstacle
  2. Read lidar front 120° → closest_lidar_obstacle
  3. obstacle_distance = min(closest_depth, closest_lidar)

  If obstacle_distance < 0.2m:  → STOP immediately
  If obstacle_distance < 0.5m:  → SLOW + prepare to turn
  If obstacle_distance > 0.5m:  → proceed with frontier plan
```

**Why depth camera as primary:**
- Better at detecting thin/small objects in direct path
- 3D data catches low obstacles (< 10cm) lidar beam might pass over
- Lidar confirms and extends awareness to sides/rear

## Frontier Planner

### Direction Memory
- Divide 360° into 8 sectors (45° each): Front, Front-Right, Right, Back-Right, Back, Back-Left, Left, Front-Left
- Each sector has a "freshness" score (0-100)
- When robot faces a sector, its score resets to 100
- Scores decay over time (~5 points/second)
- Lower score = less recently seen = more interesting

### Exploration Decision

```python
Every 500ms (when not avoiding obstacle):
  1. Get lidar distances for all 8 sectors
  2. Filter out blocked sectors (obstacle < 0.8m)
  3. From open sectors, pick the one with lowest freshness score
  4. Turn toward that sector, then move forward

Special cases:
  - All sectors blocked → back up, spin to find opening
  - Stuck detection → if position unchanged for 10s, try random escape maneuver
```

### Behavior Example
- Robot enters room facing north
- North sector freshness = 100, others start at 50
- Robot moves forward (north is clear)
- As it moves, north stays fresh, east/west decay
- Eventually east or west becomes "stale" → robot turns to explore
- Results in systematic sweeping pattern rather than random wandering

## Safety Monitor

### Runtime Limit
- Default: 30 minutes (configurable via `--duration` flag)
- Countdown displayed in logs every 5 minutes
- At timeout: graceful stop, announce "Exploration complete, stopping"
- Voice can extend: "Hey TARS, keep exploring" adds another 30 minutes

### Battery Threshold
- Check battery every 30 seconds via `board.get_battery()`
- Battery: 7.4V nominal (2S LiPo)
- Warning at 7.0V: "Battery getting low"
- Auto-stop at 6.6V: "Battery low, stopping exploration"
- Default threshold: 6.6V (configurable via `--min-battery`)

### Manual Stop
- Script: Ctrl+C triggers graceful shutdown
- Voice: "Hey TARS, stop" halts exploration
- Emergency: `test_exploration.py stop` command for immediate halt

### Graceful Shutdown Sequence

```python
1. Stop motors immediately
2. Announce status via TTS (if voice enabled)
3. Flush and close data logs
4. Report summary: runtime, distance estimate, battery used
5. Exit cleanly
```

### Status Reporting
- Every 60 seconds, log: runtime elapsed, battery level, sectors explored
- On stop: summary of session

## Data Logging

### Lightweight Logging (default)
- Location: `~/landerpi/exploration_logs/YYYY-MM-DD_HH-MM-SS/`
- Format: JSON Lines (`.jsonl`) - one JSON object per line

**Log files:**
```
session_2024-01-15_14-30-00/
├── metadata.json          # Session info, robot config, start/end time
├── lidar_scans.jsonl      # Timestamped lidar data (1 Hz sampling)
├── depth_summary.jsonl    # Depth stats per frame (not full images)
├── odometry.jsonl         # Motor commands + estimated position
└── events.jsonl           # Obstacles detected, turns made, stops
```

**Lightweight sample (lidar_scans.jsonl):**
```json
{"ts": 1705312200.5, "ranges_min": [0.8, 1.2, 0.5, ...], "ranges_max": [...]}
```
- Stores min/max per sector (8 values), not full 360 points
- ~1KB per minute, hours of logging without filling disk

### ROS2 Bag Mode (`--rosbag` flag)

```bash
uv run python validation/test_exploration.py start --rosbag
```
- Records full `/scan`, `/aurora/depth/image_raw`, `/cmd_vel`
- Stored in `~/landerpi/rosbags/`
- Warning: ~50-100MB per minute with depth camera
- Auto-deletes bags older than 7 days (configurable)

### Future SLAM Use
- Lightweight logs: need conversion script to bag format
- ROS2 bags: directly usable with `slam_toolbox` and `nav2`

## Control Interface

### Script Interface (development/testing)

```bash
# Start exploration (default 30 min, stops at 6.6V)
uv run python validation/test_exploration.py start --yes

# Custom settings
uv run python validation/test_exploration.py start --duration 60 --min-battery 7.0 --yes

# With ROS2 bag recording
uv run python validation/test_exploration.py start --rosbag --yes

# Stop exploration (from another terminal)
uv run python validation/test_exploration.py stop

# Check status
uv run python validation/test_exploration.py status
```

### Voice Commands (via TARS)

| Command | Action |
|---------|--------|
| "Hey TARS, explore" | Start exploration (30 min default) |
| "Hey TARS, stop" | Stop exploration |
| "Hey TARS, keep exploring" | Extend by 30 minutes |
| "Hey TARS, status" | Report battery, runtime, state |

### Voice Integration
- Add new commands to `robot_voicecontroller.py`
- LLM maps "explore", "wander", "look around" → `explore` action
- Exploration runs as background thread, voice remains responsive
- "Stop" interrupts exploration immediately

### Feedback
- TTS announces: "Starting exploration", "Obstacle avoided", "Battery low, stopping"
- Beep patterns: single beep = direction change, double beep = obstacle detected

## File Structure

### New Files

```
validation/
├── test_exploration.py        # Main CLI script (start/stop/status)
└── exploration/
    ├── __init__.py
    ├── explorer.py            # Main exploration controller
    ├── sensor_fusion.py       # Depth + lidar fusion logic
    ├── frontier_planner.py    # Direction memory + planning
    ├── safety_monitor.py      # Battery + timer checks
    └── data_logger.py         # Lightweight + rosbag logging

robot_voicecontroller.py       # Add explore/stop commands (modify existing)
```

### Dependencies
- Existing: `ros_robot_controller_sdk`, Docker ROS2 stack
- New: None (uses existing sensor infrastructure)

### Runtime Architecture

```
test_exploration.py
       │
       ▼
┌─────────────────────────────────────┐
│         ExplorationController       │
│  (main loop, 10Hz)                  │
│                                     │
│  ┌─────────┐ ┌─────────┐ ┌───────┐ │
│  │ Sensor  │ │Frontier │ │Safety │ │
│  │ Fusion  │ │Planner  │ │Monitor│ │
│  └────┬────┘ └────┬────┘ └───┬───┘ │
└───────│───────────│──────────│─────┘
        │           │          │
   SSH+Docker    /cmd_vel    SDK
   (sensors)     (motion)   (battery)
```

Runs locally on development machine, connects to robot via SSH for sensors and motion (same pattern as existing validation scripts).

## Parameters

| Parameter | Value |
|-----------|-------|
| Stop distance | 20cm |
| Slow distance | 50cm |
| Speed | 0.2 m/s |
| Runtime default | 30 minutes |
| Battery cutoff | 6.6V |
| Battery warning | 7.0V |
| Loop rate | 10 Hz |
| Frontier sectors | 8 (45° each) |

## Implementation Order

1. Sensor fusion module (depth + lidar reading)
2. Basic obstacle avoidance (stop/slow/turn)
3. Frontier planner (direction memory)
4. Safety monitor (battery + timer)
5. Data logger (lightweight first)
6. Voice integration
7. ROS2 bag support
