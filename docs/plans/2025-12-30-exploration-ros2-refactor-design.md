# Exploration ROS2 Refactor Design

**Date:** 2025-12-30
**Status:** Approved

## Goals

1. **Performance** - On-robot execution for fast sensor reading (~10x faster than SSH)
2. **Maintainability** - Modular structure from validation/exploration/
3. **ROS2-first** - Minimize SDK/serial access, use ROS2 topics for all hardware
4. **Depth camera integration** - Both escape scanning and continuous fusion modes

## Architecture

### Directory Structure

```
landerpi/                          # On robot: ~/landerpi/
├── exploration/                   # Module folder
│   ├── __init__.py
│   ├── ros2_hardware.py          # NEW: ROS2 interface layer
│   ├── explorer.py               # Main controller (adapted)
│   ├── frontier_planner.py       # Navigation logic (as-is)
│   ├── sensor_fusion.py          # Lidar + depth fusion (adapted)
│   ├── arm_scanner.py            # Depth camera scanning (adapted)
│   ├── escape_handler.py         # Escape logic (as-is)
│   ├── safety_monitor.py         # Battery/runtime (adapted for ROS2)
│   └── data_logger.py            # Local logging (as-is)
├── robot_explorer.py             # Entry point (simplified)
└── exploration_logs/             # Session data
```

### ROS2 Topics

| Component | Topic | Message Type | Direction |
|-----------|-------|--------------|-----------|
| Motion | `/cmd_vel` | geometry_msgs/Twist | Publish |
| Lidar | `/scan` | sensor_msgs/LaserScan | Subscribe |
| Depth | `/aurora/depth/image_raw` | sensor_msgs/Image | Subscribe |
| Arm commands | `/arm/cmd` | std_msgs/String (JSON) | Publish |
| Arm state | `/arm/state` | std_msgs/String (JSON) | Subscribe |
| Battery | `/battery` | std_msgs/Float32 | Subscribe |

### ROS2 Hardware Layer

`ros2_hardware.py` provides all hardware access via ROS2 topics:

```python
class ROS2Hardware:
    """Hardware interface via ROS2 topics."""

    # Motion control
    def move(self, vx: float, vy: float, wz: float) -> bool
    def stop(self) -> None

    # Lidar
    def read_lidar(self) -> tuple[list, float, float]  # ranges, angle_min, angle_inc

    # Depth camera
    def read_depth(self) -> dict | None  # {min_depth, avg_depth, valid_percent}

    # Arm control
    def set_arm_pose(self, positions: list, duration: float) -> bool
    def set_servo(self, servo_id: int, position: int, duration: float) -> bool

    # Battery
    def get_battery(self) -> float | None
```

Implementation uses `docker exec landerpi-ros2 ros2 topic pub/echo` commands.

### Sensor Fusion

Unified obstacle detection combining lidar and depth camera:

- **Lidar:** 360° coverage, 12m range
- **Depth (Aurora 930):** ~60° FOV, 0.15m-3m range, better for close/thin obstacles

Fusion logic:
- Front sector: use minimum of (lidar, depth) when both valid
- Side sectors: lidar only
- Arm scan mode: depth only (arm pans camera 180°)

### Battery Monitor Node

New ROS2 node: `ros2_nodes/battery_monitor/`

- Publishes `/battery` (std_msgs/Float32) at 1Hz
- Uses SDK internally (only node that touches serial for battery)
- Added to docker-compose.yml

## Files to Create

| File | Purpose |
|------|---------|
| `validation/exploration/ros2_hardware.py` | ROS2 interface layer |
| `ros2_nodes/battery_monitor/battery_monitor_node.py` | Battery publisher |
| `ros2_nodes/battery_monitor/setup.py` | Package setup |
| `ros2_nodes/battery_monitor/package.xml` | Package manifest |

## Files to Modify

| File | Changes |
|------|---------|
| `robot_explorer.py` | Simplify to thin CLI, import from exploration/ |
| `validation/exploration/sensor_fusion.py` | Add continuous depth fusion |
| `validation/exploration/safety_monitor.py` | Use ROS2 battery topic |
| `validation/exploration/arm_scanner.py` | Use ROS2 arm commands |
| `ros2_nodes/arm_controller/arm_controller_node.py` | Fix servo 4 = 350 |
| `deploy_explorer.py` | Deploy module folder |
| `docker/docker-compose.yml` | Add battery_monitor node |

## Implementation Phases

### Phase 1: ROS2 Hardware Layer
- Create `ros2_hardware.py` with all ROS2 topic wrappers
- Test each method independently

### Phase 2: Battery Monitor Node
- Create `battery_monitor` node
- Add to docker-compose.yml
- Deploy and test

### Phase 3: Adapt Exploration Modules
- Update `sensor_fusion.py` for depth integration
- Update `safety_monitor.py` to use /battery topic
- Update `arm_scanner.py` to use /arm/cmd topic
- Keep dependency injection pattern

### Phase 4: Entry Point & Deployment
- Simplify `robot_explorer.py` to thin CLI
- Update `deploy_explorer.py` for module deployment
- Fix arm_controller HOME_POSITIONS (servo 4 = 350)

### Phase 5: Integration Testing
- Deploy to robot
- Test lidar-only mode
- Test depth fusion mode
- Test arm scanning escape
- Test battery monitoring

## Trade-offs Considered

### Approach 1 (Chosen): Adapt existing modules
- Reuse tested validation/exploration/ code
- Minimal rewrite via dependency injection
- Some cleanup of SSH assumptions needed

### Approach 2 (Rejected): Clean rewrite
- Cleaner result but more work
- Risk of losing battle-tested escape logic

### Approach 3 (Rejected): Keep both implementations
- Two codepaths to maintain
- Divergence over time

## Dependencies

- ROS2 Humble (in Docker)
- ros_robot_controller_msgs (vendor package)
- Aurora 930 depth camera driver
- LD19 lidar driver
