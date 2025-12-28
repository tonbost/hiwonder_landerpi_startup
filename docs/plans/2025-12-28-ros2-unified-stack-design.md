# ROS2 Unified Stack Design

**Date:** 2025-12-28
**Status:** Approved
**Goal:** Standardize all sensor testing to use persistent ROS2 stack pattern (like test_chassis_motion.py)

## Problem

Current test scripts have inconsistent approaches:
- `test_chassis_motion.py` - Uses deployed ROS2 stack via `ros2 topic pub`
- `test_arm.py` - Direct SDK via SSH (no ROS2)
- `test_lidar.py` - Ad-hoc Docker containers, mixed approach
- `test_cameradepth.py` - Ad-hoc Docker containers, mixed approach

This creates confusion and prevents integration with nav2/SLAM workflows.

## Solution

1. Create new `*_ros2.py` test scripts that follow the `test_chassis_motion.py` pattern
2. Add all drivers to the persistent Docker Compose stack
3. Keep existing direct SDK scripts for quick testing/debugging
4. Update agent and skills with this implementation strategy

## Architecture

```
┌───────────────────────────────────────────────────────────────────────┐
│         Docker Compose Stack (restart: unless-stopped)                 │
├───────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌───────────────┐  ┌───────────────┐  ┌─────────────────────────┐   │
│  │ cmd_vel_bridge│  │ arm_controller │  │ lidar_driver (LD19)     │   │
│  │               │  │               │  │                         │   │
│  │ /cmd_vel ────►│  │ /arm/cmd ────►│  │ ◄──── /scan (publish)   │   │
│  │ (Twist)       │  │ (ArmCmd)      │  │       (LaserScan)       │   │
│  └───────────────┘  └───────────────┘  └─────────────────────────┘   │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │ camera_driver (Aurora 930) - vendor package                      │ │
│  │                                                                   │ │
│  │ ◄──── /aurora/rgb/image_raw, /aurora/depth/image_raw, etc.       │ │
│  └─────────────────────────────────────────────────────────────────┘ │
│                                                                       │
│  Devices: /dev/ttyAMA0 (motors), /dev/ttyUSB0 (lidar), USB camera    │
└───────────────────────────────────────────────────────────────────────┘
```

## Testing Modes

### Direct SDK Tests (No ROS2)
- Scripts: `test_chassis_direct.py`, `test_arm.py`, `test_lidar.py`, `test_cameradepth.py`
- Use `ros_robot_controller_sdk.py` for direct serial communication
- Self-contained, start their own Docker containers if needed
- Good for: Quick testing, debugging, when ROS2 stack is not deployed

### ROS2 Stack Tests (Integrated)
- Scripts: `test_chassis_motion.py`, `test_arm_ros2.py`, `test_lidar_ros2.py`, `test_cameradepth_ros2.py`
- Require persistent ROS2 stack: `deploy_ros2_stack.py deploy`
- Use standard ROS2 topics via `docker exec landerpi-ros2`
- Good for: Production use, integration testing, nav2/SLAM workflows

### When to Use Which

| Scenario | Use Direct | Use ROS2 |
|----------|------------|----------|
| Quick hardware verification | Yes | |
| Debugging motor/servo issues | Yes | |
| Integration with nav2/SLAM | | Yes |
| Production deployment | | Yes |
| Robot offline/recovery | Yes | |

## File Structure

### New ROS2 Nodes

```
ros2_nodes/
├── cmd_vel_bridge/          # EXISTS - motion control
│   └── ...
├── arm_controller/          # NEW - arm servo control
│   ├── package.xml
│   ├── setup.py
│   └── arm_controller/
│       ├── __init__.py
│       └── arm_controller_node.py
└── lidar_driver/            # NEW - LD19/MS200 driver
    ├── package.xml
    ├── setup.py
    └── lidar_driver/
        ├── __init__.py
        └── ld19_driver_node.py
```

Camera uses existing vendor package from `~/deptrum_ws`.

### ROS2 Topics

| Node | Subscribes | Publishes |
|------|------------|-----------|
| `cmd_vel_bridge` | `/cmd_vel` (Twist) | `/ros_robot_controller/set_motor` |
| `arm_controller` | `/arm/cmd` (custom) | `/arm/state` (optional) |
| `lidar_driver` | - | `/scan` (LaserScan) |
| `camera_driver` | - | `/aurora/*` (vendor topics) |

## Test Script Pattern

All `*_ros2.py` scripts follow this pattern:

```python
class SensorTest:
    def __init__(self, host, user, password):
        self.conn = Connection(host=host, user=user, connect_kwargs={'password': password})

    def check_ros2_topic(self, topic) -> bool:
        """Check if topic exists in the running stack."""
        result = self.conn.run(
            f"docker exec landerpi-ros2 bash -c '"
            f"source /opt/ros/humble/setup.bash && "
            f"source /ros2_ws/install/setup.bash && "
            f"ros2 topic list | grep {topic}'"
        )
        return result.ok

    def publish_command(self, topic, msg_type, data):
        """Publish a command to the stack."""
        self.conn.run(
            f"docker exec landerpi-ros2 bash -c '"
            f"source /opt/ros/humble/setup.bash && "
            f"source /ros2_ws/install/setup.bash && "
            f"ros2 topic pub --once {topic} {msg_type} \"{data}\"'"
        )

    def read_topic(self, topic, timeout=5):
        """Read one message from a topic."""
        self.conn.run(
            f"docker exec landerpi-ros2 bash -c '"
            f"timeout {timeout} ros2 topic echo --once {topic}'"
        )
```

## Docker Compose Changes

```yaml
services:
  ros2-stack:
    image: landerpi-ros2:latest
    container_name: landerpi-ros2
    restart: unless-stopped
    network_mode: host
    privileged: true

    devices:
      - /dev/ttyAMA0:/dev/ttyAMA0    # Motor controller
      - /dev/ttyUSB0:/dev/ttyUSB0    # Lidar (primary)
      - /dev/ttyUSB1:/dev/ttyUSB1    # Lidar (fallback)

    volumes:
      # Custom nodes
      - ../ros2_nodes/cmd_vel_bridge:/ros2_ws/src/cmd_vel_bridge:ro
      - ../ros2_nodes/arm_controller:/ros2_ws/src/arm_controller:ro
      - ../ros2_nodes/lidar_driver:/ros2_ws/src/lidar_driver:ro
      # Vendor drivers
      - ../drivers/ros_robot_controller-ros2/src/ros_robot_controller:/ros2_ws/src/ros_robot_controller:ro
      - ../drivers/ros_robot_controller-ros2/src/ros_robot_controller_msgs:/ros2_ws/src/ros_robot_controller_msgs:ro
      # Camera driver workspace (built on robot)
      - ~/deptrum_ws:/deptrum_ws:ro
      # Config
      - ../config:/ros2_ws/config:ro

    command: >
      bash -c "
        source /opt/ros/humble/setup.bash
        cd /ros2_ws
        colcon build --symlink-install
        source install/setup.bash
        source /deptrum_ws/install/local_setup.bash 2>/dev/null || true
        ros2 launch cmd_vel_bridge landerpi.launch.py
      "
```

## Implementation Plan

### Phase 1: Core Infrastructure
- [ ] Create `ros2_nodes/arm_controller/` package
- [ ] Create `ros2_nodes/lidar_driver/` package
- [ ] Update `docker/docker-compose.yml` with new mounts
- [ ] Update `ros2_nodes/cmd_vel_bridge/launch/landerpi.launch.py` to launch all nodes

### Phase 2: Test Scripts
- [ ] Create `test_arm_ros2.py`
- [ ] Create `test_lidar_ros2.py`
- [ ] Create `test_cameradepth_ros2.py`

### Phase 3: Documentation
- [ ] Update `.claude/agents/landerpi-robot.md` with testing modes
- [ ] Update `.claude/skills/landerpi-core/SKILL.md` with stack architecture
- [ ] Update `.claude/skills/landerpi-motion/SKILL.md` with ROS2 test reference
- [ ] Update `.claude/skills/landerpi-arm/SKILL.md` with ROS2 test reference
- [ ] Update `.claude/skills/landerpi-lidar/SKILL.md` with ROS2 test reference
- [ ] Update `.claude/skills/landerpi-camera/SKILL.md` with ROS2 test reference
- [ ] Update `CLAUDE.md` with new commands

## New Test Scripts

### test_arm_ros2.py
Commands: `test`, `home`, `status`, `stop`
- Publishes to `/arm/cmd` topic
- Reads from `/arm/state` topic

### test_lidar_ros2.py
Commands: `check`, `scan`, `status`
- Reads from `/scan` topic
- No publishing needed (read-only sensor)

### test_cameradepth_ros2.py
Commands: `check`, `stream`, `status`
- Reads from `/aurora/*` topics
- No publishing needed (read-only sensor)

## Constraints

- **Never modify `drivers/` directory** - vendor code stays untouched
- Keep existing direct SDK test scripts unchanged
- New scripts use `_ros2` suffix for clarity
- All ROS2 tests require stack to be deployed first

## Testing

```bash
# Deploy stack first
uv run python deploy_ros2_stack.py deploy

# Test motion (existing)
uv run python test_chassis_motion.py test

# Test arm (new)
uv run python test_arm_ros2.py test --yes

# Test lidar (new)
uv run python test_lidar_ros2.py scan --samples 5

# Test camera (new)
uv run python test_cameradepth_ros2.py stream
```
