# ROS2 cmd_vel Bridge Design

**Date:** 2025-12-28
**Status:** Approved
**Goal:** Enable standard ROS2 `/cmd_vel` control with persistence across reboots

## Problem

The LanderPi has a `ros_robot_controller` ROS2 package that communicates with the STM32 via serial, but it uses a custom `~/set_motor` topic instead of the standard `/cmd_vel` (Twist) interface. This prevents integration with standard ROS2 navigation and teleop tools.

## Solution

Create a separate `cmd_vel_bridge` node that:
1. Subscribes to `/cmd_vel` (geometry_msgs/Twist)
2. Performs mecanum kinematics calculations
3. Publishes to `/ros_robot_controller/set_motor` (MotorsState)

Deploy via Docker Compose with `restart: unless-stopped` for persistence.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Docker Compose Stack                          │
│  (restart: unless-stopped - survives reboot)                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────┐       ┌──────────────────────────────┐   │
│  │  cmd_vel_bridge  │       │   ros_robot_controller       │   │
│  │                  │       │                              │   │
│  │ /cmd_vel ──────────────► /ros_robot_controller/set_motor│   │
│  │ (Twist)          │       │ (MotorsState)                │   │
│  │                  │       │                              │   │
│  │ Mecanum kinematic│       │ SDK serial ──► STM32         │   │
│  └──────────────────┘       └──────────────────────────────┘   │
│                                                                  │
│  Shared: /dev/ttyAMA0 (serial to STM32)                         │
└─────────────────────────────────────────────────────────────────┘
```

## File Structure

```
hiwonderSetup/
├── drivers/                    # NEVER MODIFY
├── ros2_nodes/
│   └── cmd_vel_bridge/
│       ├── package.xml
│       ├── setup.py
│       └── cmd_vel_bridge/
│           ├── __init__.py
│           └── cmd_vel_bridge_node.py
├── docker/
│   ├── Dockerfile.ros2-stack
│   └── docker-compose.yml
└── config/
    └── robot_params.yaml
```

## cmd_vel Bridge Node

### Topics

| Direction | Topic | Message Type |
|-----------|-------|--------------|
| Subscribe | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| Publish | `/ros_robot_controller/set_motor` | `ros_robot_controller_msgs/msg/MotorsState` |

### Mecanum Kinematics

```python
wheel_fl = (vx - vy - (lx + ly) * wz) / wheel_radius
wheel_bl = (vx + vy - (lx + ly) * wz) / wheel_radius
wheel_fr = (vx + vy + (lx + ly) * wz) / wheel_radius
wheel_br = (vx - vy + (lx + ly) * wz) / wheel_radius
```

Where:
- `vx` = linear.x (forward/backward m/s)
- `vy` = linear.y (strafe left/right m/s)
- `wz` = angular.z (rotation rad/s)
- `lx` = wheel_base / 2
- `ly` = track_width / 2

### Parameters (`robot_params.yaml`)

```yaml
cmd_vel_bridge:
  ros__parameters:
    wheel_radius: 0.05      # 50mm (adjustable)
    wheel_base: 0.15        # 150mm front-to-back
    track_width: 0.15       # 150mm left-to-right
    max_wheel_rps: 3.0      # Safety limit
    cmd_vel_timeout: 0.5    # Watchdog timeout (seconds)
```

### Safety Features

- **Watchdog:** Stops motors if no cmd_vel received for 500ms
- **Speed clamping:** Limits wheel RPS to configured maximum
- **Graceful shutdown:** Sends zero velocity on SIGINT

## Docker Compose

```yaml
services:
  ros2-stack:
    image: landerpi-ros2:latest
    container_name: landerpi-ros2
    restart: unless-stopped
    network_mode: host
    privileged: true
    devices:
      - /dev/ttyAMA0:/dev/ttyAMA0
    volumes:
      - ../ros2_nodes:/ros2_ws/src/cmd_vel_bridge:ro
      - ../drivers/ros_robot_controller-ros2/src:/ros2_ws/src/vendor:ro
      - ../config:/ros2_ws/config:ro
    environment:
      - ROS_DOMAIN_ID=0
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        cd /ros2_ws &&
        colcon build --symlink-install &&
        source install/setup.bash &&
        ros2 launch landerpi_bringup landerpi.launch.py
      "
```

## Deployment Steps

1. Create directories on robot
2. Upload `ros2_nodes/`, `docker/`, `config/` to robot
3. Run `docker compose up -d --build`
4. Verify with `docker ps`
5. Test with `test_chassis_motion.py`

## Constraints

- **Never modify `drivers/` directory** - vendor code stays untouched
- Use existing `landerpi-ros2:latest` Docker image as base
- Parameters must be tunable without code changes

## Testing

```bash
# From Mac:
uv run python test_chassis_motion.py test --host 192.168.50.169 --user tonbost --password <pass>

# Quick test on robot:
docker exec -it landerpi-ros2 bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'"
```
