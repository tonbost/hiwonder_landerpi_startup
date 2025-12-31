# LanderPi Architecture Reference

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              LOCAL MACHINE                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│  CLI Tools (Typer + Rich)                                                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐    │
│  │ setup_       │  │ deploy_      │  │ check        │  │ validation/  │    │
│  │ landerpi.py  │  │ ros2_stack.py│  │ LanderPi.py  │  │ test_*.py    │    │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘    │
│         │                 │                 │                  │            │
│         └─────────────────┴─────────────────┴──────────────────┘            │
│                                    │                                         │
│                            Fabric (SSH)                                      │
└────────────────────────────────────┼────────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           RASPBERRY PI 5 (Robot)                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                        Docker Container (landerpi-ros2)                 │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                  │ │
│  │  │ cmd_vel_     │  │ arm_         │  │ lidar_       │                  │ │
│  │  │ bridge       │  │ controller   │  │ driver       │                  │ │
│  │  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘                  │ │
│  │         │                 │                 │                           │ │
│  │         └─────────────────┴─────────────────┘                           │ │
│  │                           │                                              │ │
│  │                    ROS2 Topics                                           │ │
│  │         /cmd_vel  /servo_controller  /scan  /aurora/*                   │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                    │                                         │
│                           Serial/USB                                         │
│                                    │                                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐    │
│  │ STM32 Motor  │  │ Bus Servos   │  │ LD19/MS200   │  │ Aurora 930   │    │
│  │ Controller   │  │ (Arm)        │  │ Lidar        │  │ Depth Camera │    │
│  │ /dev/ttyACM0 │  │ via STM32    │  │ /dev/ldlidar │  │ USB          │    │
│  └──────────────┘  └──────────────┘  └──────────────┘  └──────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Component Interactions

### Direct SDK Control (No ROS2)

```
Local Machine                          Robot
┌─────────────┐                       ┌─────────────────────┐
│ test_arm.py │ ──── SSH ──────────▶ │ Python script       │
└─────────────┘                       │ + SDK               │
                                      └──────────┬──────────┘
                                                 │
                                                 ▼
                                      ┌─────────────────────┐
                                      │ ros_robot_          │
                                      │ controller_sdk.py   │
                                      └──────────┬──────────┘
                                                 │ Serial
                                                 ▼
                                      ┌─────────────────────┐
                                      │ STM32 Controller    │
                                      │ /dev/ttyACM0        │
                                      └─────────────────────┘
```

### ROS2-Based Control

```
Local Machine                          Robot
┌─────────────────┐                   ┌─────────────────────┐
│ test_arm_ros2.py│ ──── SSH ──────▶ │ docker exec         │
└─────────────────┘                   │ ros2 topic pub      │
                                      └──────────┬──────────┘
                                                 │
                                                 ▼
                                      ┌─────────────────────┐
                                      │ Docker Container    │
                                      │ landerpi-ros2       │
                                      │ ┌─────────────────┐ │
                                      │ │ ROS2 Node       │ │
                                      │ │ arm_controller  │ │
                                      │ └────────┬────────┘ │
                                      └──────────┼──────────┘
                                                 │ Serial
                                                 ▼
                                      ┌─────────────────────┐
                                      │ STM32 Controller    │
                                      └─────────────────────┘
```

## Exploration Module Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          ExplorationController                               │
│                                                                              │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐                │
│  │ SafetyMonitor  │  │ SensorFusion   │  │ FrontierPlanner│                │
│  │ - battery      │  │ - lidar        │  │ - sectors      │                │
│  │ - runtime      │  │ - depth        │  │ - goal select  │                │
│  │ - emergency    │  │ - obstacles    │  │                │                │
│  └────────────────┘  └────────────────┘  └────────────────┘                │
│                                                                              │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐                │
│  │ EscapeHandler  │  │ ArmScanner     │  │ DataLogger     │                │
│  │ - stuck detect │  │ - depth scan   │  │ - telemetry    │                │
│  │ - progressive  │  │ - pan sweep    │  │ - events       │                │
│  │ - 360° scan    │  │ - find opening │  │ - rosbag       │                │
│  └────────────────┘  └────────────────┘  └────────────────┘                │
│                                                                              │
│                         Callbacks (Dependency Injection)                     │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │ move_func | stop_func | get_battery_func | get_lidar_func | ...      │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                             ROS2Hardware                                     │
│  Hardware abstraction layer - implements all callbacks                       │
│                                                                              │
│  move()        → docker exec ros2 topic pub /cmd_vel                        │
│  get_lidar()   → docker exec ros2 topic echo /scan                          │
│  get_depth()   → docker exec ros2 topic echo /depth_stats                   │
│  get_battery() → docker exec ros2 topic echo /battery                       │
│  set_servo()   → docker exec ros2 topic pub /servo_controller               │
└─────────────────────────────────────────────────────────────────────────────┘
```

## ROS2 Topic Map

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            ROS2 Topics                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  Motion Control                                                              │
│  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐       │
│  │ /cmd_vel        │ ──▶ │ cmd_vel_bridge  │ ──▶ │ /ros_robot_     │       │
│  │ Twist           │     │ node            │     │ controller/     │       │
│  │ linear.x,y      │     │                 │     │ set_motor       │       │
│  │ angular.z       │     │ mecanum         │     │ Int32MultiArray │       │
│  └─────────────────┘     │ kinematics      │     └─────────────────┘       │
│                          └─────────────────┘                                 │
│                                                                              │
│  Arm Control                                                                 │
│  ┌─────────────────┐                                                        │
│  │ /servo_         │     ServosPosition message:                            │
│  │ controller      │     - duration: float64                                │
│  │                 │     - position_unit: "pulse"                           │
│  │                 │     - position[]: ServoPosition(id, position)          │
│  └─────────────────┘                                                        │
│                                                                              │
│  Sensors                                                                     │
│  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐       │
│  │ /scan           │     │ /depth_stats    │     │ /battery        │       │
│  │ LaserScan       │     │ custom JSON     │     │ Float32         │       │
│  │ 360° lidar      │     │ min/avg depth   │     │ voltage         │       │
│  └─────────────────┘     └─────────────────┘     └─────────────────┘       │
│                                                                              │
│  Camera (Aurora 930)                                                         │
│  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐       │
│  │ /aurora/rgb/    │     │ /aurora/depth/  │     │ /aurora/        │       │
│  │ image_color     │     │ image           │     │ points          │       │
│  │ Image           │     │ Image           │     │ PointCloud2     │       │
│  └─────────────────┘     └─────────────────┘     └─────────────────┘       │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Deployment State Machine

```
setup_landerpi.py uses marker files for idempotent deployment:

~/.landerpi_setup/
├── system_update.done
├── hardware_config.done
├── peripheral_access.done
├── motion_controller.done
├── camera_setup.done
├── docker_ros2.done
├── camera_driver_build.done
└── remote_access.done

Each step:
1. Check if marker exists → skip if done
2. Execute step
3. Create marker file
4. Continue to next step

Resume after interruption:
- Reads existing markers
- Skips completed steps
- Continues from first incomplete step
```

## Hardware Specifications

### Motor Controller (STM32)
- Device: `/dev/ttyACM0` (symlink: `/dev/rrc`)
- Baud rate: 1,000,000
- Protocol: Custom packet-based
- Motors: M1 (front-left), M2 (back-left), M3 (front-right), M4 (back-right)
- Mecanum wheel kinematics with slip compensation

### Bus Servos (Arm)
- IDs: 1-5 (arm joints), 10 (gripper)
- Protocol: Via STM32 controller
- Position range: 0-1000 (500 = center)
- Communication: Serial bus through motor controller

### Lidar (LD19/MS200)
- Device: `/dev/ldlidar` or `/dev/ttyUSB0`
- Baud rate: 230,400
- Protocol: Custom packet (start byte 0x54)
- Output: 360° scan, ~10Hz update rate

### Depth Camera (Aurora 930)
- Interface: USB 3.0
- Vendor ID: 3251 (Deptrum)
- ROS2 driver: Deptrum SDK
- Topics: RGB, Depth, PointCloud2

## File Naming Patterns

```
Root level:
├── setup_*.py          # Initial setup/deployment (stateful)
├── deploy_*.py         # Component deployment (idempotent)
├── check*.py           # Diagnostic/health checks
├── robot_*.py          # Scripts that run ON the robot

validation/:
├── test_*_direct.py    # Direct SDK tests (no ROS2)
├── test_*_ros2.py      # ROS2-based tests
├── test_*.py           # Generic tests (may use either)
└── exploration/        # Modular packages use directories

ros2_nodes/:
├── node_name/
│   ├── node_name/
│   │   ├── __init__.py
│   │   └── node_name_node.py
│   ├── setup.py
│   └── package.xml
```

## Configuration Hierarchy

```
1. Command-line options (highest priority)
   --host, --user, --password

2. config.json (project root)
   {"host": "...", "user": "...", "password": "..."}

3. Dataclass defaults (lowest priority)
   @dataclass
   class Config:
       speed: float = 0.35
```

## Error Recovery Patterns

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Error Recovery Flow                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  Connection Error                                                            │
│  ┌─────────┐     ┌─────────────┐     ┌─────────────┐                       │
│  │ SSH     │ ──▶ │ Retry with  │ ──▶ │ Exit with   │                       │
│  │ fails   │     │ timeout     │     │ clear error │                       │
│  └─────────┘     └─────────────┘     └─────────────┘                       │
│                                                                              │
│  Robot Movement Error                                                        │
│  ┌─────────┐     ┌─────────────┐     ┌─────────────┐                       │
│  │ Command │ ──▶ │ Emergency   │ ──▶ │ Log and     │                       │
│  │ fails   │     │ stop()      │     │ report      │                       │
│  └─────────┘     └─────────────┘     └─────────────┘                       │
│                                                                              │
│  Exploration Stuck                                                           │
│  ┌─────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────────┐  │
│  │ Stuck   │ ──▶ │ Level 1:    │ ──▶ │ Level 2:    │ ──▶ │ Level 3:    │  │
│  │ detect  │     │ Backup      │     │ Turn + scan │     │ 360° scan   │  │
│  └─────────┘     └─────────────┘     └─────────────┘     └─────────────┘  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```
