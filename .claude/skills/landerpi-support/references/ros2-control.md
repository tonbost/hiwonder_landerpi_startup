# ROS2 Control Reference

## Topics

### Velocity Command Topic

| Property | Value |
|----------|-------|
| Topic | `/ros_robot_controller/cmd_vel` |
| Type | `geometry_msgs/msg/Twist` |
| Publisher | Motion control scripts |
| Subscriber | ros_robot_controller node |

### IMU Topic

| Property | Value |
|----------|-------|
| Topic | `/ros_robot_controller/imu_raw` |
| Type | `sensor_msgs/msg/Imu` |
| Publisher | ros_robot_controller node |

### Odometry Topic

| Property | Value |
|----------|-------|
| Topic | `/odom_raw` |
| Type | `nav_msgs/msg/Odometry` |
| Publisher | odom_publisher node |

## Message Formats

### geometry_msgs/msg/Twist

```yaml
linear:
  x: float64  # Forward/backward velocity (m/s)
  y: float64  # Lateral velocity (m/s) - Mecanum only
  z: float64  # Unused for ground robots
angular:
  x: float64  # Unused
  y: float64  # Unused
  z: float64  # Rotation velocity (rad/s)
```

### Velocity Limits

| Parameter | Min | Max | Unit |
|-----------|-----|-----|------|
| linear.x | -0.3 | 0.3 | m/s |
| linear.y | -0.3 | 0.3 | m/s |
| angular.z | -1.0 | 1.0 | rad/s |

## Launch Files

### Start Robot Controller
```bash
ros2 launch ros_robot_controller ros_robot_controller.launch.py
```

### Start Odometry Publisher
```bash
ros2 launch controller odom_publisher.launch.py
```

### Start IMU Filter
```bash
ros2 launch peripherals imu_filter.launch.py
```

### View IMU in RViz
```bash
ros2 launch peripherals imu_view.launch.py
```

## Kinematics

### Mecanum Wheel Equations

Forward kinematics (wheel speeds to robot velocity):
- `Vx = (VA + VB + VC + VD) / 4`
- `Vy = (-VA + VB + VC - VD) / 4`
- `Vω = (-VA + VB - VC + VD) / (4 * (L + H))`

Inverse kinematics (robot velocity to wheel speeds):
- `VA = Vx - Vy - (L + H) * Vω`
- `VB = Vx + Vy + (L + H) * Vω`
- `VC = Vx + Vy - (L + H) * Vω`
- `VD = Vx - Vy + (L + H) * Vω`

Where:
- `L` = Half track width
- `H` = Half wheelbase
- `VA, VB, VC, VD` = Wheel velocities

### Ackerman Steering

```
R = H / tan(θ)
VL = V * (R - D/2) / R
VR = V * (R + D/2) / R
```

Where:
- `R` = Turning radius
- `H` = Wheelbase
- `D` = Track width
- `θ` = Steering angle
- `V` = Linear velocity

## Calibration Files

Configuration stored in:
```
~/ros2_ws/src/driver/controller/config/calibrate_params.yaml
```

Parameters:
- `linear_correction_factor`: Linear velocity scaling (default 1.0)
- `angular_correction_factor`: Angular velocity scaling (default 1.0)
