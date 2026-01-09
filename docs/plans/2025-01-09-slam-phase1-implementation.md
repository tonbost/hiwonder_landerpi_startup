# Phase 1: SLAM Navigation Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Enable the robot to build maps with rtabmap, localize itself, and navigate to coordinates with Nav2.

**Architecture:** Docker-based ROS2 stack with rtabmap for SLAM (lidar-only mode), Nav2 for path planning, wheel odometry for localization, and static TF for sensor frames. All nodes launch via the existing `landerpi.launch.py` pattern with enable flags.

**Tech Stack:** ROS2 Humble, rtabmap_ros, Nav2, tf2_ros, Docker

---

## Prerequisites

- Existing ROS2 stack deployed (`deploy_ros2_stack.py deploy`)
- Lidar working (`/scan` topic publishing)
- Robot can move via `/cmd_vel`

## TF Tree Target

```
map (published by rtabmap)
 └── odom (published by odom_publisher)
      └── base_footprint (robot base)
           └── laser_frame (lidar, static transform)
```

---

## Task 1: Add rtabmap-ros to Docker Image

**Files:**
- Modify: `docker/Dockerfile`

**Step 1: Add rtabmap packages to Dockerfile**

```dockerfile
# Add after line 17 (ros-humble-slam-toolbox \)
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
```

**Step 2: Verify syntax**

Run locally:
```bash
cat docker/Dockerfile | grep rtabmap
```
Expected: See `ros-humble-rtabmap` and `ros-humble-rtabmap-ros` in output

**Step 3: Commit**

```bash
git add docker/Dockerfile
git commit -m "feat(slam): add rtabmap-ros packages to Docker image"
```

---

## Task 2: Create Odometry Publisher Node

The robot needs wheel odometry to provide the odom→base_footprint transform. This node subscribes to `/cmd_vel` and integrates velocity to estimate pose (dead reckoning).

**Files:**
- Create: `ros2_nodes/odom_publisher/package.xml`
- Create: `ros2_nodes/odom_publisher/setup.py`
- Create: `ros2_nodes/odom_publisher/odom_publisher/__init__.py`
- Create: `ros2_nodes/odom_publisher/odom_publisher/odom_publisher_node.py`

**Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>odom_publisher</name>
  <version>0.1.0</version>
  <description>Wheel odometry publisher for LanderPi</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2_ros</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Step 2: Create setup.py**

```python
from setuptools import setup

package_name = 'odom_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Wheel odometry publisher for LanderPi',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_publisher_node = odom_publisher.odom_publisher_node:main',
        ],
    },
)
```

**Step 3: Create __init__.py**

```python
# odom_publisher package
```

**Step 4: Create odom_publisher_node.py**

```python
#!/usr/bin/env python3
"""
Wheel odometry publisher for LanderPi.

Subscribes to /cmd_vel and integrates velocity commands to estimate robot pose.
Publishes nav_msgs/Odometry and broadcasts odom->base_footprint transform.

This is dead-reckoning odometry - accuracy degrades over time.
For better accuracy, use laser odometry (rf2o) or sensor fusion (EKF).
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert Euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return {
        'w': cy * cp * cr + sy * sp * sr,
        'x': cy * cp * sr - sy * sp * cr,
        'y': sy * cp * sr + cy * sp * cr,
        'z': sy * cp * cr - cy * sp * sr,
    }


class OdomPublisher(Node):
    """Publishes odometry from cmd_vel integration."""

    # Covariance matrices for odometry
    POSE_COVARIANCE = [
        1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1e3,
    ]

    TWIST_COVARIANCE = [
        1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1e3,
    ]

    def __init__(self):
        super().__init__('odom_publisher')

        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 50.0)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        publish_rate = self.get_parameter('publish_rate').value

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.last_time = self.get_clock().now()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

        # Timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_odom)

        self.get_logger().info(
            f'Odom publisher started: {self.odom_frame} -> {self.base_frame}'
        )

    def cmd_vel_callback(self, msg: Twist):
        """Store latest velocity command."""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vtheta = msg.angular.z

    def publish_odom(self):
        """Integrate velocity and publish odometry."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Integrate velocity (2D kinematics)
        # For mecanum wheels, vy contributes to sideways motion
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_theta = self.vtheta * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Create quaternion from yaw
        q = euler_to_quaternion(0.0, 0.0, self.theta)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q['x']
        odom.pose.pose.orientation.y = q['y']
        odom.pose.pose.orientation.z = q['z']
        odom.pose.pose.orientation.w = q['w']
        odom.pose.covariance = self.POSE_COVARIANCE

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vtheta
        odom.twist.covariance = self.TWIST_COVARIANCE

        self.odom_pub.publish(odom)

        # Broadcast TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q['x']
            t.transform.rotation.y = q['y']
            t.transform.rotation.z = q['z']
            t.transform.rotation.w = q['w']
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 5: Commit**

```bash
git add ros2_nodes/odom_publisher/
git commit -m "feat(slam): add wheel odometry publisher node"
```

---

## Task 3: Create Static TF Publisher Node

Publishes the static transform from `base_footprint` to `laser_frame`. The lidar is mounted at approximately (0.1, 0, 0.15) relative to robot center.

**Files:**
- Create: `ros2_nodes/static_tf_publisher/package.xml`
- Create: `ros2_nodes/static_tf_publisher/setup.py`
- Create: `ros2_nodes/static_tf_publisher/static_tf_publisher/__init__.py`
- Create: `ros2_nodes/static_tf_publisher/static_tf_publisher/static_tf_node.py`

**Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>static_tf_publisher</name>
  <version>0.1.0</version>
  <description>Static TF publisher for LanderPi sensor frames</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>tf2_ros</depend>
  <depend>geometry_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Step 2: Create setup.py**

```python
from setuptools import setup

package_name = 'static_tf_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Static TF publisher for LanderPi sensor frames',
    license='MIT',
    entry_points={
        'console_scripts': [
            'static_tf_node = static_tf_publisher.static_tf_node:main',
        ],
    },
)
```

**Step 3: Create __init__.py**

```python
# static_tf_publisher package
```

**Step 4: Create static_tf_node.py**

```python
#!/usr/bin/env python3
"""
Static TF publisher for LanderPi sensor frames.

Publishes static transforms for sensors that don't move relative to base_footprint:
- base_footprint -> laser_frame (lidar)
- base_footprint -> camera_link (depth camera, if needed)

Measurements should be verified on actual robot.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class StaticTFPublisher(Node):
    """Publishes static transforms for LanderPi sensors."""

    def __init__(self):
        super().__init__('static_tf_publisher')

        # Parameters for lidar position (meters, relative to base_footprint)
        # base_footprint is on the ground, centered between wheels
        # lidar is mounted on top, slightly forward
        self.declare_parameter('lidar_x', 0.0)  # Forward from center
        self.declare_parameter('lidar_y', 0.0)      # Left-right (centered)
        self.declare_parameter('lidar_z', 0.15)     # Height above ground
        self.declare_parameter('lidar_frame', 'laser_frame')
        self.declare_parameter('base_frame', 'base_footprint')

        lidar_x = self.get_parameter('lidar_x').value
        lidar_y = self.get_parameter('lidar_y').value
        lidar_z = self.get_parameter('lidar_z').value
        lidar_frame = self.get_parameter('lidar_frame').value
        base_frame = self.get_parameter('base_frame').value

        # Create broadcaster
        self.broadcaster = StaticTransformBroadcaster(self)

        # Publish lidar transform
        lidar_tf = TransformStamped()
        lidar_tf.header.stamp = self.get_clock().now().to_msg()
        lidar_tf.header.frame_id = base_frame
        lidar_tf.child_frame_id = lidar_frame
        lidar_tf.transform.translation.x = lidar_x
        lidar_tf.transform.translation.y = lidar_y
        lidar_tf.transform.translation.z = lidar_z
        # No rotation - lidar is level with robot
        lidar_tf.transform.rotation.x = 0.0
        lidar_tf.transform.rotation.y = 0.0
        lidar_tf.transform.rotation.z = 0.0
        lidar_tf.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(lidar_tf)

        self.get_logger().info(
            f'Published static TF: {base_frame} -> {lidar_frame} '
            f'at ({lidar_x}, {lidar_y}, {lidar_z})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 5: Commit**

```bash
git add ros2_nodes/static_tf_publisher/
git commit -m "feat(slam): add static TF publisher for sensor frames"
```

---

## Task 4: Create SLAM Configuration Files

**Files:**
- Create: `config/slam/rtabmap_lidar.yaml`
- Create: `config/slam/nav2_params.yaml`

**Step 1: Create rtabmap_lidar.yaml**

```yaml
# rtabmap configuration for lidar-only SLAM
# Reference: https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_launch/launch/rtabmap.launch.py

rtabmap:
  ros__parameters:
    # Frame configuration
    frame_id: base_footprint
    odom_frame_id: odom
    map_frame_id: map

    # Subscribe to lidar only (no RGB-D)
    subscribe_scan: true
    subscribe_depth: false
    subscribe_rgb: false
    subscribe_rgbd: false

    # SLAM mode (vs localization)
    # Set to true when loading existing map
    localization: false

    # 2D SLAM settings
    Reg/Strategy: '1'        # 1=ICP (scan matching)
    Reg/Force3DoF: 'true'    # Force 2D (x, y, theta only)

    # Grid map from lidar
    Grid/Sensor: '0'         # 0=laser scan
    Grid/FromDepth: 'false'  # Don't use depth camera
    Grid/RangeMin: '0.2'     # Min lidar range (meters)
    Grid/RangeMax: '12.0'    # Max lidar range (meters)
    Grid/CellSize: '0.05'    # Map resolution (5cm)

    # Memory management
    Mem/IncrementalMemory: 'true'
    Mem/InitWMWithAllNodes: 'false'

    # Loop closure
    RGBD/ProximityBySpace: 'true'
    RGBD/AngularUpdate: '0.05'   # radians
    RGBD/LinearUpdate: '0.05'    # meters

    # ICP settings for scan matching
    Icp/VoxelSize: '0.05'
    Icp/MaxCorrespondenceDistance: '0.1'
    Icp/Iterations: '10'

    # Database
    database_path: ''  # Empty = use default ~/.ros/rtabmap.db

    # QoS
    qos_scan: 2  # Best effort
```

**Step 2: Create nav2_params.yaml**

```yaml
# Nav2 parameters for LanderPi
# Based on nav2_bringup defaults with adjustments for mecanum drive

amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: base_footprint
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: map
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 12.0
    laser_min_range: 0.2
    laser_model_type: likelihood_field
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: odom
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: nav2_amcl::OmniMotionModel
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: odom
    bt_loop_duration: 10
    default_server_timeout: 20
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001  # Enable for mecanum
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: progress_checker
    goal_checker_plugins: [goal_checker]
    controller_plugins: [FollowPath]

    progress_checker:
      plugin: nav2_controller::SimpleProgressChecker
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: nav2_controller::SimpleGoalChecker
      xy_goal_tolerance: 0.1
      yaw_goal_tolerance: 0.1
      stateful: true

    FollowPath:
      plugin: dwb_core::DWBLocalPlanner
      debug_trajectory_details: true
      min_vel_x: -0.2
      max_vel_x: 0.3
      min_vel_y: -0.2  # Mecanum: enable lateral motion
      max_vel_y: 0.2
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.3
      min_speed_theta: 0.0
      acc_lim_x: 1.0
      acc_lim_y: 1.0
      acc_lim_theta: 2.0
      decel_lim_x: -1.0
      decel_lim_y: -1.0
      decel_lim_theta: -2.0
      vx_samples: 20
      vy_samples: 10
      vtheta_samples: 20
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.1
      trans_stopped_velocity: 0.1
      short_circuit_trajectory_evaluation: true
      stateful: true
      critics:
        - RotateToGoal
        - Oscillation
        - BaseObstacle
        - GoalAlign
        - PathAlign
        - PathDist
        - GoalDist
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_footprint
      use_sim_time: false
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      footprint: '[[0.15, 0.12], [0.15, -0.12], [-0.15, -0.12], [-0.15, 0.12]]'
      plugins: [voxel_layer, inflation_layer]
      voxel_layer:
        plugin: nav2_costmap_2d::VoxelLayer
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: LaserScan
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        cost_scaling_factor: 3.0
        inflation_radius: 0.3
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: false
      footprint: '[[0.15, 0.12], [0.15, -0.12], [-0.15, -0.12], [-0.15, 0.12]]'
      resolution: 0.05
      track_unknown_space: true
      plugins: [static_layer, obstacle_layer, inflation_layer]
      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: nav2_costmap_2d::ObstacleLayer
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: LaserScan
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        cost_scaling_factor: 3.0
        inflation_radius: 0.3
      always_send_full_costmap: true

planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 20.0
    planner_plugins: [GridBased]
    GridBased:
      plugin: nav2_navfn_planner/NavfnPlanner
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

behavior_server:
  ros__parameters:
    use_sim_time: false
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: [spin, backup, wait]
    spin:
      plugin: nav2_behaviors/Spin
    backup:
      plugin: nav2_behaviors/BackUp
    wait:
      plugin: nav2_behaviors/Wait
    global_frame: odom
    robot_base_frame: base_footprint
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

**Step 3: Commit**

```bash
git add config/slam/
git commit -m "feat(slam): add rtabmap and Nav2 configuration files"
```

---

## Task 5: Create SLAM Launch File

**Files:**
- Create: `ros2_nodes/slam_navigation/package.xml`
- Create: `ros2_nodes/slam_navigation/setup.py`
- Create: `ros2_nodes/slam_navigation/slam_navigation/__init__.py`
- Create: `ros2_nodes/slam_navigation/launch/slam.launch.py`

**Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>slam_navigation</name>
  <version>0.1.0</version>
  <description>SLAM and navigation launch files for LanderPi</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>MIT</license>

  <exec_depend>rtabmap_ros</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Step 2: Create setup.py**

```python
import os
from glob import glob
from setuptools import setup

package_name = 'slam_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='SLAM and navigation launch files for LanderPi',
    license='MIT',
)
```

**Step 3: Create resource file and __init__.py**

```bash
mkdir -p ros2_nodes/slam_navigation/resource
touch ros2_nodes/slam_navigation/resource/slam_navigation
```

```python
# slam_navigation package
```

**Step 4: Create slam.launch.py**

```python
"""
SLAM launch file for LanderPi.

Launches rtabmap in mapping or localization mode with Nav2 for path planning.

Usage:
  # Mapping mode (build new map)
  ros2 launch slam_navigation slam.launch.py mode:=mapping

  # Localization mode (use existing map)
  ros2 launch slam_navigation slam.launch.py mode:=localization
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='SLAM mode: mapping (build map) or localization (use existing)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Config paths (mounted in Docker at /ros2_ws/config)
    rtabmap_config = '/ros2_ws/config/slam/rtabmap_lidar.yaml'
    nav2_config = '/ros2_ws/config/slam/nav2_params.yaml'

    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Determine if localization mode
    is_localization = PythonExpression(["'", mode, "' == 'localization'"])

    # rtabmap node
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_config,
            {
                'use_sim_time': use_sim_time,
                # Override localization param based on mode
                'localization': is_localization,
            }
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
        # -d flag deletes database on start (only in mapping mode)
        arguments=[],
    )

    # Nav2 bringup
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_config,
        }.items(),
    )

    return LaunchDescription([
        mode_arg,
        use_sim_time_arg,
        rtabmap_node,
        nav2_launch,
    ])
```

**Step 5: Commit**

```bash
git add ros2_nodes/slam_navigation/
git commit -m "feat(slam): add SLAM navigation launch package"
```

---

## Task 6: Update Main Launch File

Add odom_publisher, static_tf_publisher, and enable_slam flag to the main launch file.

**Files:**
- Modify: `ros2_nodes/cmd_vel_bridge/launch/landerpi.launch.py`
- Modify: `docker/docker-compose.yml`

**Step 1: Add new nodes to landerpi.launch.py**

Add after the battery_monitor_node definition (around line 177):

```python
    # Odometry publisher - integrates cmd_vel to estimate pose
    odom_publisher_node = Node(
        package='odom_publisher',
        executable='odom_publisher_node',
        name='odom_publisher',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_tf': True,
            'publish_rate': 50.0,
        }],
    )

    # Static TF publisher - sensor frame transforms
    static_tf_node = Node(
        package='static_tf_publisher',
        executable='static_tf_node',
        name='static_tf_publisher',
        output='screen',
        parameters=[{
            'lidar_x': 0.0,
            'lidar_y': 0.0,
            'lidar_z': 0.15,
            'lidar_frame': 'laser_frame',
            'base_frame': 'base_footprint',
        }],
    )
```

Add to the LaunchDescription return list:

```python
        odom_publisher_node,
        static_tf_node,
```

**Step 2: Add new packages to docker-compose.yml**

Update the PACKAGES line in docker-compose.yml command section:

```yaml
        PACKAGES='ros_robot_controller_msgs ros_robot_controller cmd_vel_bridge arm_controller lidar_driver battery_monitor depth_stats yolo_detector obstacle_fusion sensor_bridge odom_publisher static_tf_publisher'
```

**Step 3: Add volume mount for slam_navigation**

Add to volumes section in docker-compose.yml:

```yaml
      - ../ros2_nodes/odom_publisher:/ros2_ws/src/odom_publisher:ro
      - ../ros2_nodes/static_tf_publisher:/ros2_ws/src/static_tf_publisher:ro
      - ../ros2_nodes/slam_navigation:/ros2_ws/src/slam_navigation:ro
```

**Step 4: Commit**

```bash
git add ros2_nodes/cmd_vel_bridge/launch/landerpi.launch.py docker/docker-compose.yml
git commit -m "feat(slam): integrate odom and static TF publishers into main launch"
```

---

## Task 7: Create SLAM Deployment CLI

**Files:**
- Create: `deploy_slam.py`

**Step 1: Create deploy_slam.py**

```python
#!/usr/bin/env python3
"""
Deploy and manage SLAM navigation on LanderPi robot.

Commands:
  deploy    - Upload files and rebuild Docker image with rtabmap
  map       - Start mapping mode (drive robot to build map)
  localize  - Start localization mode (use existing map)
  save      - Save current map to file
  stop      - Stop SLAM nodes
  status    - Check SLAM status
  navigate  - Send navigation goal
"""

import json
import sys
import typer
from pathlib import Path
from fabric import Connection
from rich.console import Console
from rich.panel import Panel

app = typer.Typer(help="SLAM Navigation for LanderPi")
console = Console()


def get_config():
    """Load connection config from config.json."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        with open(config_path) as f:
            return json.load(f)
    return {}


def get_connection(host: str, user: str, password: str) -> Connection:
    """Create SSH connection."""
    config = get_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not all([host, user, password]):
        console.print("[red]Missing connection details. Use --host, --user, --password or config.json[/red]")
        raise typer.Exit(1)

    return Connection(host, user=user, connect_kwargs={"password": password})


@app.command()
def deploy(
    host: str = typer.Option(None, help="Robot IP address"),
    user: str = typer.Option(None, help="SSH username"),
    password: str = typer.Option(None, help="SSH password"),
):
    """Deploy SLAM stack to robot (uploads files, rebuilds Docker image)."""
    console.print(Panel("Deploying SLAM Navigation Stack", title="LanderPi SLAM"))

    with get_connection(host, user, password) as conn:
        # First run the standard deploy
        console.print("\n[bold]Running base deployment...[/bold]")
        result = conn.local("uv run python deploy_ros2_stack.py deploy", warn=True)

        if not result.ok:
            console.print("[red]Base deployment failed[/red]")
            raise typer.Exit(1)

        # Rebuild Docker image (includes rtabmap now)
        console.print("\n[bold]Rebuilding Docker image with rtabmap...[/bold]")
        console.print("[yellow]This may take several minutes on first run[/yellow]")
        result = conn.run(
            "cd ~/landerpi/docker && docker compose build",
            warn=True
        )

        if result.ok:
            console.print("[green]SLAM stack deployed successfully![/green]")
            console.print("\nNext steps:")
            console.print("  1. Start mapping: uv run python deploy_slam.py map")
            console.print("  2. Drive robot around to build map")
            console.print("  3. Save map: uv run python deploy_slam.py save")
        else:
            console.print("[red]Docker build failed. Check logs.[/red]")


@app.command()
def map(
    host: str = typer.Option(None, help="Robot IP address"),
    user: str = typer.Option(None, help="SSH username"),
    password: str = typer.Option(None, help="SSH password"),
    delete_db: bool = typer.Option(True, "--delete-db/--keep-db", help="Delete existing map database"),
):
    """Start SLAM in mapping mode (build new map)."""
    console.print(Panel("Starting SLAM Mapping Mode", title="LanderPi SLAM"))

    with get_connection(host, user, password) as conn:
        # Stop any existing SLAM
        console.print("[dim]Stopping existing SLAM nodes...[/dim]")
        conn.run(
            "docker exec landerpi-ros2 bash -c 'pkill -f rtabmap || true'",
            warn=True, hide=True
        )

        if delete_db:
            console.print("[dim]Deleting existing map database...[/dim]")
            conn.run("rm -f ~/.ros/rtabmap.db", warn=True, hide=True)

        # Start rtabmap in mapping mode
        console.print("[bold]Starting rtabmap in mapping mode...[/bold]")
        cmd = """docker exec -d landerpi-ros2 bash -c '
            source /opt/ros/humble/setup.bash && \
            source /ros2_ws/install/setup.bash && \
            ros2 run rtabmap_slam rtabmap \
                --ros-args \
                -p subscribe_scan:=true \
                -p subscribe_depth:=false \
                -p frame_id:=base_footprint \
                -p odom_frame_id:=odom \
                -p Reg/Strategy:=1 \
                -p Reg/Force3DoF:=true \
                -p Grid/Sensor:=0 \
                -p Grid/FromDepth:=false \
                -r scan:=/scan \
                -r odom:=/odom
        '"""
        result = conn.run(cmd, warn=True)

        if result.ok:
            console.print("[green]Mapping started![/green]")
            console.print("\nDrive the robot around to build the map.")
            console.print("Use keyboard teleop or voice commands.")
            console.print("\nWhen done, save the map:")
            console.print("  uv run python deploy_slam.py save")
        else:
            console.print("[red]Failed to start mapping[/red]")


@app.command()
def localize(
    host: str = typer.Option(None, help="Robot IP address"),
    user: str = typer.Option(None, help="SSH username"),
    password: str = typer.Option(None, help="SSH password"),
):
    """Start SLAM in localization mode (use existing map)."""
    console.print(Panel("Starting SLAM Localization Mode", title="LanderPi SLAM"))

    with get_connection(host, user, password) as conn:
        # Check if map exists
        result = conn.run("test -f ~/.ros/rtabmap.db", warn=True, hide=True)
        if not result.ok:
            console.print("[red]No map found at ~/.ros/rtabmap.db[/red]")
            console.print("Run mapping first: uv run python deploy_slam.py map")
            raise typer.Exit(1)

        # Stop any existing SLAM
        conn.run(
            "docker exec landerpi-ros2 bash -c 'pkill -f rtabmap || true'",
            warn=True, hide=True
        )

        # Start rtabmap in localization mode
        console.print("[bold]Starting rtabmap in localization mode...[/bold]")
        cmd = """docker exec -d landerpi-ros2 bash -c '
            source /opt/ros/humble/setup.bash && \
            source /ros2_ws/install/setup.bash && \
            ros2 run rtabmap_slam rtabmap \
                --ros-args \
                -p subscribe_scan:=true \
                -p subscribe_depth:=false \
                -p frame_id:=base_footprint \
                -p odom_frame_id:=odom \
                -p localization:=true \
                -p Reg/Strategy:=1 \
                -p Reg/Force3DoF:=true \
                -r scan:=/scan \
                -r odom:=/odom
        '"""
        result = conn.run(cmd, warn=True)

        if result.ok:
            console.print("[green]Localization started![/green]")
            console.print("Robot is now localizing in the existing map.")
        else:
            console.print("[red]Failed to start localization[/red]")


@app.command()
def save(
    host: str = typer.Option(None, help="Robot IP address"),
    user: str = typer.Option(None, help="SSH username"),
    password: str = typer.Option(None, help="SSH password"),
    name: str = typer.Option("landerpi_map", help="Map name (without extension)"),
):
    """Save current map to file."""
    console.print(Panel(f"Saving Map: {name}", title="LanderPi SLAM"))

    with get_connection(host, user, password) as conn:
        # Export map to PGM/YAML format for Nav2
        console.print("[bold]Exporting map...[/bold]")

        # Create maps directory
        conn.run("mkdir -p ~/landerpi/maps", warn=True)

        # Use rtabmap-export to save occupancy grid
        cmd = f"""docker exec landerpi-ros2 bash -c '
            source /opt/ros/humble/setup.bash && \
            rtabmap-export --poses ~/.ros/rtabmap.db && \
            ros2 run nav2_map_server map_saver_cli -f /maps/{name}
        '"""

        result = conn.run(cmd, warn=True)

        if result.ok:
            console.print(f"[green]Map saved to ~/landerpi/maps/{name}[/green]")
            console.print("Files created:")
            console.print(f"  - {name}.pgm (occupancy grid)")
            console.print(f"  - {name}.yaml (metadata)")
        else:
            console.print("[yellow]Map export may have partially succeeded[/yellow]")
            console.print("Check ~/.ros/rtabmap.db exists and contains map data")


@app.command()
def stop(
    host: str = typer.Option(None, help="Robot IP address"),
    user: str = typer.Option(None, help="SSH username"),
    password: str = typer.Option(None, help="SSH password"),
):
    """Stop SLAM nodes."""
    with get_connection(host, user, password) as conn:
        console.print("[bold]Stopping SLAM nodes...[/bold]")
        conn.run(
            "docker exec landerpi-ros2 bash -c 'pkill -f rtabmap || true; pkill -f nav2 || true'",
            warn=True
        )
        console.print("[green]SLAM stopped[/green]")


@app.command()
def status(
    host: str = typer.Option(None, help="Robot IP address"),
    user: str = typer.Option(None, help="SSH username"),
    password: str = typer.Option(None, help="SSH password"),
):
    """Check SLAM status."""
    with get_connection(host, user, password) as conn:
        console.print(Panel("SLAM Status", title="LanderPi"))

        # Check rtabmap process
        result = conn.run(
            "docker exec landerpi-ros2 pgrep -f rtabmap",
            warn=True, hide=True
        )
        rtabmap_running = result.ok

        # Check for map database
        result = conn.run("test -f ~/.ros/rtabmap.db", warn=True, hide=True)
        map_exists = result.ok

        # Check TF tree
        result = conn.run(
            "docker exec landerpi-ros2 bash -c 'source /opt/ros/humble/setup.bash && ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -5'",
            warn=True, hide=True
        )
        tf_ok = "Transform" in result.stdout if result.ok else False

        # Check topics
        result = conn.run(
            "docker exec landerpi-ros2 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null | grep -E \"(scan|odom|map)\"'",
            warn=True, hide=True
        )
        topics = result.stdout.strip().split('\n') if result.ok else []

        # Display status
        console.print(f"rtabmap running: {'[green]Yes[/green]' if rtabmap_running else '[red]No[/red]'}")
        console.print(f"Map database: {'[green]Exists[/green]' if map_exists else '[yellow]Not found[/yellow]'}")
        console.print(f"TF map->base: {'[green]OK[/green]' if tf_ok else '[red]Missing[/red]'}")
        console.print(f"\nTopics:")
        for topic in topics:
            if topic:
                console.print(f"  {topic}")


@app.command()
def navigate(
    host: str = typer.Option(None, help="Robot IP address"),
    user: str = typer.Option(None, help="SSH username"),
    password: str = typer.Option(None, help="SSH password"),
    x: float = typer.Argument(..., help="Target X coordinate (meters)"),
    y: float = typer.Argument(..., help="Target Y coordinate (meters)"),
    theta: float = typer.Option(0.0, help="Target orientation (radians)"),
):
    """Send navigation goal to Nav2."""
    console.print(Panel(f"Navigating to ({x}, {y}, {theta})", title="LanderPi Nav"))

    with get_connection(host, user, password) as conn:
        # Convert theta to quaternion (simplified for 2D)
        import math
        qz = math.sin(theta / 2)
        qw = math.cos(theta / 2)

        cmd = f"""docker exec landerpi-ros2 bash -c '
            source /opt/ros/humble/setup.bash && \
            source /ros2_ws/install/setup.bash && \
            ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{{
                pose: {{
                    header: {{frame_id: map}},
                    pose: {{
                        position: {{x: {x}, y: {y}, z: 0.0}},
                        orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}
                    }}
                }}
            }}"
        '"""

        result = conn.run(cmd, warn=True)
        if result.ok:
            console.print("[green]Navigation goal sent![/green]")
        else:
            console.print("[red]Failed to send goal. Is Nav2 running?[/red]")


if __name__ == "__main__":
    app()
```

**Step 2: Commit**

```bash
git add deploy_slam.py
git commit -m "feat(slam): add SLAM deployment CLI"
```

---

## Task 8: Create Validation Test

**Files:**
- Create: `validation/test_slam.py`

**Step 1: Create test_slam.py**

```python
#!/usr/bin/env python3
"""
SLAM validation tests for LanderPi.

Tests:
- TF tree connectivity (map -> odom -> base_footprint -> laser_frame)
- Topic availability (/scan, /odom, /map)
- rtabmap responsiveness
- Navigation goal acceptance
"""

import json
import sys
import time
from pathlib import Path
from typing import Optional

import typer
from fabric import Connection
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

app = typer.Typer(help="SLAM Validation Tests")
console = Console()


def load_config() -> dict:
    """Load connection config from config.json."""
    config_path = Path(__file__).parent.parent / "config.json"
    if config_path.exists():
        return json.loads(config_path.read_text())
    return {}


class SLAMValidator:
    """Validates SLAM system components."""

    def __init__(self, host: str, user: str, password: str):
        self.conn = Connection(host, user=user, connect_kwargs={"password": password})

    def docker_exec(self, cmd: str, timeout: int = 10) -> tuple[bool, str]:
        """Execute command in Docker container."""
        full_cmd = f"""docker exec landerpi-ros2 bash -c '
            source /opt/ros/humble/setup.bash && \
            source /ros2_ws/install/setup.bash 2>/dev/null || true && \
            {cmd}
        '"""
        try:
            result = self.conn.run(full_cmd, warn=True, hide=True, timeout=timeout)
            return result.ok, result.stdout
        except Exception as e:
            return False, str(e)

    def check_topic(self, topic: str) -> tuple[bool, str]:
        """Check if a topic is being published."""
        ok, output = self.docker_exec(
            f"timeout 2 ros2 topic echo {topic} --once 2>&1 | head -5"
        )
        if ok and "data:" in output.lower() or "header:" in output.lower():
            return True, "Publishing"
        return False, "Not publishing or timeout"

    def check_tf(self, parent: str, child: str) -> tuple[bool, str]:
        """Check TF transform availability."""
        ok, output = self.docker_exec(
            f"timeout 3 ros2 run tf2_ros tf2_echo {parent} {child} 2>&1 | head -3"
        )
        if ok and "Translation" in output:
            return True, "OK"
        if "Lookup would require extrapolation" in output:
            return True, "Available (extrapolation warning)"
        return False, "Not available"

    def check_rtabmap(self) -> tuple[bool, str]:
        """Check if rtabmap is running."""
        ok, output = self.docker_exec("pgrep -f 'rtabmap' | head -1")
        if ok and output.strip():
            return True, f"PID {output.strip()}"
        return False, "Not running"

    def check_nav2(self) -> tuple[bool, str]:
        """Check if Nav2 is available."""
        ok, output = self.docker_exec(
            "ros2 action list 2>/dev/null | grep navigate_to_pose"
        )
        if ok and "navigate_to_pose" in output:
            return True, "Available"
        return False, "Not available"


@app.command()
def check(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Run comprehensive SLAM system check."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not host or not user:
        console.print("[red]Error: host and user required[/red]")
        sys.exit(1)

    console.print(Panel(f"SLAM Validation: {host}", title="LanderPi"))

    validator = SLAMValidator(host, user, password)

    # Results table
    table = Table(title="SLAM System Status")
    table.add_column("Component", style="cyan")
    table.add_column("Status", style="green")
    table.add_column("Details")

    all_ok = True

    # Check topics
    console.print("\n[bold]Checking topics...[/bold]")
    for topic in ["/scan", "/odom", "/map"]:
        ok, detail = validator.check_topic(topic)
        status = "[green]OK[/green]" if ok else "[red]FAIL[/red]"
        table.add_row(f"Topic {topic}", status, detail)
        if not ok and topic != "/map":  # /map may not exist yet
            all_ok = False

    # Check TF tree
    console.print("[bold]Checking TF tree...[/bold]")
    tf_checks = [
        ("odom", "base_footprint"),
        ("base_footprint", "laser_frame"),
        ("map", "odom"),
    ]
    for parent, child in tf_checks:
        ok, detail = validator.check_tf(parent, child)
        status = "[green]OK[/green]" if ok else "[yellow]WARN[/yellow]"
        table.add_row(f"TF {parent}->{child}", status, detail)
        if not ok and parent != "map":  # map->odom requires rtabmap
            all_ok = False

    # Check processes
    console.print("[bold]Checking processes...[/bold]")
    ok, detail = validator.check_rtabmap()
    status = "[green]OK[/green]" if ok else "[yellow]Not started[/yellow]"
    table.add_row("rtabmap", status, detail)

    ok, detail = validator.check_nav2()
    status = "[green]OK[/green]" if ok else "[yellow]Not started[/yellow]"
    table.add_row("Nav2", status, detail)

    console.print(table)

    if all_ok:
        console.print("\n[green]Core SLAM prerequisites OK![/green]")
        console.print("Start mapping with: uv run python deploy_slam.py map")
    else:
        console.print("\n[yellow]Some components need attention.[/yellow]")
        console.print("Ensure ROS2 stack is running: uv run python deploy_ros2_stack.py deploy")


@app.command()
def topics(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """List all SLAM-related topics."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    validator = SLAMValidator(host, user, password)
    ok, output = validator.docker_exec("ros2 topic list | grep -E '(scan|odom|map|tf|cmd_vel)'")

    if ok:
        console.print("[bold]SLAM-related topics:[/bold]")
        for line in output.strip().split('\n'):
            if line:
                console.print(f"  {line}")
    else:
        console.print("[red]Failed to list topics[/red]")


@app.command()
def tf_tree(
    host: Optional[str] = typer.Option(None, help="Robot IP address"),
    user: Optional[str] = typer.Option(None, help="SSH Username"),
    password: Optional[str] = typer.Option(None, help="SSH Password"),
):
    """Display TF tree."""
    config = load_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    validator = SLAMValidator(host, user, password)
    ok, output = validator.docker_exec("ros2 run tf2_tools view_frames --help 2>&1; ros2 run tf2_ros tf2_monitor 2>&1 | head -20", timeout=5)

    # Simpler approach - just check frames
    ok, output = validator.docker_exec("ros2 topic echo /tf_static --once 2>&1 | head -20")
    console.print("[bold]TF Static frames:[/bold]")
    console.print(output if ok else "No static transforms")

    ok, output = validator.docker_exec("ros2 topic echo /tf --once 2>&1 | head -20")
    console.print("\n[bold]TF Dynamic frames:[/bold]")
    console.print(output if ok else "No dynamic transforms")


if __name__ == "__main__":
    app()
```

**Step 2: Commit**

```bash
git add validation/test_slam.py
git commit -m "feat(slam): add SLAM validation test script"
```

---

## Task 9: Update CLAUDE.md Documentation

**Files:**
- Modify: `CLAUDE.md`

**Step 1: Add SLAM commands section**

Add after the voice control section:

```markdown
# SLAM and Navigation

```bash
# Deploy SLAM stack (first time - rebuilds Docker with rtabmap)
uv run python deploy_slam.py deploy

# Start mapping mode (drive robot to build map)
uv run python deploy_slam.py map

# Save map after mapping
uv run python deploy_slam.py save --name my_apartment

# Start localization mode (use existing map)
uv run python deploy_slam.py localize

# Check SLAM status
uv run python deploy_slam.py status

# Navigate to coordinates
uv run python deploy_slam.py navigate 1.0 2.0 --theta 0.0

# Stop SLAM
uv run python deploy_slam.py stop

# Run SLAM validation tests
uv run python validation/test_slam.py check
uv run python validation/test_slam.py topics
uv run python validation/test_slam.py tf-tree
```

**SLAM Architecture:**
```
map (rtabmap)
 └── odom (odom_publisher)
      └── base_footprint (robot base)
           └── laser_frame (lidar, static TF)
```

**Key Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | LaserScan | Lidar data |
| `/odom` | Odometry | Wheel odometry |
| `/map` | OccupancyGrid | SLAM map |
| `/tf` | TFMessage | Transform tree |
```

**Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: add SLAM commands to CLAUDE.md"
```

---

## Task 10: End-to-End Integration Test

**Step 1: Deploy to robot**

```bash
uv run python deploy_ros2_stack.py deploy
```

**Step 2: Check prerequisites**

```bash
uv run python validation/test_slam.py check
```

Expected: `/scan` and `/odom` topics publishing, TF odom->base_footprint and base_footprint->laser_frame available.

**Step 3: Start mapping**

```bash
uv run python deploy_slam.py map
```

**Step 4: Verify mapping started**

```bash
uv run python deploy_slam.py status
```

Expected: rtabmap running, map->odom TF available.

**Step 5: Drive robot around**

Use voice control or manual commands to move robot through environment.

**Step 6: Save map**

```bash
uv run python deploy_slam.py save --name test_map
```

**Step 7: Test navigation**

```bash
uv run python deploy_slam.py navigate 0.5 0.0
```

**Step 8: Final commit**

```bash
git add -A
git commit -m "feat(slam): complete Phase 1 SLAM navigation implementation"
```

---

## Success Criteria

- [ ] Docker image builds with rtabmap
- [ ] `/odom` topic publishes at 50Hz
- [ ] TF tree: map -> odom -> base_footprint -> laser_frame
- [ ] `deploy_slam.py map` starts mapping mode
- [ ] Robot can build map by driving around
- [ ] `deploy_slam.py save` exports map
- [ ] `deploy_slam.py localize` works with saved map
- [ ] `deploy_slam.py navigate` sends goals to Nav2
