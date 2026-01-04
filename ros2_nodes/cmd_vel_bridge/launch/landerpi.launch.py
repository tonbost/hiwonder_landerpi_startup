"""
LanderPi unified ROS2 launch file.

Launches all LanderPi nodes:
- ros_robot_controller (vendor) - STM32 communication
- cmd_vel_bridge - Twist to motor commands
- arm_controller - Arm servo control
- lidar_driver - LD19/MS200 lidar
- battery_monitor - Battery voltage monitoring
- camera_driver - Aurora 930 (if workspace available)
- depth_stats - Depth camera stats for exploration (if camera available)
- yolo_detector - YOLOv11 object detection (if camera available)
- obstacle_fusion - Fuses YOLO detections with lidar for hazard detection
"""

import json
import os
from pathlib import Path
from launch import LaunchDescription


def load_hazard_config():
    """Load hazard classes from JSON config file."""
    # Config is mounted at /ros2_ws/config in Docker
    config_paths = [
        Path('/ros2_ws/config/yolo_hazards.json'),
        Path(__file__).parent.parent.parent.parent / 'config' / 'yolo_hazards.json',
    ]

    for config_path in config_paths:
        if config_path.exists():
            try:
                with open(config_path) as f:
                    config = json.load(f)
                print(f"Loaded hazard config from: {config_path}")
                return config
            except Exception as e:
                print(f"Warning: Failed to load {config_path}: {e}")

    # Default fallback
    print("Warning: Using default hazard config (no yolo_hazards.json found)")
    return {
        'hazard_classes': ['person', 'dog', 'cat', 'cup', 'bottle', 'stop sign'],
        'hazard_distance': 2.5,
    }


from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Load hazard configuration from JSON
    hazard_config = load_hazard_config()

    # Declare arguments
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable lidar driver'
    )

    enable_arm_arg = DeclareLaunchArgument(
        'enable_arm',
        default_value='true',
        description='Enable arm controller'
    )

    enable_battery_arg = DeclareLaunchArgument(
        'enable_battery',
        default_value='true',
        description='Enable battery monitor'
    )

    # Auto-detect camera driver availability
    camera_launch_path = '/deptrum_ws/install/deptrum-ros-driver-aurora930/share/deptrum-ros-driver-aurora930/launch/aurora930_launch.py'
    camera_available = os.path.exists(camera_launch_path)

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true' if camera_available else 'false',
        description='Enable camera driver (requires deptrum_ws)'
    )

    enable_yolo_arg = DeclareLaunchArgument(
        'enable_yolo',
        default_value='true' if camera_available else 'false',
        description='Enable YOLO object detection (requires camera)'
    )

    enable_yolo_logging_arg = DeclareLaunchArgument(
        'enable_yolo_logging',
        default_value='false',
        description='Enable YOLO debug logging (saves images to ~/yolo_logs)'
    )

    enable_sensor_bridge_arg = DeclareLaunchArgument(
        'enable_sensor_bridge',
        default_value='true',
        description='Enable sensor bridge for low-latency topic reading via JSON files'
    )

    # Hailo is available when camera is available (server runs on host via systemd)
    # The bridge handles connection failures gracefully
    use_hailo_arg = DeclareLaunchArgument(
        'use_hailo',
        default_value='true' if camera_available else 'false',
        description='Use Hailo-8 accelerated YOLO (25-40 FPS vs 2-5 FPS on CPU)'
    )

    # Vendor node: ros_robot_controller
    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        name='ros_robot_controller',
        output='screen',
        parameters=[{
            'port': '/dev/ttyAMA0',
            'baudrate': 1000000,
        }],
    )

    # cmd_vel_bridge node
    cmd_vel_bridge_node = Node(
        package='cmd_vel_bridge',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'wheel_radius': 0.05,
            'wheel_base': 0.15,
            'track_width': 0.15,
            'max_wheel_rps': 3.0,
            'cmd_vel_timeout': 0.5,
        }],
    )

    # arm_controller node
    arm_controller_node = Node(
        package='arm_controller',
        executable='arm_controller_node',
        name='arm_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_arm')),
    )

    # lidar_driver node
    lidar_driver_node = Node(
        package='lidar_driver',
        executable='lidar_driver_node',
        name='lidar_driver',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        parameters=[{
            'port': '/dev/ttyUSB0',
            'port_fallback': '/dev/ttyUSB1',
            'baudrate': 230400,
            'frame_id': 'laser_frame',
        }],
    )

    # battery_monitor node
    battery_monitor_node = Node(
        package='battery_monitor',
        executable='battery_monitor_node',
        name='battery_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_battery')),
        parameters=[{
            'publish_rate': 1.0,
            'low_voltage_warning': 7.0,
            'critical_voltage': 6.6,
        }],
    )

    # sensor_bridge node - bridges ROS2 topics to JSON files for low-latency reading
    # Writes to /landerpi_data/*.json (shared volume with host)
    sensor_bridge_node = Node(
        package='sensor_bridge',
        executable='sensor_bridge_node',
        name='sensor_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_sensor_bridge')),
        parameters=[{
            'data_dir': '/landerpi_data',
            'lidar_throttle_hz': 20.0,
            'hazard_throttle_hz': 20.0,
            'depth_throttle_hz': 10.0,
            'battery_throttle_hz': 1.0,
        }],
    )

    # Camera driver (Aurora 930) - include vendor launch file
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path),
        condition=IfCondition(LaunchConfiguration('enable_camera')),
    ) if camera_available else LogInfo(msg='Camera driver not available (deptrum_ws not found)')

    # depth_stats node - processes depth images for exploration
    # Only useful when camera driver is available
    depth_stats_node = Node(
        package='depth_stats',
        executable='depth_stats_node',
        name='depth_stats',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_camera')),
        parameters=[{
            'center_width': 200,
            'center_height': 160,
            'min_valid_depth': 150,
            'max_valid_depth': 3000,
            'publish_rate': 10.0,
        }],
    ) if camera_available else LogInfo(msg='depth_stats skipped (no camera)')

    # yolo_detector node (CPU) - YOLOv11 object detection
    # Runs when enable_yolo=true AND use_hailo=false
    # Subscribes to /aurora/rgb/image_raw, publishes to /yolo/detections
    yolo_detector_cpu_node = Node(
        package='yolo_detector',
        executable='yolo_node',
        name='yolo_detector',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_yolo'), "' == 'true' and '",
            LaunchConfiguration('use_hailo'), "' == 'false'"
        ])),
        parameters=[{
            'model_path': 'yolo11n.pt',
            'confidence_threshold': 0.4,
            'device': 'cpu',
            'enable_logging': LaunchConfiguration('enable_yolo_logging'),
            'log_dir': '/yolo_logs',  # Mounted to ~/yolo_logs on host
            'log_all_frames': False,
            'max_log_images': 500,
        }],
    ) if camera_available else LogInfo(msg='yolo_detector skipped (no camera)')

    # yolo_hailo_bridge node - Connects to host Hailo inference server via ZeroMQ
    # Runs when enable_yolo=true AND use_hailo=true
    # Host server (hailo_inference_server.py) must be running on port 5555
    # 25-40 FPS vs 2-5 FPS on CPU
    yolo_detector_hailo_node = Node(
        package='yolo_hailo_bridge',
        executable='bridge_node',
        name='yolo_detector',  # Same name so obstacle_fusion works with either
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('enable_yolo'), "' == 'true' and '",
            LaunchConfiguration('use_hailo'), "' == 'true'"
        ])),
        parameters=[{
            'server_host': 'localhost',  # Host is accessible via localhost (network_mode: host)
            'server_port': 5555,
            'timeout_ms': 500,
        }],
    ) if camera_available else LogInfo(msg='yolo_hailo_bridge skipped (no camera)')

    # obstacle_fusion node - Fuses YOLO detections with lidar for hazards
    # Subscribes to /yolo/detections and /scan, publishes to /hazards
    # Hazard classes loaded from config/yolo_hazards.json
    obstacle_fusion_node = Node(
        package='obstacle_fusion',
        executable='fusion_node',
        name='obstacle_fusion',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_yolo')),
        parameters=[{
            'image_width': 640,
            'fov_horizontal': 60.0,
            'hazard_classes': hazard_config.get('hazard_classes', ['person', 'dog', 'cat']),
            'hazard_distance': hazard_config.get('hazard_distance', 2.5),
        }],
    ) if camera_available else LogInfo(msg='obstacle_fusion skipped (no camera)')

    return LaunchDescription([
        # Arguments
        enable_lidar_arg,
        enable_arm_arg,
        enable_battery_arg,
        enable_camera_arg,
        enable_yolo_arg,
        enable_yolo_logging_arg,
        use_hailo_arg,
        enable_sensor_bridge_arg,

        # Log startup
        LogInfo(msg='Starting LanderPi ROS2 stack...'),

        # Nodes
        ros_robot_controller_node,
        cmd_vel_bridge_node,
        arm_controller_node,
        lidar_driver_node,
        battery_monitor_node,
        sensor_bridge_node,
        camera_launch,
        depth_stats_node,
        yolo_detector_cpu_node,
        yolo_detector_hailo_node,
        obstacle_fusion_node,

        LogInfo(msg='LanderPi ROS2 stack launched'),
    ])
