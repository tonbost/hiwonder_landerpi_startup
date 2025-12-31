"""
LanderPi unified ROS2 launch file.

Launches all LanderPi nodes:
- ros_robot_controller (vendor) - STM32 communication
- cmd_vel_bridge - Twist to motor commands
- arm_controller - Arm servo control
- lidar_driver - LD19/MS200 lidar
- battery_monitor - Battery voltage monitoring
- camera_driver - Aurora 930 (if workspace available)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
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

    # Camera driver (Aurora 930) - include vendor launch file
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path),
        condition=IfCondition(LaunchConfiguration('enable_camera')),
    ) if camera_available else LogInfo(msg='Camera driver not available (deptrum_ws not found)')

    return LaunchDescription([
        # Arguments
        enable_lidar_arg,
        enable_arm_arg,
        enable_battery_arg,
        enable_camera_arg,

        # Log startup
        LogInfo(msg='Starting LanderPi ROS2 stack...'),

        # Nodes
        ros_robot_controller_node,
        cmd_vel_bridge_node,
        arm_controller_node,
        lidar_driver_node,
        battery_monitor_node,
        camera_launch,

        LogInfo(msg='LanderPi ROS2 stack launched'),
    ])
