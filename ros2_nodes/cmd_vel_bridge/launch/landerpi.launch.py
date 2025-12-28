#!/usr/bin/env python3
"""Launch file for LanderPi ROS2 stack."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Config file path (mounted from host)
    config_file = '/ros2_ws/config/robot_params.yaml'

    return LaunchDescription([
        # Motor controller node (from ros_robot_controller package)
        Node(
            package='ros_robot_controller',
            executable='ros_robot_controller',
            name='ros_robot_controller',
            output='screen',
            parameters=[{'imu_frame': 'imu_link'}],
        ),

        # cmd_vel bridge node
        Node(
            package='cmd_vel_bridge',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[config_file],
        ),
    ])
