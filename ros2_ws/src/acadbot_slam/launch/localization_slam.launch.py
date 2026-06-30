#!/usr/bin/env python3
"""localization_slam.launch.py — slam_toolbox in localization mode (Session 4).

Loads a serialized map and provides the map->odom transform without mapping.
Use this with Nav2 instead of AMCL when you prefer pose-graph localization.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('acadbot_slam')
    default_params = os.path.join(pkg, 'config', 'localization_params.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file],
        ),
    ])
