#!/usr/bin/env python3
"""patrol.launch.py — run the Nav2 patrol commander (Session 4)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('acadbot_control')
    params = os.path.join(pkg, 'config', 'patrol_waypoints.yaml')

    return LaunchDescription([
        Node(
            package='acadbot_control',
            executable='patrol_commander',
            name='patrol_commander',
            output='screen',
            parameters=[params],
        ),
    ])
