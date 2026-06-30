#!/usr/bin/env python3
"""square_driver.launch.py — run the open-loop drift demo (Session 1)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('acadbot_control')
    params = os.path.join(pkg, 'config', 'square_driver.yaml')

    return LaunchDescription([
        Node(
            package='acadbot_control',
            executable='square_driver',
            name='square_driver',
            output='screen',
            parameters=[params],
        ),
    ])
