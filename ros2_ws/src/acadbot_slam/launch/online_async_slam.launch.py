#!/usr/bin/env python3
"""online_async_slam.launch.py — start slam_toolbox to build a map live.

Run this AFTER the simulation is up (Session 2):
    ros2 launch acadbot_gazebo simulation.launch.py
    ros2 launch acadbot_slam online_async_slam.launch.py

Then drive the robot (teleop) and watch the /map fill in inside RViz2. When the
map looks complete, save it from the RViz "SlamToolbox" panel or with:
    ros2 run nav2_map_server map_saver_cli -f ~/academy_map
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('acadbot_slam')
    default_params = os.path.join(
        pkg, 'config', 'mapper_params_online_async.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
