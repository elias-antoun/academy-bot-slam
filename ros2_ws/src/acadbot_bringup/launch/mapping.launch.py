#!/usr/bin/env python3
"""mapping.launch.py — one command for the Session 2 LiDAR-SLAM lab.

Brings up:  simulation (Gazebo) + slam_toolbox (mapping) + RViz2.
Then drive the robot with teleop and watch the map build:

    ros2 launch acadbot_bringup mapping.launch.py
    # in another terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('acadbot_gazebo')
    pkg_slam = get_package_share_directory('acadbot_slam')
    pkg_desc = get_package_share_directory('acadbot_description')

    rviz_config = os.path.join(pkg_desc, 'rviz', 'slam.rviz')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'simulation.launch.py')))

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_slam.launch.py')))

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=None,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        sim,
        slam,
        rviz,
    ])
