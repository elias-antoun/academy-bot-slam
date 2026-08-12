#!/usr/bin/env python3
"""localization.launch.py — load a saved map and localize on it with AMCL.

Brings up:  simulation + map_server + AMCL + RViz2 + localization_monitor.
No Nav2 — this answers "where am I on this map?", nothing more.

    ros2 launch acadbot_localization localization.launch.py
    # then in RViz: 2D Pose Estimate, and drive with teleop_twist_keyboard

Arguments:
    map:=<path to a .yaml>   which saved map to load
    headless:=true           Gazebo server only — no GUI, no GPU needed
    rviz:=false              skip RViz2 (no display available, or CI)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('acadbot_gazebo')
    pkg_nav = get_package_share_directory('acadbot_navigation')
    pkg_desc = get_package_share_directory('acadbot_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_self = get_package_share_directory('acadbot_localization')

    default_map = os.path.join(pkg_nav, 'maps', 'academy_map.yaml')
    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_desc, 'rviz', 'nav2.rviz')
    monitor_params = os.path.join(pkg_self, 'config', 'localization_monitor.yaml')

    headless = LaunchConfiguration('headless')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'simulation.launch.py')),
        launch_arguments={'headless': headless}.items())

    # map_server + amcl + lifecycle_manager_localization, already written by Nav2.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={'map': LaunchConfiguration('map'),
                          'use_sim_time': 'true',
                          'params_file': nav2_params}.items())

    monitor = Node(
        package='acadbot_localization', executable='localization_monitor',
        name='localization_monitor', output='screen',
        parameters=[monitor_params])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server-only (no GUI, no GPU required).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2. Set false on a machine with no display.'),
        sim,
        localization,
        monitor,
        rviz,
    ])
