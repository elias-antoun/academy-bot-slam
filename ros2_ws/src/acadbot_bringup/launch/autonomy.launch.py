#!/usr/bin/env python3
"""autonomy.launch.py — one command for the Session 4 full-autonomy demo.

Brings up:  simulation + Nav2 (with recovery behaviors) + slam_toolbox
localization + RViz2. Start the C++ mission node separately so students can see
it command the stack:

    ros2 launch acadbot_bringup autonomy.launch.py
    # in another terminal, the C++ patrol commander:
    ros2 launch acadbot_control patrol.launch.py

NOTE: requires a serialized map at
      acadbot_navigation/maps/academy_map_serial(.posegraph/.data)
      (build it in Session 2). For a quick test without a saved map, run
      mapping.launch.py instead and send goals manually in RViz.

Arguments:
    localization:=slam|amcl   which localizer to run (default slam)
    headless:=true            Gazebo server only — no GUI, no GPU needed
    rviz:=false               skip RViz2 (no display available, or CI)
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

    rviz_config = os.path.join(pkg_desc, 'rviz', 'nav2.rviz')
    localization = LaunchConfiguration('localization')
    headless = LaunchConfiguration('headless')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'simulation.launch.py')),
        launch_arguments={'headless': headless}.items())

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')),
        launch_arguments={'localization': localization}.items())

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('localization', default_value='slam'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server-only (no GUI, no GPU required).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2. Set false on a machine with no display.'),
        sim,
        nav,
        rviz,
    ])
