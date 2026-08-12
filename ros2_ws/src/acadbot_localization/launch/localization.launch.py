#!/usr/bin/env python3
"""localization.launch.py — simulation + a saved map + AMCL + RViz + the monitor.

SKELETON ONLY. The three launch arguments below are the agreed contract and are
already declared; the pieces they are supposed to start are not wired up yet.

OWNER: person 3 (the launch file).  What it has to bring up, and nothing else
(no Nav2 planner, controller or behaviour tree):

  1. acadbot_gazebo/launch/simulation.launch.py
     - forward the `headless` argument to it, the way
       acadbot_bringup/launch/mapping.launch.py already does.

  2. nav2_bringup/launch/localization_launch.py
     - this is map_server + amcl + their lifecycle manager, already written.
     - it takes `map`, `use_sim_time` and `params_file`.
     - params_file: acadbot_navigation/config/nav2_params.yaml (AMCL settings
       are already in there).
     - acadbot_navigation/launch/navigation.launch.py does exactly this inside
       a GroupAction — read it first, you are extracting that idea, not
       inventing it.

  3. rviz2 with acadbot_description/rviz/nav2.rviz, behind the `rviz` argument.

  4. this package's own localization_monitor node, with
     config/localization_monitor.yaml as its parameters, so that one command
     starts everything.

Run it, once implemented:
    ros2 launch acadbot_localization localization.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_nav = get_package_share_directory('acadbot_navigation')

    default_map = os.path.join(pkg_nav, 'maps', 'academy_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Occupancy-grid .yaml to localise against.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server-only (no GUI, no GPU required).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2. Set false on a machine with no display.'),

        LogInfo(msg=['[acadbot_localization] SKELETON launch file - nothing is '
                     'started yet. map=', LaunchConfiguration('map')]),
    ])
