#!/usr/bin/env python3
"""online_async_slam.launch.py — start slam_toolbox to build a map live.

Run this AFTER the simulation is up (Session 2):
    ros2 launch acadbot_gazebo simulation.launch.py
    ros2 launch acadbot_slam online_async_slam.launch.py

Then drive the robot (teleop) and watch the /map fill in inside RViz2. When the
map looks complete, save it from the RViz "SlamToolbox" panel or with:
    ros2 run nav2_map_server map_saver_cli -f src/acadbot_navigation/maps/academy_map

NOTE (ROS 2 Jazzy): `async_slam_toolbox_node` is a *lifecycle* node. Starting it
as a plain Node leaves it unconfigured — it never subscribes to /scan, never
publishes /map and never publishes the map->odom transform, so map_saver_cli
fails with "Failed to spin map subscription". It has to be launched as a
LifecycleNode and driven through configure -> activate, which is what the
autostart wiring below does (mirrors slam_toolbox's own online_async_launch.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = get_package_share_directory('acadbot_slam')
    default_params = os.path.join(
        pkg, 'config', 'mapper_params_online_async.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')

    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'use_lifecycle_manager': use_lifecycle_manager,
        }],
    )

    autostart_on = IfCondition(
        AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))

    configure = EmitEvent(
        event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                          transition_id=Transition.TRANSITION_CONFIGURE),
        condition=autostart_on)

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring', goal_state='inactive',
            entities=[
                LogInfo(msg='[acadbot_slam] slam_toolbox configured, activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE)),
            ]),
        condition=autostart_on)

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Configure and activate slam_toolbox automatically.'),
        DeclareLaunchArgument(
            'use_lifecycle_manager', default_value='false',
            description='Let an external Nav2 lifecycle manager drive the node '
                        'instead of autostarting it here.'),
        slam,
        configure,
        activate,
    ])
