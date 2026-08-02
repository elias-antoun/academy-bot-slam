#!/usr/bin/env python3
"""localization_slam.launch.py — slam_toolbox in localization mode (Session 4).

Loads a serialized map and provides the map->odom transform without mapping.
Use this with Nav2 instead of AMCL when you prefer pose-graph localization.

NOTE (ROS 2 Jazzy): like the mapping node, `localization_slam_toolbox_node` is a
*lifecycle* node — it has to be configured and activated or it never publishes
map->odom and Nav2 sits there with no localisation. See the sibling
online_async_slam.launch.py for the same wiring.
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
    default_params = os.path.join(pkg, 'config', 'localization_params.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')

    slam = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
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
                LogInfo(msg='[acadbot_slam] slam_toolbox (localization) '
                            'configured, activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE)),
            ]),
        condition=autostart_on)

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_lifecycle_manager', default_value='false'),
        slam,
        configure,
        activate,
    ])
