#!/usr/bin/env python3
"""courier.launch.py — everything needed to demonstrate the courier, in one command.

    ros2 launch acadbot_courier courier.launch.py

Brings up, in this order:
  1. the simulation (Gazebo + the ros_gz bridge)
  2. map_server + AMCL on the saved map, seeding themselves
  3. the Nav2 servers, held back until localization is up
  4. RViz, showing the map, the costmaps and the courier's locations
  5. courier_server itself

Then, in another terminal:

    ros2 service call /request_delivery \\
      acadbot_courier_msgs/srv/RequestDelivery "{pickup: reception, dropoff: lab_bench}"
    ros2 action send_goal /execute_delivery \\
      acadbot_courier_msgs/action/ExecuteDelivery "{job_id: job_0001}" --feedback

Two things about the ordering are not arbitrary.

AMCL seeds itself from `set_initial_pose` in config/amcl.yaml rather than
waiting for a 2D Pose Estimate click in RViz. "One command" cannot mean "one
command and then go and click something".

Nav2 is delayed while localization is not. The global costmap cannot configure
without map->odom, and if a lifecycle transition times out the manager aborts
the whole bringup and leaves every server INACTIVE -- at which point every goal
comes back "rejected". Starting the two together reproduces exactly that.

This composes the stack itself rather than including acadbot_bringup's
autonomy.launch.py, which does not forward params_file: AMCL here needs the
courier's own parameter file, while the Nav2 servers keep the course's
untouched nav2_params.yaml.

Arguments:
    headless:=true    Gazebo server only — no GUI, no GPU needed
    rviz:=false       skip RViz2
    map:=<path>       localize on a different saved map
    nav2_delay:=<s>   seconds to wait for localization before starting Nav2
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_courier = get_package_share_directory('acadbot_courier')
    pkg_gazebo = get_package_share_directory('acadbot_gazebo')
    pkg_nav = get_package_share_directory('acadbot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    courier_params = os.path.join(pkg_courier, 'config', 'courier.yaml')
    amcl_params = os.path.join(pkg_courier, 'config', 'amcl.yaml')
    rviz_config = os.path.join(pkg_courier, 'rviz', 'courier.rviz')
    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_nav, 'maps', 'academy_map.yaml')

    headless = LaunchConfiguration('headless')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'simulation.launch.py')),
        launch_arguments={'headless': headless}.items(),
    )

    # map_server + AMCL + their lifecycle manager. Not delayed: AMCL has to be
    # publishing map->odom before the Nav2 costmaps try to configure.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'params_file': amcl_params,
        }.items(),
    )

    # The course's own Nav2 bringup, with the course's own parameters: nothing
    # about the planner, controller or recovery behaviours changes for this
    # project.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2_stack.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
        }.items(),
    )
    delayed_nav2 = TimerAction(
        period=LaunchConfiguration('nav2_delay'), actions=[nav2])

    # Started immediately rather than with Nav2. Until navigate_to_pose comes
    # up the service rejects bookings with a reason saying so, which is the
    # behaviour we want anyway.
    courier = Node(
        package='acadbot_courier',
        executable='courier_server',
        name='courier_server',
        output='screen',
        parameters=[courier_params],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server-only (no GUI, no GPU required).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2. Set false on a machine with no display.'),
        DeclareLaunchArgument(
            'nav2_delay', default_value='12.0',
            description='Seconds to wait for localization before starting Nav2.'),
        sim,
        localization,
        delayed_nav2,
        courier,
        rviz,
    ])
