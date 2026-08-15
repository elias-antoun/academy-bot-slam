#!/usr/bin/env python3
"""courier.launch.py — everything needed to demonstrate the courier, in one command.

    ros2 launch acadbot_courier courier.launch.py

Brings up, in this order:
  1. the simulation (Gazebo + the ros_gz bridge)
  2. map_server + AMCL on the saved map, seeded with the robot's spawn pose
  3. the Nav2 servers, held back until localization is up
  4. RViz, showing the map, the costmaps and the courier's locations
  5. courier_server itself

Then, in another terminal:

    ros2 service call /request_delivery \\
      acadbot_courier_msgs/srv/RequestDelivery "{pickup: reception, dropoff: lab_bench}"
    ros2 action send_goal /execute_delivery \\
      acadbot_courier_msgs/action/ExecuteDelivery "{job_id: job_0001}" --feedback

Two things about the ordering are not arbitrary.

AMCL is seeded by publishing /initialpose on a timer, not by its
set_initial_pose parameter -- see config/amcl.yaml for why that parameter
cannot work under simulated time. Localization itself starts immediately, so
AMCL's TF buffer has the whole startup to fill before the pose arrives.

Nav2 is delayed until after the seed. Its costmaps cannot configure without
map->odom, and if that transition times out the lifecycle manager aborts the
whole bringup and leaves every server INACTIVE, after which every goal comes
back "rejected".

This composes the stack itself rather than including acadbot_bringup's
autonomy.launch.py, which does not forward params_file: AMCL here needs the
courier's own parameter file, while the Nav2 servers keep the course's
untouched nav2_params.yaml.

Arguments:
    headless:=true    Gazebo server only — no GUI, no GPU needed
    rviz:=false       skip RViz2
    map:=<path>       localize on a different saved map
    initial_x/initial_y/initial_yaw   where the robot starts, in the map frame
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

    # map_server + AMCL + their lifecycle manager, started with the
    # simulation and not after it. AMCL keeps its own TF buffer, and that
    # buffer only starts filling when the node does -- start it late and the
    # buffer holds a single sample, so an initial pose arriving moments later
    # cannot be transformed at all:
    #
    #   Requested time 20.243 but the earliest data is at time 20.400
    #
    # Starting it here gives the buffer the whole startup to fill before the
    # pose is seeded below.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'params_file': amcl_params,
        }.items(),
    )

    # Tell AMCL where the robot starts, so the demo needs no mouse.
    #
    # A node rather than a `ros2 topic pub` on a timer, because publishing once
    # at a fixed moment is unreliable: AMCL looks up base_footprint->odom
    # around the pose's timestamp, and early in a run its TF buffer holds only
    # a fraction of a second, so the pose lands outside it. When the window
    # opens depends on the machine. The seeder publishes, checks whether
    # map->odom actually appeared, and repeats until it has -- so there is no
    # delay to guess at. It also leaves an already-localized robot alone.
    seeder = Node(
        package='acadbot_courier',
        executable='initial_pose_seeder',
        name='initial_pose_seeder',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
        }],
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
            'initial_x', default_value='0.0',
            description="AMCL's starting guess, in the map frame. The default "
                        'is where the robot spawns: map = gazebo + (3, 2), and '
                        'simulation.launch.py spawns it at gazebo (-3, -2).'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'nav2_delay', default_value='15.0',
            description='Seconds to wait for localization before starting Nav2.'),
        sim,
        localization,
        seeder,
        delayed_nav2,
        courier,
        rviz,
    ])
