#!/usr/bin/env python3
"""simulation.launch.py — bring up the full AcadBot simulation.

Starts, in order:
  1. robot_state_publisher  (URDF -> /robot_description + static TF)
  2. Gazebo Harmonic        (the academy_world)
  3. spawn the robot        (from /robot_description)
  4. ros_gz_bridge          (gz <-> ROS topics, see config/ros_gz_bridge.yaml)
  5. an image bridge        (camera color image, which needs ros_gz_image)

This is the single command students run first:
    ros2 launch acadbot_gazebo simulation.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                   PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('acadbot_gazebo')
    pkg_desc = get_package_share_directory('acadbot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    world_path = PathJoinSubstitution([pkg_gazebo, 'worlds', world])
    bridge_config = os.path.join(pkg_gazebo, 'config', 'ros_gz_bridge.yaml')

    # When headless, run gz server-only ('-s') — no GUI, so no GPU/OpenGL needed.
    # Handy for CI, grading and machines without a display.
    gz_flags = PythonExpression(
        ["'-r -s -v4 ' if '", headless, "' == 'true' else '-r -v4 '"])

    # Let Gazebo find our world & (future) model resources.
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_gazebo, 'worlds') + ':' +
              os.path.join(pkg_desc, '..'))

    # 1. robot_state_publisher (reuse the description package launch)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 2. Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [gz_flags, world_path]}.items(),
    )

    # 3. Spawn the robot from the /robot_description topic
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'acadbot',
            '-x', x, '-y', y, '-z', '0.1', '-Y', yaw,
        ],
    )

    # 4. Topic bridge (sensors, odom, cmd_vel, tf, clock)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
    )

    # 5. Camera image bridge (faster path for images than parameter_bridge)
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image', '/camera/depth_image'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='academy_world.sdf'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server-only (no GUI / no GPU needed)'),
        DeclareLaunchArgument('x', default_value='-3.0'),
        DeclareLaunchArgument('y', default_value='-2.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gz_resource_path,
        rsp,
        gz_sim,
        spawn,
        bridge,
        image_bridge,
    ])
