#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_pkg_share = get_package_share_directory('slam_pkg')
    launch_dir = os.path.join(slam_pkg_share, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    pause = LaunchConfiguration('pause')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    use_slam = LaunchConfiguration('use_slam')
    use_observer = LaunchConfiguration('use_observer')
    use_robo_env = LaunchConfiguration('use_robo_env')
    publish_cmd_vel = LaunchConfiguration('publish_cmd_vel')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')

    default_world = os.path.join(slam_pkg_share, 'worlds', 'robocup_home.world')
    rviz_config = os.path.join(slam_pkg_share, 'rviz', 'g1_slam.rviz')
    slam_params = os.path.join(slam_pkg_share, 'config', 'slam_toolbox_g1.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'pause': pause,
        }.items(),
    )

    spawn_g1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_g1.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x': robot_x,
            'y': robot_y,
            'z': robot_z,
            'yaw': robot_yaw,
        }.items(),
    )

    spawn_robo_env_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_objects.launch.py')),
        condition=IfCondition(use_robo_env),
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_slam),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    observer = Node(
        package='slam_pkg',
        executable='sim_observer',
        name='sim_observer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_cmd_vel': publish_cmd_vel,
        }],
        condition=IfCondition(use_observer),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('pause', default_value='false'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_slam', default_value='true'),
        DeclareLaunchArgument('use_observer', default_value='true'),
        DeclareLaunchArgument('use_robo_env', default_value='true'),
        DeclareLaunchArgument('publish_cmd_vel', default_value='false'),
        DeclareLaunchArgument('robot_x', default_value='-6.0'),
        DeclareLaunchArgument('robot_y', default_value='-0.5'),
        DeclareLaunchArgument('robot_z', default_value='0.0'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),
        gazebo,
        spawn_g1,
        spawn_robo_env_objects,
        slam_toolbox,
        rviz,
        observer,
    ])
