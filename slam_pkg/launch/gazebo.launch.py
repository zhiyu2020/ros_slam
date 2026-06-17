#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    slam_pkg_share = get_package_share_directory('slam_pkg')
    gazebo_share = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    pause = LaunchConfiguration('pause')

    default_world = os.path.join(slam_pkg_share, 'worlds', 'empty_room.world')
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            os.path.dirname(slam_pkg_share),
            ':',
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
        ],
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'pause': pause,
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='World file loaded by Gazebo.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Open the Gazebo graphical client.',
        ),
        DeclareLaunchArgument(
            'pause',
            default_value='false',
            description='Start Gazebo with physics paused.',
        ),
        gazebo_model_path,
        gzserver,
        gzclient,
    ])
