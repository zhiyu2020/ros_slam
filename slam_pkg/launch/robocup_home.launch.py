#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_slam = FindPackageShare("slam_pkg")
    pkg_gazebo_ros = FindPackageShare("gazebo_ros")

    world = PathJoinSubstitution([pkg_slam, "worlds", "robocup_home.world"])

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, "launch", "gzserver.launch.py"])
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, "launch", "gzclient.launch.py"])
        )
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slam, "launch", "spawn_wpb_lidar.launch.py"])
        ),
        launch_arguments={
            "pose_x": LaunchConfiguration("robot_pose_x"),
            "pose_y": LaunchConfiguration("robot_pose_y"),
            "pose_theta": LaunchConfiguration("robot_pose_theta"),
        }.items(),
    )

    spawn_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slam, "launch", "spawn_objects.launch.py"])
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_pose_x", default_value="-6.0"),
            DeclareLaunchArgument("robot_pose_y", default_value="-0.5"),
            DeclareLaunchArgument("robot_pose_theta", default_value="0.0"),
            gzserver,
            gzclient,
            spawn_robot,
            spawn_objects,
        ]
    )
