#!/usr/bin/env python3

import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_gazebo_readable_model(source_model, package_share):
    with open(source_model, "r", encoding="utf-8") as model_file:
        robot_xml = model_file.read()

    robot_xml = robot_xml.replace(
        "package://slam_pkg/meshes/",
        f"file://{os.path.join(package_share, 'meshes')}/",
    )
    robot_xml = robot_xml.replace(
        "$(find slam_pkg)",
        package_share,
    )

    patched_model = os.path.join(tempfile.gettempdir(), "slam_pkg_wpb_home_lidar.model")
    with open(patched_model, "w", encoding="utf-8") as model_file:
        model_file.write(robot_xml)

    return patched_model, robot_xml


def spawn_robot_nodes(context):
    package_share = get_package_share_directory("slam_pkg")
    robot_model = os.path.join(package_share, "models", "wpb_home_lidar.model")
    patched_model, robot_xml = make_gazebo_readable_model(robot_model, package_share)

    doc = xacro.parse(robot_xml)
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            patched_model,
            "-entity",
            "wpb_home",
            "-x",
            LaunchConfiguration("pose_x").perform(context),
            "-y",
            LaunchConfiguration("pose_y").perform(context),
            "-Y",
            LaunchConfiguration("pose_theta").perform(context),
        ],
        output="screen",
    )

    return [robot_state_publisher, spawn_robot]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("pose_x", default_value="0.0"),
            DeclareLaunchArgument("pose_y", default_value="0.0"),
            DeclareLaunchArgument("pose_theta", default_value="0.0"),
            OpaqueFunction(function=spawn_robot_nodes),
        ]
    )
