#!/usr/bin/env python3

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch_ros.actions import Node


def make_gazebo_readable_model(package_share, *model_parts):
    source_model = os.path.join(package_share, "models", *model_parts)

    with open(source_model, "r", encoding="utf-8") as model_file:
        model_xml = model_file.read()

    model_xml = model_xml.replace(
        "package://slam_pkg/meshes/",
        f"file://{os.path.join(package_share, 'meshes')}/",
    )
    model_xml = model_xml.replace(
        "$(find slam_pkg)",
        package_share,
    )

    patched_name = "slam_pkg_" + "_".join(model_parts)
    patched_model = os.path.join(tempfile.gettempdir(), patched_name)
    with open(patched_model, "w", encoding="utf-8") as model_file:
        model_file.write(model_xml)

    return patched_model


def spawn_model(entity, model_file, x, y, z=None, yaw=None):
    arguments = ["-file", model_file, "-entity", entity, "-x", x, "-y", y]
    if z is not None:
        arguments.extend(["-z", z])
    if yaw is not None:
        arguments.extend(["-Y", yaw])

    return Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name=f"spawn_{entity}",
        arguments=arguments,
        output="screen",
    )


def spawn_object_nodes(context):
    package_share = get_package_share_directory("slam_pkg")

    furniture_1 = [
        spawn_model("bed", make_gazebo_readable_model(package_share, "bed.model"), "5.0", "-3.9", yaw="3.1415926"),
        spawn_model("sofa", make_gazebo_readable_model(package_share, "sofa.model"), "-1.0", "-3.9", yaw="1.57"),
        spawn_model("tea_table", make_gazebo_readable_model(package_share, "tea_table.model"), "-2.1", "-2.2", yaw="1.57"),
        spawn_model("bookshelft", make_gazebo_readable_model(package_share, "bookshelft.model"), "2.0", "-0.55", yaw="-1.57"),
    ]

    furniture_2 = [
        spawn_model("kitchen_table", make_gazebo_readable_model(package_share, "table.model"), "-3.5", "3.7", yaw="1.57"),
        spawn_model("cupboard_0", make_gazebo_readable_model(package_share, "cupboard.model"), "-2.0", "0.7", yaw="1.57"),
        spawn_model("cupboard_1", make_gazebo_readable_model(package_share, "cupboard.model"), "-1.3", "3.7", yaw="-1.57"),
    ]

    dining_tables = [
        spawn_model("dinning_table_0", make_gazebo_readable_model(package_share, "table.model"), "1.5", "1.5", yaw="1.57"),
        spawn_model("dinning_table_1", make_gazebo_readable_model(package_share, "table.model"), "1.5", "2.0", yaw="1.57"),
        spawn_model("dinning_table_2", make_gazebo_readable_model(package_share, "table.model"), "2.7", "1.5", yaw="1.57"),
        spawn_model("dinning_table_3", make_gazebo_readable_model(package_share, "table.model"), "2.7", "2.0", yaw="1.57"),
    ]

    chairs = [
        spawn_model("chair_0", make_gazebo_readable_model(package_share, "chair.model"), "1.5", "1.2", yaw="1.57"),
        spawn_model("chair_1", make_gazebo_readable_model(package_share, "chair.model"), "1.5", "2.3", yaw="-1.57"),
        spawn_model("chair_2", make_gazebo_readable_model(package_share, "chair.model"), "2.7", "1.2", yaw="1.57"),
        spawn_model("chair_3", make_gazebo_readable_model(package_share, "chair.model"), "2.7", "2.3", yaw="-1.57"),
    ]

    bottles = [
        spawn_model("red_bottle", make_gazebo_readable_model(package_share, "bottles", "red_bottle.model"), "-3.3", "3.55", z="2"),
        spawn_model("green_bottle", make_gazebo_readable_model(package_share, "bottles", "green_bottle.model"), "-3.6", "3.55", z="2"),
    ]

    return [
        TimerAction(period=1.0, actions=furniture_1),
        TimerAction(period=2.0, actions=furniture_2),
        TimerAction(period=3.0, actions=dining_tables),
        TimerAction(period=4.0, actions=chairs),
        TimerAction(period=5.0, actions=bottles),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=spawn_object_nodes)])
