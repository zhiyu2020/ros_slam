#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Step 1: expose the important choices as launch arguments.
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_simulator = LaunchConfiguration("start_simulator")
    start_rviz = LaunchConfiguration("start_rviz")
    simulation_package = LaunchConfiguration("simulation_package")
    simulation_launch_file = LaunchConfiguration("simulation_launch_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    default_slam_params = PathJoinSubstitution(
        [FindPackageShare("slam_pkg"), "config", "slam_toolbox.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("slam_pkg"), "rviz", "slam.rviz"]
    )

    # Step 2: start the simulator that publishes /scan, /odom, /tf and /clock.
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(simulation_package),
                    "launch",
                    simulation_launch_file,
                ]
            )
        ),
        condition=IfCondition(start_simulator),
    )

    # Step 3: start slam_toolbox in mapping mode.
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Step 4: open RViz with map, scan and robot displays already configured.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo /clock for all ROS time.",
            ),
            DeclareLaunchArgument(
                "start_simulator",
                default_value="true",
                description="Start the Gazebo world before SLAM.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz with the local SLAM view.",
            ),
            DeclareLaunchArgument(
                "simulation_package",
                default_value="slam_pkg",
                description="Package that provides the simulator launch file.",
            ),
            DeclareLaunchArgument(
                "simulation_launch_file",
                default_value="robocup_home.launch.py",
                description="Simulator launch file inside simulation_package/launch.",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_slam_params,
                description="YAML parameters for slam_toolbox.",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=default_rviz_config,
                description="RViz config used to visualize mapping.",
            ),
            simulator,
            slam_toolbox,
            rviz,
        ]
    )
