#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    start_rl_sim = LaunchConfiguration('start_rl_sim')
    start_controllers = LaunchConfiguration('start_controllers')
    publish_cmd_vel = LaunchConfiguration('publish_cmd_vel')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')
    entity = LaunchConfiguration('entity')
    model_states_topic = LaunchConfiguration('model_states_topic')
    use_entity_state_service = LaunchConfiguration('use_entity_state_service')

    default_world = os.path.join(slam_pkg_share, 'worlds', 'robocup_home.world')
    g1_xacro = os.path.join(slam_pkg_share, 'xacro', 'g1', 'g1_rl_slam.urdf.xacro')
    rviz_config = os.path.join(slam_pkg_share, 'rviz', 'g1_rl_slam.rviz')
    slam_params = os.path.join(slam_pkg_share, 'config', 'slam_toolbox_g1_rl.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', g1_xacro]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'pause': pause,
        }.items(),
    )

    spawn_robo_env_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_objects.launch.py')),
        condition=IfCondition(use_robo_env),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_g1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_g1_rl',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', entity,
            '-x', robot_x,
            '-y', robot_y,
            '-z', robot_z,
            '-Y', robot_yaw,
        ],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        name='spawn_joint_state_broadcaster',
        output='screen',
        arguments=['joint_state_broadcaster'],
        condition=IfCondition(start_controllers),
    )

    param_node = Node(
        package='demo_nodes_cpp',
        executable='parameter_blackboard',
        name='param_node',
        output='screen',
        parameters=[{
            'robot_name': 'g1',
            'gazebo_model_name': ParameterValue(entity, value_type=str),
            'use_sim_time': use_sim_time,
        }],
    )

    odom = Node(
        package='slam_pkg',
        executable='gazebo_model_odom',
        name='gazebo_model_odom',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_name': ParameterValue(entity, value_type=str),
            'model_states_topic': ParameterValue(model_states_topic, value_type=str),
            'entity_state_service': '/get_entity_state',
            'use_entity_state_service': ParameterValue(use_entity_state_service, value_type=bool),
            'update_rate': 50.0,
            'odom_frame': 'odom',
            'base_frame': 'base',
            'publish_tf': True,
        }],
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
            'linear_x': 0.0,
            'angular_z': 0.0,
        }],
        condition=IfCondition(use_observer),
    )

    rl_sim = Node(
        package='rl_sar',
        executable='rl_sim',
        name='rl_sim',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(start_rl_sim),
    )

    after_spawn_g1 = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_g1,
            on_exit=[
                TimerAction(period=2.0, actions=[joint_state_broadcaster]),
                TimerAction(period=1.0, actions=[odom]),
                spawn_robo_env_objects,
                slam_toolbox,
                rviz,
                observer,
                TimerAction(period=5.0, actions=[rl_sim]),
            ],
        )
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
        DeclareLaunchArgument('start_rl_sim', default_value='false'),
        DeclareLaunchArgument('start_controllers', default_value='true'),
        DeclareLaunchArgument('publish_cmd_vel', default_value='false'),
        DeclareLaunchArgument('robot_x', default_value='-6.0'),
        DeclareLaunchArgument('robot_y', default_value='-0.5'),
        DeclareLaunchArgument('robot_z', default_value='1.0'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),
        DeclareLaunchArgument('entity', default_value='g1_rl'),
        DeclareLaunchArgument('model_states_topic', default_value='/model_states'),
        DeclareLaunchArgument('use_entity_state_service', default_value='false'),
        gazebo,
        robot_state_publisher,
        spawn_g1,
        param_node,
        after_spawn_g1,
    ])
