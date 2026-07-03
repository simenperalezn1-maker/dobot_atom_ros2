#!/usr/bin/env python3
"""
Atom P2 Gazebo launch - Full simulation with legged humanoid robot.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'atom_p2_gazebo'
    urdf_name = "atom_p2/atom_gazebo_control.xacro"

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('atom_urdf'), 'worlds', 'empty.world'
        ]),
        description='Gazebo world file'
    )

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('atom_urdf'), 'urdf', urdf_name])
    ])
    robot_description = {'robot_description': robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher', output='screen',
        parameters=[robot_description,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'atom_p2'],
        output='screen'
    )

    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare('atom_urdf'), 'config', 'atom_controllers.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg, world_arg, gazebo_launch,
        node_robot_state_publisher, spawn_entity,
        controller_manager, joint_state_broadcaster_spawner
    ])
