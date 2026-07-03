#!/usr/bin/env python3
"""
Atom P2 Arms-only launch - Arms control only.
"""

import os
import time
import random
import string
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    timestamp = str(int(time.time()))
    random_suffix = ''.join(random.choices(string.ascii_lowercase, k=4))
    robot_name_in_model = f'atom_p2_arms_{timestamp}_{random_suffix}'

    urdf_name = "atom_p2/atom_gazebo_control.xacro"

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )

    paused_arg = DeclareLaunchArgument(
        'paused', default_value='false',
        description='Pause Gazebo on start'
    )

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Show Gazebo GUI'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'paused': LaunchConfiguration('paused'),
            'gui': LaunchConfiguration('gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'verbose': 'true'
        }.items(),
    )

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('atom_urdf'), 'urdf', urdf_name])
    ])
    robot_description = {'robot_description': robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name_in_model,
            '-z', '0.1', '-timeout', '60'
        ],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_left_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'left_arm_controller'],
        output='screen'
    )

    load_right_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'right_arm_controller'],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg, paused_arg, gui_arg, gazebo,
        node_robot_state_publisher, spawn_entity,
        TimerAction(period=3.0, actions=[load_joint_state_broadcaster]),
        TimerAction(period=4.0, actions=[load_left_arm_controller, load_right_arm_controller])
    ])
