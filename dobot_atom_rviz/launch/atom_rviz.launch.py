#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot Atom机器人RViz可视化启动文件

功能描述:
- 启动robot_state_publisher节点发布机器人状态
- 启动joint_state_publisher节点发布关节状态
- 启动RViz2进行机器人模型可视化

作者: futingxing
日期: 2026-06-01
版本: 1.0.0
Copyright (c) 2026 Dobot. All rights reserved.
"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    """
    生成Atom机器人RViz可视化的启动描述
    
    Returns:
        LaunchDescription: ROS2启动描述对象
    """
    
    # 获取atom_urdf包的路径
    atom_urdf_path = get_package_share_path('atom_urdf')
    # 设置默认的URDF模型文件路径
    default_model_path = atom_urdf_path / 'urdf/atom_w_p3/atom_w_p3.urdf'
    
    # 获取当前包的路径用于RViz配置文件
    dobot_atom_rviz_path = get_package_share_path('dobot_atom_rviz')
    default_rviz_config_path = dobot_atom_rviz_path / 'rviz/atom.rviz'

    # 声明启动参数
    # 模型参数：URDF文件的绝对路径
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_model_path),
        description='Absolute path to Atom robot urdf file'
    )
    
    # RViz配置参数：RViz配置文件的绝对路径
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    # 机器人描述参数：使用xacro处理URDF文件
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # 机器人状态发布器节点：发布机器人的TF变换和状态信息
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # 关节状态发布器节点：发布关节状态信息
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # RViz2可视化节点：启动RViz进行机器人模型可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # 返回启动描述
    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])