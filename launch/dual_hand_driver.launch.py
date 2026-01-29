#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LZ Dual Hand Driver Launch File for ROS2
双手驱动ROS2启动文件

Usage:
    ros2 launch lz_hand_rs485_driver dual_hand_driver.launch.py
    ros2 launch lz_hand_rs485_driver dual_hand_driver.launch.py right_port:=/dev/ttyUSB0 left_port:=/dev/ttyUSB1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Declare launch arguments
    right_port_arg = DeclareLaunchArgument(
        'right_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for right hand'
    )
    
    left_port_arg = DeclareLaunchArgument(
        'left_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for left hand'
    )
    
    right_enabled_arg = DeclareLaunchArgument(
        'right_enabled',
        default_value='true',
        description='Enable right hand'
    )
    
    left_enabled_arg = DeclareLaunchArgument(
        'left_enabled',
        default_value='true',
        description='Enable left hand'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Communication baud rate'
    )
    
    feedback_rate_arg = DeclareLaunchArgument(
        'feedback_rate',
        default_value='50.0',
        description='Feedback publishing rate in Hz'
    )
    
    gradual_step_size_arg = DeclareLaunchArgument(
        'gradual_step_size',
        default_value='100',
        description='Max change per update'
    )
    
    # Get launch configurations
    right_port = LaunchConfiguration('right_port')
    left_port = LaunchConfiguration('left_port')
    right_enabled = LaunchConfiguration('right_enabled')
    left_enabled = LaunchConfiguration('left_enabled')
    baudrate = LaunchConfiguration('baudrate')
    feedback_rate = LaunchConfiguration('feedback_rate')
    gradual_step_size = LaunchConfiguration('gradual_step_size')
    
    # Right hand driver
    right_hand_group = GroupAction([
        PushRosNamespace('right_hand'),
        Node(
            package='lz_hand_rs485_driver',
            executable='hand_driver_node.py',
            name='lz_hand_driver',
            output='screen',
            parameters=[{
                'port': right_port,
                'baudrate': baudrate,
                'timeout': 0.1,
                'hand_id': 1,
                'feedback_rate': feedback_rate,
                'gradual_step_size': gradual_step_size,
                'frame_id': 'right_hand_link',
            }],
            emulate_tty=True,
        ),
    ], condition=IfCondition(right_enabled))
    
    # Left hand driver
    left_hand_group = GroupAction([
        PushRosNamespace('left_hand'),
        Node(
            package='lz_hand_rs485_driver',
            executable='hand_driver_node.py',
            name='lz_hand_driver',
            output='screen',
            parameters=[{
                'port': left_port,
                'baudrate': baudrate,
                'timeout': 0.1,
                'hand_id': 2,
                'feedback_rate': feedback_rate,
                'gradual_step_size': gradual_step_size,
                'frame_id': 'left_hand_link',
            }],
            emulate_tty=True,
        ),
    ], condition=IfCondition(left_enabled))
    
    return LaunchDescription([
        right_port_arg,
        left_port_arg,
        right_enabled_arg,
        left_enabled_arg,
        baudrate_arg,
        feedback_rate_arg,
        gradual_step_size_arg,
        right_hand_group,
        left_hand_group,
    ])
