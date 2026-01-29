#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LZ Hand Driver Launch File for ROS2
灵巧手驱动ROS2启动文件

Usage:
    ros2 launch lz_hand_rs485_driver hand_driver.launch.py
    ros2 launch lz_hand_rs485_driver hand_driver.launch.py port:=/dev/ttyUSB1 hand_id:=2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RS485 communication'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Communication baud rate'
    )
    
    hand_id_arg = DeclareLaunchArgument(
        'hand_id',
        default_value='1',
        description='Hand slave address (1=right, 2=left)'
    )
    
    feedback_rate_arg = DeclareLaunchArgument(
        'feedback_rate',
        default_value='50.0',
        description='Feedback publishing rate in Hz'
    )
    
    gradual_step_size_arg = DeclareLaunchArgument(
        'gradual_step_size',
        default_value='100',
        description='Max change per update for gradual adjustment'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node'
    )
    
    # Get launch configurations
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    hand_id = LaunchConfiguration('hand_id')
    feedback_rate = LaunchConfiguration('feedback_rate')
    gradual_step_size = LaunchConfiguration('gradual_step_size')
    namespace = LaunchConfiguration('namespace')
    
    # Hand driver node
    hand_driver_node = Node(
        package='lz_hand_rs485_driver',
        executable='hand_driver_node.py',
        name='lz_hand_driver',
        namespace=namespace,
        output='screen',
        parameters=[{
            'port': port,
            'baudrate': baudrate,
            'timeout': 0.1,
            'hand_id': hand_id,
            'hand_control_topic': 'hand_control',
            'joint_control_topic': 'joint_control',
            'hand_feedback_topic': 'hand_feedback',
            'joint_feedback_topic': 'joint_feedback',
            'force_feedback_topic': 'force_feedback',
            'motor_feedback_topic': 'motor_feedback',
            'joint_states_topic': 'joint_states',
            'feedback_rate': feedback_rate,
            'gradual_step_size': gradual_step_size,
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        port_arg,
        baudrate_arg,
        hand_id_arg,
        feedback_rate_arg,
        gradual_step_size_arg,
        namespace_arg,
        hand_driver_node,
    ])
