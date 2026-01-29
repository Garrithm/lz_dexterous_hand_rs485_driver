#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LZ Hand Test Launch File for ROS2
灵巧手测试ROS2启动文件

Usage:
    ros2 launch lz_hand_rs485_driver hand_test.launch.py
    ros2 launch lz_hand_rs485_driver hand_test.launch.py mode:=demo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port'
    )
    
    hand_id_arg = DeclareLaunchArgument(
        'hand_id',
        default_value='1',
        description='Hand ID (1=right, 2=left)'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='interactive',
        description='Test mode: interactive or demo'
    )
    
    # Get launch configurations
    port = LaunchConfiguration('port')
    hand_id = LaunchConfiguration('hand_id')
    mode = LaunchConfiguration('mode')
    
    # Include hand driver launch
    hand_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lz_hand_rs485_driver'),
                'launch',
                'hand_driver.launch.py'
            ])
        ]),
        launch_arguments={
            'port': port,
            'hand_id': hand_id,
        }.items()
    )
    
    # Test node
    test_node = Node(
        package='lz_hand_rs485_driver',
        executable='hand_test_node.py',
        name='lz_hand_test',
        output='screen',
        arguments=[mode],
        parameters=[{
            'hand_id': hand_id,
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        port_arg,
        hand_id_arg,
        mode_arg,
        hand_driver_launch,
        test_node,
    ])
