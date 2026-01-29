#!/usr/bin/env python3
"""
灵巧手驱动启动文件（Hand Driver Launch File）

用法（Usage）:
    ros2 launch lz_hand_rs485_driver hand_driver.launch.py
    ros2 launch lz_hand_rs485_driver hand_driver.launch.py port:=/dev/ttyUSB0 hand_id:=2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 启动参数（Launch arguments）
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='串口路径（Serial port）'),
        DeclareLaunchArgument('baudrate', default_value='115200', description='波特率（Baud rate）'),
        DeclareLaunchArgument('hand_id', default_value='1', description='手ID：1=右手，2=左手（1=right, 2=left）'),
        DeclareLaunchArgument('feedback_rate', default_value='20.0', description='反馈频率Hz（Feedback rate）'),
        DeclareLaunchArgument('auto_reconnect', default_value='true', description='自动重连（Auto reconnect）'),
        DeclareLaunchArgument('namespace', default_value='', description='命名空间（Namespace）'),
        
        # 驱动节点（Driver node）
        Node(
            package='lz_hand_rs485_driver',
            executable='hand_driver_node.py',
            name='lz_hand_driver',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'hand_id': LaunchConfiguration('hand_id'),
                'feedback_rate': LaunchConfiguration('feedback_rate'),
                'auto_reconnect': LaunchConfiguration('auto_reconnect'),
            }],
        ),
    ])
