#!/usr/bin/env python3
"""
双手驱动启动文件（Dual Hand Driver Launch File）

用法（Usage）:
    ros2 launch lz_hand_rs485_driver dual_hand_driver.launch.py
    ros2 launch lz_hand_rs485_driver dual_hand_driver.launch.py right_port:=/dev/ttyUSB0 left_port:=/dev/ttyUSB1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    return LaunchDescription([
        # 启动参数（Launch arguments）
        DeclareLaunchArgument('right_port', default_value='/dev/ttyUSB0', description='右手串口（Right hand port）'),
        DeclareLaunchArgument('left_port', default_value='/dev/ttyUSB1', description='左手串口（Left hand port）'),
        DeclareLaunchArgument('right_enabled', default_value='true', description='启用右手（Enable right）'),
        DeclareLaunchArgument('left_enabled', default_value='true', description='启用左手（Enable left）'),
        DeclareLaunchArgument('baudrate', default_value='115200', description='波特率（Baud rate）'),
        DeclareLaunchArgument('feedback_rate', default_value='20.0', description='反馈频率Hz（Feedback rate）'),
        DeclareLaunchArgument('auto_reconnect', default_value='true', description='自动重连（Auto reconnect）'),
        
        # 右手节点，命名空间right_hand（Right hand node, namespace: right_hand）
        GroupAction([
            PushRosNamespace('right_hand'),
            Node(
                package='lz_hand_rs485_driver',
                executable='hand_driver_node.py',
                name='lz_hand_driver',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'port': LaunchConfiguration('right_port'),
                    'baudrate': LaunchConfiguration('baudrate'),
                    'hand_id': 1,
                    'feedback_rate': LaunchConfiguration('feedback_rate'),
                    'auto_reconnect': LaunchConfiguration('auto_reconnect'),
                    'frame_id': 'right_hand_link',
                }],
            ),
        ], condition=IfCondition(LaunchConfiguration('right_enabled'))),
        
        # 左手节点，命名空间left_hand（Left hand node, namespace: left_hand）
        GroupAction([
            PushRosNamespace('left_hand'),
            Node(
                package='lz_hand_rs485_driver',
                executable='hand_driver_node.py',
                name='lz_hand_driver',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'port': LaunchConfiguration('left_port'),
                    'baudrate': LaunchConfiguration('baudrate'),
                    'hand_id': 2,
                    'feedback_rate': LaunchConfiguration('feedback_rate'),
                    'auto_reconnect': LaunchConfiguration('auto_reconnect'),
                    'frame_id': 'left_hand_link',
                }],
            ),
        ], condition=IfCondition(LaunchConfiguration('left_enabled'))),
    ])
