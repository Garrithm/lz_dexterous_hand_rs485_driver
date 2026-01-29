#!/usr/bin/env python3
"""
灵巧手测试启动文件（Hand Test Launch File）

注意：交互模式请使用 ros2 run（launch无法处理键盘输入）
Note: Use ros2 run for interactive mode (launch cannot handle keyboard input)

用法（Usage）:
    ros2 launch lz_hand_rs485_driver hand_test.launch.py mode:=demo
    ros2 launch lz_hand_rs485_driver hand_test.launch.py port:=/dev/ttyUSB0 hand_id:=2 mode:=demo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # 启动参数（Launch arguments）
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='串口路径（Serial port）'),
        DeclareLaunchArgument('hand_id', default_value='1', description='手ID：1=右手，2=左手（1=right, 2=left）'),
        DeclareLaunchArgument('mode', default_value='interactive', description='模式：interactive/demo（Mode）'),
        DeclareLaunchArgument('feedback_rate', default_value='10.0', description='反馈频率Hz（Feedback rate）'),
        
        # 包含驱动启动文件（Include driver launch）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('lz_hand_rs485_driver'), 'launch', 'hand_driver.launch.py'])
            ]),
            launch_arguments={
                'port': LaunchConfiguration('port'),
                'hand_id': LaunchConfiguration('hand_id'),
                'feedback_rate': LaunchConfiguration('feedback_rate'),
            }.items()
        ),
        
        # 测试节点，延迟1秒启动（Test node, delayed 1s）
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='lz_hand_rs485_driver',
                    executable='hand_test_node.py',
                    name='lz_hand_test',
                    output='screen',
                    emulate_tty=True,
                    arguments=[LaunchConfiguration('mode')],
                    parameters=[{'hand_id': LaunchConfiguration('hand_id')}],
                )
            ]
        ),
    ])
