#!/usr/bin/env python3
"""
双手驱动启动文件（Dual Hand Driver Launch File）

用法（Usage）:
    ros2 launch lz_hand_rs485_driver dual_hand_driver.launch.py
    ros2 launch lz_hand_rs485_driver dual_hand_driver.launch.py right_port:=/dev/ttyUSB0 left_port:=/dev/ttyUSB1

说明：不指定端口时自动从config/hand_config.yaml读取
Note: Ports will be automatically read from config/hand_config.yaml if not specified
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def load_config_file(package_name, config_file):
    """加载配置文件（Load configuration file）"""
    try:
        package_share_path = FindPackageShare(package_name).find(package_name)
        config_path = os.path.join(package_share_path, 'config', config_file)
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
    except Exception:
        pass
    return None


def get_default_config():
    """获取默认配置（Get default configuration）"""
    config = load_config_file('lz_hand_rs485_driver', 'hand_config.yaml')
    defaults = {'baudrate': 115200, 'feedback_rate': 20.0}
    if config:
        if 'serial' in config and 'baudrate' in config['serial']:
            defaults['baudrate'] = config['serial']['baudrate']
        if 'control' in config and 'feedback_rate' in config['control']:
            defaults['feedback_rate'] = config['control']['feedback_rate']
    return defaults


def get_ports_from_config():
    """从配置文件获取串口（Get ports from config）"""
    config = load_config_file('lz_hand_rs485_driver', 'hand_config.yaml')
    ports = {'right_port': '/dev/ttyUSB0', 'left_port': '/dev/ttyUSB1'}
    if config and 'serial' in config:
        if 'right_hand_port' in config['serial']:
            ports['right_port'] = config['serial']['right_hand_port']
        if 'left_hand_port' in config['serial']:
            ports['left_port'] = config['serial']['left_hand_port']
    return ports


def launch_setup(context):
    """启动设置（Launch setup）"""
    defaults = get_default_config()
    ports = get_ports_from_config()
    
    return [
        DeclareLaunchArgument('right_port', default_value=ports['right_port'], description='右手串口（Right hand port）'),
        DeclareLaunchArgument('left_port', default_value=ports['left_port'], description='左手串口（Left hand port）'),
        DeclareLaunchArgument('right_enabled', default_value='true', description='启用右手（Enable right）'),
        DeclareLaunchArgument('left_enabled', default_value='true', description='启用左手（Enable left）'),
        DeclareLaunchArgument('baudrate', default_value=str(defaults['baudrate']), description='波特率（Baud rate）'),
        DeclareLaunchArgument('feedback_rate', default_value=str(defaults['feedback_rate']), description='反馈频率Hz（Feedback rate）'),
        DeclareLaunchArgument('auto_reconnect', default_value='true', description='自动重连（Auto reconnect）'),
        
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
    ]


def generate_launch_description():
    """生成启动描述（Generate launch description）"""
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
