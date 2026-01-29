#!/usr/bin/env python3
"""
灵巧手驱动启动文件（Hand Driver Launch File）

用法（Usage）:
    ros2 launch lz_hand_rs485_driver hand_driver.launch.py
    ros2 launch lz_hand_rs485_driver hand_driver.launch.py hand_id:=2
    ros2 launch lz_hand_rs485_driver hand_driver.launch.py port:=/dev/ttyUSB0 hand_id:=2

说明：指定hand_id时自动从config/hand_config.yaml读取串口
Note: Port will be automatically read from config/hand_config.yaml when hand_id is specified
"""

import os
import sys
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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


def get_port_from_config(hand_id_str):
    """从配置文件获取串口（Get port from config）"""
    try:
        hand_id = int(hand_id_str)
        config = load_config_file('lz_hand_rs485_driver', 'hand_config.yaml')
        if config and 'serial' in config:
            if hand_id == 1 and 'right_hand_port' in config['serial']:
                return config['serial']['right_hand_port']
            elif hand_id == 2 and 'left_hand_port' in config['serial']:
                return config['serial']['left_hand_port']
    except Exception:
        pass
    return None


def get_default_config():
    """获取默认配置（Get default configuration）"""
    config = load_config_file('lz_hand_rs485_driver', 'hand_config.yaml')
    defaults = {'port': '/dev/ttyUSB0', 'baudrate': 115200, 'feedback_rate': 20.0}
    if config:
        if 'serial' in config and 'baudrate' in config['serial']:
            defaults['baudrate'] = config['serial']['baudrate']
        if 'control' in config and 'feedback_rate' in config['control']:
            defaults['feedback_rate'] = config['control']['feedback_rate']
    return defaults


def parse_hand_id_from_args():
    """从命令行解析hand_id（Parse hand_id from command line）"""
    for arg in sys.argv:
        if arg.startswith('hand_id:='):
            return arg.split(':=', 1)[1]
    return None


def launch_setup(context):
    """启动设置（Launch setup）"""
    defaults = get_default_config()
    hand_id = parse_hand_id_from_args() or '1'
    port_from_config = get_port_from_config(hand_id)
    default_port = port_from_config if port_from_config else defaults['port']
    
    return [
        DeclareLaunchArgument('hand_id', default_value='1', description='手ID：1=右手，2=左手（1=right, 2=left）'),
        DeclareLaunchArgument('port', default_value=default_port, description='串口路径（Serial port）'),
        DeclareLaunchArgument('baudrate', default_value=str(defaults['baudrate']), description='波特率（Baud rate）'),
        DeclareLaunchArgument('feedback_rate', default_value=str(defaults['feedback_rate']), description='反馈频率Hz（Feedback rate）'),
        DeclareLaunchArgument('auto_reconnect', default_value='true', description='自动重连（Auto reconnect）'),
        DeclareLaunchArgument('namespace', default_value='', description='命名空间（Namespace）'),
        
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
    ]


def generate_launch_description():
    """生成启动描述（Generate launch description）"""
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
