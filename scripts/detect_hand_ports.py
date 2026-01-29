#!/usr/bin/env python3
"""
灵巧手串口检测工具（LZ Hand Port Detection Tool）
自动扫描USB串口并识别hand_id（1=右手，2=左手）
Automatically scan USB serial ports and identify hand_id (1=right, 2=left)
"""

import sys
import os
import glob
import time


def setup_driver_path():
    """设置C++驱动路径（Setup C++ driver path）"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    possible_paths = [script_dir]
    
    if 'src' in script_dir or 'scripts' in script_dir:
        package_dir = os.path.dirname(script_dir)
        possible_paths.extend([
            os.path.join(package_dir, '..', '..', 'install', 'lz_hand_rs485_driver', 'lib', 'lz_hand_rs485_driver'),
            os.path.join(package_dir, '..', '..', 'build', 'lz_hand_rs485_driver'),
        ])
    
    for path in possible_paths:
        abs_path = os.path.abspath(path)
        if os.path.exists(abs_path):
            module_file = os.path.join(abs_path, 'lz_hand_driver_cpp.cpython-*.so')
            if glob.glob(module_file) or os.path.exists(abs_path):
                if abs_path not in sys.path:
                    sys.path.insert(0, abs_path)
                return True
    
    print("[ERROR] 找不到C++驱动库，请先编译: colcon build --packages-select lz_hand_rs485_driver")
    return False


def find_serial_ports():
    """查找所有USB串口（Find all USB serial ports）"""
    ports = []
    for port in glob.glob('/dev/ttyUSB*'):
        if os.path.exists(port):
            ports.append(port)
    return sorted(ports)


def test_hand_id(port, hand_id, baudrate=115200):
    """测试串口和hand_id是否匹配（Test if port matches hand_id）"""
    try:
        import lz_hand_driver_cpp as driver_lib
        driver = driver_lib.LZHandModbusDriver(port, hand_id, baudrate, True)
        if not driver.is_connected():
            return False
        
        time.sleep(0.1)
        positions = driver.read_motor_positions()
        driver.disconnect()
        return positions is not None and len(positions) == 6
    except Exception:
        return False


def detect_all_hands():
    """检测所有连接的灵巧手（Detect all connected hands）"""
    if not setup_driver_path():
        return None
    
    try:
        import lz_hand_driver_cpp as driver_lib
    except ImportError as e:
        print(f"[ERROR] 无法导入C++驱动库: {e}")
        return None
    
    ports = find_serial_ports()
    if not ports:
        print("[INFO] 未找到USB串口设备（No USB serial ports found）")
        return {}
    
    print(f"[INFO] 检测到 {len(ports)} 个串口设备（Found {len(ports)} serial port(s)）")
    
    results = {}
    for port in ports:
        if not os.access(port, os.R_OK | os.W_OK):
            results[port] = {'hand_id': None, 'error': 'permission_denied'}
            continue
        
        if test_hand_id(port, 1):
            results[port] = {'hand_id': 1}
            continue
        
        if test_hand_id(port, 2):
            results[port] = {'hand_id': 2}
            continue
        
        results[port] = {'hand_id': None, 'error': 'not_found'}
    
    found_hands = {}
    for port, result in results.items():
        if result.get('hand_id'):
            hand_id = result['hand_id']
            hand_name = '右手' if hand_id == 1 else '左手'
            print(f"{port}: hand_id={hand_id} ({hand_name})")
            found_hands[port] = hand_id
        elif result.get('error') == 'permission_denied':
            print(f"{port}: 权限不足（Permission denied）")
        else:
            print(f"{port}: 未检测到设备（No device detected）")
    
    if found_hands:
        print("\n配置文件建议（Config suggestion）:")
        print("更新 config/hand_config.yaml 中的 serial 部分:")
        print("serial:")
        for port, hand_id in sorted(found_hands.items(), key=lambda x: x[1]):
            if hand_id == 1:
                print(f"  right_hand_port: \"{port}\"")
            elif hand_id == 2:
                print(f"  left_hand_port: \"{port}\"")
    
    return results


if __name__ == '__main__':
    detect_all_hands()
