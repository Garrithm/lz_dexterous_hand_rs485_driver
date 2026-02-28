#!/usr/bin/env python3
"""
灵巧手串口检测工具（LZ Hand Port Detection Tool）
自动扫描USB串口并识别hand_id（1=右手，2=左手）
Automatically scan USB serial ports and identify hand_id (1=right, 2=left)

注意（Note）:
  - 此工具仅用于USB转串口（/dev/ttyUSB*）场景
  - This tool is only for USB-to-Serial (/dev/ttyUSB*) connections
  - 直接485接口用户无需使用此工具，直接在配置文件中指定设备路径即可
  - Direct RS485 users don't need this tool, just configure device path in config file
"""

import sys
import os
import glob
import time


def setup_driver_path():
    """设置C++驱动路径（Setup C++ driver path）
    
    查找策略（按优先级）:
    1. 直接导入检查（如果已 source install/setup.bash）
    2. 脚本所在目录（install 后 .so 与脚本同目录）
    3. AMENT_PREFIX_PATH 环境变量（如果已 source）
    4. 从脚本位置向上查找 workspace 根目录（包含 install/ 或 build/）
    5. 从当前工作目录向上查找 workspace 根目录
    
    适用场景:
    - 脚本在 src/ 下（源码）：向上找 workspace
    - 脚本在 install/ 下（已安装）：检查同目录
    - 从任意目录运行：向上找 workspace
    """
    import importlib.util

    MODULE_NAME = 'lz_hand_driver_cpp'
    PACKAGE_NAME = 'lz_hand_rs485_driver'
    SO_PATTERN = MODULE_NAME + '.cpython-*.so'

    def _try_path(path):
        """尝试将路径加入 sys.path 并验证模块可导入"""
        if not path or not os.path.isdir(path):
            return False
        if not glob.glob(os.path.join(path, SO_PATTERN)):
            return False
        if path not in sys.path:
            sys.path.insert(0, path)
        return importlib.util.find_spec(MODULE_NAME) is not None

    def _find_in_workspace(ws_root):
        """在 workspace 根目录下的 install/ 和 build/ 中查找"""
        for sub in [
            os.path.join('install', PACKAGE_NAME, 'lib', PACKAGE_NAME),
            os.path.join('build', PACKAGE_NAME),
        ]:
            lib_path = os.path.join(ws_root, sub)
            if _try_path(lib_path):
                return True
        return False

    def _walk_up_to_workspace(start_dir, max_levels=10):
        """从 start_dir 向上逐级查找 workspace 根目录（包含 install/ 或 build/）"""
        current = os.path.abspath(start_dir)
        for _ in range(max_levels):
            if os.path.isdir(os.path.join(current, 'install')) or \
               os.path.isdir(os.path.join(current, 'build')):
                if _find_in_workspace(current):
                    return True
            parent = os.path.dirname(current)
            if parent == current:  # 到达根目录
                break
            current = parent
        return False

    # 方法1: 已经可以直接导入（最快路径，如已 source setup.bash）
    if importlib.util.find_spec(MODULE_NAME) is not None:
        return True

    # 方法2: 脚本所在目录（install 后 .so 与脚本同目录）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if _try_path(script_dir):
        return True

    # 方法3: AMENT_PREFIX_PATH（如果已 source install/setup.bash）
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if prefix:
            lib_path = os.path.join(prefix, 'lib', PACKAGE_NAME)
            if _try_path(lib_path):
                return True

    # 方法4: 从脚本位置向上查找 workspace
    if _walk_up_to_workspace(script_dir):
        return True

    # 方法5: 从当前工作目录向上查找 workspace（如果与脚本目录不同）
    cwd = os.getcwd()
    if os.path.abspath(cwd) != os.path.abspath(script_dir):
        if _walk_up_to_workspace(cwd):
            return True

    # 全部失败
    print(f"[ERROR] 找不到C++驱动库 {MODULE_NAME}")
    print("请确保:")
    print(f"  1. 已编译: colcon build --packages-select {PACKAGE_NAME}")
    print("  2. 已source: source install/setup.bash")
    print("  3. 或在 workspace 根目录（包含 install/ 的目录）下运行此脚本")
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
