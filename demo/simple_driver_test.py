#!/usr/bin/env python3
"""
简单的驱动测试程序 - 不依赖ROS
Simple Driver Test - No ROS dependency

用于测试:
1. 串口通讯是否正常
2. C++驱动是否工作
3. 单个手指控制是否正常

使用方法:
    python3 simple_driver_test.py --port /dev/ttyUSB0 --hand_id 2
"""

import sys
import time
import argparse

# 添加C++绑定库的路径
def setup_driver_path():
    """设置C++驱动路径（Setup C++ driver path）"""
    import os
    import glob
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    possible_paths = [script_dir]
    
    if 'src' in script_dir or 'demo' in script_dir:
        package_dir = os.path.dirname(os.path.dirname(script_dir))
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


def print_separator(title=""):
    """打印分隔线（Print separator）"""
    print("\n" + "=" * 60)
    if title:
        print(f"  {title}")
        print("=" * 60)


def test_connection(driver):
    """测试连接（Test connection）"""
    print_separator("测试1: 连接状态")
    if driver.is_connected():
        print("[OK] 驱动已连接")
        return True
    else:
        print("[FAIL] 驱动未连接")
        return False


def test_read_motor_positions(driver):
    """测试读取电机位置（Test read motor positions）"""
    print_separator("测试2: 读取电机位置")
    positions = driver.read_motor_positions()
    if positions is not None:
        print("[OK] 读取电机位置成功:")
        joint_names = ["大拇指翻转", "大拇指弯曲", "食指弯曲", "中指弯曲", "无名指弯曲", "小拇指弯曲"]
        for i, (name, pos) in enumerate(zip(joint_names, positions)):
            print(f"    关节{i} ({name}): {pos}")
        return True
    else:
        print("[FAIL] 读取电机位置失败")
        return False


def test_read_feedback(driver):
    """测试读取完整反馈（Test read feedback）"""
    print_separator("测试3: 读取完整反馈数据")
    feedback = driver.read_all_feedback()
    if feedback is not None:
        print("[OK] 读取反馈数据成功:")
        print(f"    电机位置: {list(feedback.motor_positions)}")
        print(f"    关节角度: {[round(a, 1) for a in feedback.joint_angles]}")
        print(f"    力反馈值: {list(feedback.force_values)}")
        return True
    else:
        print("[FAIL] 读取反馈数据失败")
        return False


def test_single_joint(driver, joint_index, joint_name):
    """测试单个关节控制（Test single joint control）"""
    print_separator(f"测试4: 单个关节控制 - {joint_name} (关节{joint_index})")
    
    print(f"[INFO] 设置关节{joint_index}速度为500...")
    if not driver.set_joint_speed(joint_index, 500, False):
        print("[FAIL] 设置速度失败")
        return False
    print("[OK] 速度设置成功")
    
    print(f"\n[INFO] 弯曲{joint_name} (位置 -> 1000)...")
    if not driver.set_joint_position(joint_index, 1000):
        print("[FAIL] 设置弯曲位置失败")
        return False
    print("[OK] 弯曲命令发送成功")
    
    time.sleep(2)
    positions = driver.read_motor_positions()
    if positions:
        print(f"[INFO] 当前位置: {positions[joint_index]}")
    
    print(f"\n[INFO] 张开{joint_name} (位置 -> 0)...")
    if not driver.set_joint_position(joint_index, 0):
        print("[FAIL] 设置张开位置失败")
        return False
    print("[OK] 张开命令发送成功")
    
    time.sleep(2)
    positions = driver.read_motor_positions()
    if positions:
        print(f"[INFO] 当前位置: {positions[joint_index]}")
    
    return True


def test_write_verify(driver):
    """测试写入并验证（Test write and verify）"""
    print_separator("写入诊断测试")
    state = driver.read_control_state()
    if not state:
        print("[ERROR] 读取控制状态失败")
        return False
    
    positions, speeds, forces = state
    print(f"当前位置: {list(positions)}")
    print(f"当前速度: {list(speeds)}")
    print(f"当前力: {list(forces)}")
    
    test_joint = 2
    test_value = 500
    
    print(f"\n写入关节{test_joint}位置={test_value}")
    result = driver.set_joint_position(test_joint, test_value)
    print(f"写入结果: {'成功' if result else '失败'}")
    
    time.sleep(0.1)
    state2 = driver.read_control_state()
    if state2:
        positions2, _, _ = state2
        if positions2[test_joint] == test_value:
            print(f"[OK] 位置已更新")
        else:
            print(f"[WARN] 位置未更新，当前值: {positions2[test_joint]}")
    
    return True


def interactive_mode(driver):
    """交互模式（Interactive mode）"""
    print_separator("交互模式")
    
    joint_names = ["大拇指翻转", "大拇指弯曲", "食指弯曲", "中指弯曲", "无名指弯曲", "小拇指弯曲"]
    
    print("\n可用命令:")
    print("  p <关节> <位置>  - 设置单个关节位置 (0-5, 0-1000)")
    print("  a <p0> <p1> ... - 设置所有6个位置")
    print("  r               - 读取电机反馈位置")
    print("  s               - 读取控制状态")
    print("  w               - 写入诊断测试")
    print("  i               - 初始化速度/力")
    print("  o               - 张开所有手指")
    print("  c               - 握紧所有手指")
    print("  t <关节>        - 测试单个关节")
    print("  d/D             - 切换调试模式")
    print("  q               - 退出")
    print("\n关节编号:")
    for i, name in enumerate(joint_names):
        print(f"  {i}: {name}")
    
    while True:
        try:
            cmd = input("\n>>> 请输入命令: ").strip().lower()
            
            if not cmd:
                continue
            
            if cmd == 'q':
                print("退出交互模式")
                break
            
            elif cmd == 'r':
                positions = driver.read_motor_positions()
                if positions:
                    print("电机反馈位置:")
                    for i, pos in enumerate(positions):
                        print(f"  关节{i}: {pos}")
                else:
                    print("[ERROR] 读取失败")
            
            elif cmd == 's':
                state = driver.read_control_state()
                if state:
                    positions, speeds, forces = state
                    print(f"位置: {list(positions)}")
                    print(f"速度: {list(speeds)}")
                    print(f"力: {list(forces)}")
                else:
                    print("[ERROR] 读取失败")
            
            elif cmd == 'w':
                test_write_verify(driver)
            
            elif cmd == 'i':
                print("初始化速度/力...")
                speeds = [500] * 6
                forces = [500] * 6
                r1 = driver.set_all_speeds(speeds, False)
                r2 = driver.set_all_forces(forces, False)
                print(f"速度: {'成功' if r1 else '失败'}, 力: {'成功' if r2 else '失败'}")
            
            elif cmd.startswith('a '):
                parts = cmd.split()
                if len(parts) == 7:
                    try:
                        positions = [int(parts[i+1]) for i in range(6)]
                        if all(0 <= p <= 1000 for p in positions):
                            print(f"写入所有位置: {positions}")
                            speeds = [500] * 6
                            forces = [500] * 6
                            driver.set_all_speeds(speeds, False)
                            driver.set_all_forces(forces, False)
                            result = driver.set_all_positions(positions)
                            print(f"结果: {'成功' if result else '失败'}")
                        else:
                            print("[ERROR] 位置值必须在0-1000范围内")
                    except ValueError:
                        print("[ERROR] 参数格式错误")
                else:
                    print("[ERROR] 用法: a <p0> <p1> <p2> <p3> <p4> <p5>")
            
            elif cmd == 'o':
                print("张开手指...")
                speeds = [500] * 6
                forces = [500] * 6
                positions = [0] * 6
                driver.set_all_speeds(speeds, False)
                driver.set_all_forces(forces, False)
                result = driver.set_all_positions(positions)
                print(f"结果: {'成功' if result else '失败'}")
            
            elif cmd == 'c':
                print("握紧手指...")
                speeds = [500] * 6
                forces = [500] * 6
                positions = [1000] * 6
                driver.set_all_speeds(speeds, False)
                driver.set_all_forces(forces, False)
                result = driver.set_all_positions(positions)
                print(f"结果: {'成功' if result else '失败'}")
            
            elif cmd.startswith('p '):
                parts = cmd.split()
                if len(parts) == 3:
                    try:
                        joint = int(parts[1])
                        pos = int(parts[2])
                        if 0 <= joint <= 5 and 0 <= pos <= 1000:
                            print(f"设置关节{joint}位置={pos}")
                            driver.set_joint_speed(joint, 500, False)
                            driver.set_joint_force(joint, 500, False)
                            result = driver.set_joint_position(joint, pos)
                            print(f"结果: {'成功' if result else '失败'}")
                        else:
                            print("[ERROR] 参数超出范围 (关节: 0-5, 位置: 0-1000)")
                    except ValueError:
                        print("[ERROR] 参数格式错误")
                else:
                    print("[ERROR] 用法: p <关节> <位置>")
            
            elif cmd.startswith('t '):
                parts = cmd.split()
                if len(parts) == 2:
                    try:
                        joint = int(parts[1])
                        if 0 <= joint <= 5:
                            test_single_joint(driver, joint, f"关节{joint}")
                        else:
                            print("[ERROR] 关节编号必须在 0-5 之间")
                    except ValueError:
                        print("[ERROR] 参数格式错误")
                else:
                    print("[ERROR] 用法: t <关节>")
            
            elif cmd == 'd':
                if not hasattr(interactive_mode, 'debug_on'):
                    interactive_mode.debug_on = False
                interactive_mode.debug_on = not interactive_mode.debug_on
                driver.set_debug(interactive_mode.debug_on)
                print(f"调试模式: {'开启' if interactive_mode.debug_on else '关闭'}")
            
            elif cmd == 'D':
                if not hasattr(interactive_mode, 'raw_debug_on'):
                    interactive_mode.raw_debug_on = False
                interactive_mode.raw_debug_on = not interactive_mode.raw_debug_on
                driver.set_raw_debug(interactive_mode.raw_debug_on)
                print(f"原始调试模式: {'开启' if interactive_mode.raw_debug_on else '关闭'}")
            
            else:
                print("[ERROR] 未知命令，输入 'q' 退出")
        
        except KeyboardInterrupt:
            print("\n退出交互模式")
            break


def main():
    parser = argparse.ArgumentParser(description='LZ灵巧手驱动简单测试程序 (无ROS依赖)')
    parser.add_argument('--port', '-p', type=str, default='/dev/ttyUSB0',
                        help='串口设备路径 (默认: /dev/ttyUSB0)')
    parser.add_argument('--hand_id', '-i', type=int, default=2,
                        help='机械手ID: 1=右手, 2=左手 (默认: 2)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                        help='波特率 (默认: 115200)')
    parser.add_argument('--interactive', '-I', action='store_true',
                        help='进入交互模式')
    parser.add_argument('--test_joint', '-j', type=int, default=None,
                        help='测试指定关节 (0-5)')
    parser.add_argument('--debug', '-d', action='store_true',
                        help='启用调试模式 (显示Modbus操作)')
    parser.add_argument('--raw_debug', '-D', action='store_true',
                        help='启用原始调试模式 (显示所有Modbus字节)')
    args = parser.parse_args()
    
    print_separator("LZ灵巧手驱动测试程序")
    print(f"串口: {args.port}")
    print(f"手ID: {args.hand_id} ({'左手' if args.hand_id == 2 else '右手'})")
    print(f"波特率: {args.baudrate}")
    
    if not setup_driver_path():
        sys.exit(1)
    
    try:
        import lz_hand_driver_cpp as driver_lib
    except ImportError as e:
        print(f"[ERROR] 无法导入C++驱动库: {e}")
        print("请确保已经编译: colcon build --packages-select lz_hand_rs485_driver")
        sys.exit(1)
    
    print_separator("连接设备")
    try:
        driver = driver_lib.LZHandModbusDriver(args.port, args.hand_id, args.baudrate, True)
        if args.debug:
            driver.set_debug(True)
        if args.raw_debug:
            driver.set_raw_debug(True)
    except Exception as e:
        print(f"[ERROR] 连接失败: {e}")
        sys.exit(1)
    
    all_passed = True
    
    if not test_connection(driver):
        all_passed = False
    
    if not test_read_motor_positions(driver):
        all_passed = False
    
    if not test_read_feedback(driver):
        all_passed = False
    
    if args.test_joint is not None:
        if 0 <= args.test_joint <= 5:
            joint_names = ["大拇指翻转", "大拇指弯曲", "食指弯曲", "中指弯曲", "无名指弯曲", "小拇指弯曲"]
            if not test_single_joint(driver, args.test_joint, joint_names[args.test_joint]):
                all_passed = False
        else:
            print(f"[ERROR] 无效的关节索引: {args.test_joint} (必须是0-5)")
            all_passed = False
    
    if args.interactive:
        interactive_mode(driver)
    
    print_separator("测试汇总")
    if all_passed:
        print("[OK] 所有基础测试通过!")
    else:
        print("[FAIL] 部分测试失败，请检查硬件连接和配置")
    
    driver.disconnect()
    print("\n[INFO] 已断开连接")


if __name__ == '__main__':
    main()
