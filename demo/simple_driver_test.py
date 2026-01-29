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
    """设置C++驱动的Python绑定路径"""
    import os
    
    # 尝试多个可能的路径
    possible_paths = [
        # 安装路径
        '/home/garry/ros2_ws/install/lz_hand_rs485_driver/lib/lz_hand_rs485_driver',
        # 构建路径 (如果直接从build目录运行)
        '/home/garry/ros2_ws/build/lz_hand_rs485_driver',
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            if path not in sys.path:
                sys.path.insert(0, path)
                print(f"[INFO] 添加驱动路径: {path}")
            return True
    
    print("[ERROR] 找不到C++驱动库，请先编译: colcon build --packages-select lz_hand_rs485_driver")
    return False


def print_separator(title=""):
    """打印分隔线"""
    print("\n" + "=" * 60)
    if title:
        print(f"  {title}")
        print("=" * 60)


def test_connection(driver):
    """测试连接"""
    print_separator("测试1: 连接状态")
    
    if driver.is_connected():
        print("[OK] 驱动已连接")
        return True
    else:
        print("[FAIL] 驱动未连接")
        return False


def test_read_motor_positions(driver):
    """测试读取电机位置"""
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
    """测试读取完整反馈"""
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
    """测试单个关节的弯曲和张开"""
    print_separator(f"测试4: 单个关节控制 - {joint_name} (关节{joint_index})")
    
    # 设置速度为中等
    print(f"[INFO] 设置关节{joint_index}速度为500...")
    if not driver.set_joint_speed(joint_index, 500, False):
        print("[FAIL] 设置速度失败")
        return False
    print("[OK] 速度设置成功")
    
    # 弯曲手指 (位置1000 = 完全弯曲)
    print(f"\n[INFO] 弯曲{joint_name} (位置 -> 1000)...")
    if not driver.set_joint_position(joint_index, 1000):
        print("[FAIL] 设置弯曲位置失败")
        return False
    print("[OK] 弯曲命令发送成功")
    
    # 等待运动完成
    print("[INFO] 等待2秒...")
    time.sleep(2)
    
    # 读取当前位置
    positions = driver.read_motor_positions()
    if positions:
        print(f"[INFO] 当前位置: {positions[joint_index]}")
    
    # 张开手指 (位置0 = 完全张开)
    print(f"\n[INFO] 张开{joint_name} (位置 -> 0)...")
    if not driver.set_joint_position(joint_index, 0):
        print("[FAIL] 设置张开位置失败")
        return False
    print("[OK] 张开命令发送成功")
    
    # 等待运动完成
    print("[INFO] 等待2秒...")
    time.sleep(2)
    
    # 读取当前位置
    positions = driver.read_motor_positions()
    if positions:
        print(f"[INFO] 当前位置: {positions[joint_index]}")
    
    return True


def test_write_verify(driver):
    """测试写入并验证 - 诊断写入问题"""
    print_separator("写入诊断测试")
    
    # 先读取当前控制寄存器状态
    print("[1] 读取当前控制寄存器 (0-17)...")
    state = driver.read_control_state()
    if state:
        positions, speeds, forces = state
        print(f"    位置(0-5):  {list(positions)}")
        print(f"    速度(6-11): {list(speeds)}")
        print(f"    力(12-17):  {list(forces)}")
    else:
        print("[ERROR] 读取控制状态失败")
        return False
    
    # 测试写入位置寄存器0（大拇指翻转）
    test_joint = 2  # 使用食指测试
    test_value = 500
    
    print(f"\n[2] 尝试写入: 关节{test_joint}位置 = {test_value}")
    print(f"    将写入寄存器地址: {test_joint}")
    
    result = driver.set_joint_position(test_joint, test_value)
    print(f"    写入函数返回: {result}")
    
    # 立即读回控制寄存器验证
    print(f"\n[3] 立即读取控制寄存器验证...")
    time.sleep(0.1)  # 短暂等待
    state2 = driver.read_control_state()
    if state2:
        positions2, speeds2, forces2 = state2
        print(f"    位置(0-5):  {list(positions2)}")
        
        if positions2[test_joint] == test_value:
            print(f"[OK] 位置寄存器{test_joint}已更新为{test_value}")
        else:
            print(f"[WARN] 位置寄存器{test_joint}仍为{positions2[test_joint]}，期望{test_value}")
            print("       写入可能没有生效!")
    
    # 测试写入速度
    print(f"\n[4] 尝试写入: 关节{test_joint}速度 = 500")
    print(f"    将写入寄存器地址: {6 + test_joint}")
    result = driver.set_joint_speed(test_joint, 500, False)
    print(f"    写入函数返回: {result}")
    
    # 测试写入力
    print(f"\n[5] 尝试写入: 关节{test_joint}力 = 500")
    print(f"    将写入寄存器地址: {12 + test_joint}")
    result = driver.set_joint_force(test_joint, 500, False)
    print(f"    写入函数返回: {result}")
    
    # 最终验证
    print(f"\n[6] 最终读取控制寄存器...")
    time.sleep(0.1)
    state3 = driver.read_control_state()
    if state3:
        positions3, speeds3, forces3 = state3
        print(f"    位置(0-5):  {list(positions3)}")
        print(f"    速度(6-11): {list(speeds3)}")
        print(f"    力(12-17):  {list(forces3)}")
    
    # 读取电机反馈位置
    print(f"\n[7] 读取电机反馈位置 (寄存器41-46)...")
    motor_pos = driver.read_motor_positions()
    if motor_pos:
        print(f"    电机位置: {list(motor_pos)}")
    
    print("\n" + "=" * 60)
    print("诊断结论:")
    print("  - 如果[3]步骤显示位置没有更新，说明写入没有生效")
    print("  - 可能原因: Modbus功能码不匹配、设备地址错误、写保护等")
    print("=" * 60)
    
    return True


def interactive_mode(driver):
    """交互模式 - 手动控制单个手指"""
    print_separator("交互模式 - 手动控制")
    
    joint_names = [
        "0: 大拇指翻转 (Thumb Rotation)",
        "1: 大拇指弯曲 (Thumb Bend)",
        "2: 食指弯曲 (Index)",
        "3: 中指弯曲 (Middle)",
        "4: 无名指弯曲 (Ring)",
        "5: 小拇指弯曲 (Pinky)"
    ]
    
    print("\n可用命令:")
    print("  p <关节> <位置>  - 设置单个关节位置")
    print("  a <p0> <p1> ... - 一次性设置所有6个位置 (例: a 0 0 500 0 0 0)")
    print("  r               - 读取电机反馈位置 (寄存器41-46)")
    print("  s               - 读取控制状态 (寄存器0-17)")
    print("  w               - 写入诊断测试")
    print("  i               - 初始化: 设置所有速度=500, 力=500")
    print("  o               - 张开所有手指 (set_all_positions全0)")
    print("  c               - 握紧所有手指 (set_all_positions全1000)")
    print("  t <关节>        - 测试单个关节")
    print("  d               - 切换调试模式")
    print("  D               - 切换原始调试模式 (显示所有字节)")
    print("  q               - 退出")
    print("\n关节编号:")
    for name in joint_names:
        print(f"  {name}")
    
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
                    print("电机反馈位置 (寄存器41-46):")
                    for i, pos in enumerate(positions):
                        print(f"  关节{i}: {pos}")
                else:
                    print("[ERROR] 读取失败")
            
            elif cmd == 's':
                state = driver.read_control_state()
                if state:
                    positions, speeds, forces = state
                    print("控制寄存器状态:")
                    print(f"  位置(0-5):   {list(positions)}")
                    print(f"  速度(6-11):  {list(speeds)}")
                    print(f"  力(12-17):   {list(forces)}")
                else:
                    print("[ERROR] 读取失败")
            
            elif cmd == 'w':
                test_write_verify(driver)
            
            elif cmd == 'i':
                print("初始化: 设置所有速度=500, 力=500...")
                speeds = [500, 500, 500, 500, 500, 500]
                forces = [500, 500, 500, 500, 500, 500]
                r1 = driver.set_all_speeds(speeds, False)
                r2 = driver.set_all_forces(forces, False)
                print(f"  速度写入: {'成功' if r1 else '失败'}")
                print(f"  力写入:   {'成功' if r2 else '失败'}")
                # 读回验证
                state = driver.read_control_state()
                if state:
                    _, speeds_rb, forces_rb = state
                    print(f"  验证速度: {list(speeds_rb)}")
                    print(f"  验证力:   {list(forces_rb)}")
            
            elif cmd.startswith('a '):
                parts = cmd.split()
                if len(parts) == 7:  # a + 6个位置值
                    try:
                        positions = [int(parts[i+1]) for i in range(6)]
                        # 检查范围
                        if all(0 <= p <= 1000 for p in positions):
                            print(f"一次性写入所有位置: {positions}")
                            # 先确保速度和力都设置好
                            speeds = [500, 500, 500, 500, 500, 500]
                            forces = [500, 500, 500, 500, 500, 500]
                            driver.set_all_speeds(speeds, False)
                            driver.set_all_forces(forces, False)
                            # 一次性写入所有位置
                            result = driver.set_all_positions(positions)
                            print(f"  写入结果: {'OK' if result else 'FAIL'}")
                            # 验证
                            time.sleep(0.2)
                            state = driver.read_control_state()
                            if state:
                                pos_rb, _, _ = state
                                print(f"  验证位置: {list(pos_rb)}")
                            # 读取电机反馈
                            time.sleep(1)
                            motor = driver.read_motor_positions()
                            if motor:
                                print(f"  电机反馈: {list(motor)}")
                        else:
                            print("[ERROR] 位置值必须在0-1000范围内")
                    except ValueError:
                        print("[ERROR] 参数格式错误")
                else:
                    print("[ERROR] 用法: a <p0> <p1> <p2> <p3> <p4> <p5>")
                    print("       例如: a 0 0 500 0 0 0")
            
            elif cmd == 'o':
                print("张开手指 (所有位置设为0)...")
                speeds = [500, 500, 500, 500, 500, 500]
                forces = [500, 500, 500, 500, 500, 500]
                positions = [0, 0, 0, 0, 0, 0]
                driver.set_all_speeds(speeds, False)
                driver.set_all_forces(forces, False)
                result = driver.set_all_positions(positions)
                print(f"  写入结果: {'OK' if result else 'FAIL'}")
                time.sleep(1)
                motor = driver.read_motor_positions()
                if motor:
                    print(f"  电机反馈: {list(motor)}")
            
            elif cmd == 'c':
                print("握紧手指 (所有位置设为1000)...")
                speeds = [500, 500, 500, 500, 500, 500]
                forces = [500, 500, 500, 500, 500, 500]
                positions = [1000, 1000, 1000, 1000, 1000, 1000]
                driver.set_all_speeds(speeds, False)
                driver.set_all_forces(forces, False)
                result = driver.set_all_positions(positions)
                print(f"  写入结果: {'OK' if result else 'FAIL'}")
                time.sleep(1)
                motor = driver.read_motor_positions()
                if motor:
                    print(f"  电机反馈: {list(motor)}")
            
            elif cmd.startswith('p '):
                parts = cmd.split()
                if len(parts) == 3:
                    try:
                        joint = int(parts[1])
                        pos = int(parts[2])
                        if 0 <= joint <= 5 and 0 <= pos <= 1000:
                            print(f"设置关节{joint}: 速度=500, 力=500, 位置={pos}")
                            # 先设置速度和力
                            r1 = driver.set_joint_speed(joint, 500, False)
                            r2 = driver.set_joint_force(joint, 500, False)
                            r3 = driver.set_joint_position(joint, pos)
                            print(f"  速度写入(寄存器{6+joint}): {'OK' if r1 else 'FAIL'}")
                            print(f"  力写入(寄存器{12+joint}):  {'OK' if r2 else 'FAIL'}")
                            print(f"  位置写入(寄存器{joint}):   {'OK' if r3 else 'FAIL'}")
                            # 读回验证
                            time.sleep(0.1)
                            state = driver.read_control_state()
                            if state:
                                positions, speeds, forces = state
                                print(f"  验证: 位置={positions[joint]}, 速度={speeds[joint]}, 力={forces[joint]}")
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
                # 切换调试模式
                if not hasattr(interactive_mode, 'debug_on'):
                    interactive_mode.debug_on = False
                interactive_mode.debug_on = not interactive_mode.debug_on
                driver.set_debug(interactive_mode.debug_on)
            
            elif cmd == 'D':
                # 切换原始调试模式
                if not hasattr(interactive_mode, 'raw_debug_on'):
                    interactive_mode.raw_debug_on = False
                interactive_mode.raw_debug_on = not interactive_mode.raw_debug_on
                driver.set_raw_debug(interactive_mode.raw_debug_on)
            
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
    
    # 设置驱动路径
    if not setup_driver_path():
        sys.exit(1)
    
    # 导入C++驱动
    try:
        import lz_hand_driver_cpp as driver_lib
        print("[OK] C++驱动库加载成功")
    except ImportError as e:
        print(f"[ERROR] 无法导入C++驱动库: {e}")
        print("请确保已经编译: colcon build --packages-select lz_hand_rs485_driver")
        sys.exit(1)
    
    # 创建驱动实例
    print_separator("连接设备")
    print(f"[INFO] 正在连接到 {args.port}...")
    
    try:
        driver = driver_lib.LZHandModbusDriver(
            args.port,
            args.hand_id,
            args.baudrate,
            True  # auto_connect
        )
        print("[OK] 驱动创建成功")
        
        # 启用调试模式
        if args.debug:
            driver.set_debug(True)
        if args.raw_debug:
            driver.set_raw_debug(True)
            
    except Exception as e:
        print(f"[ERROR] 连接失败: {e}")
        print("\n可能的原因:")
        print("  1. 串口设备不存在或权限不足 (试试: sudo chmod 666 /dev/ttyUSB0)")
        print("  2. 设备未连接或未上电")
        print("  3. 波特率或手ID不正确")
        sys.exit(1)
    
    # 运行测试
    all_passed = True
    
    # 测试1: 连接状态
    if not test_connection(driver):
        all_passed = False
    
    # 测试2: 读取电机位置
    if not test_read_motor_positions(driver):
        all_passed = False
    
    # 测试3: 读取完整反馈
    if not test_read_feedback(driver):
        all_passed = False
    
    # 测试4: 单个关节控制 (如果指定)
    if args.test_joint is not None:
        if 0 <= args.test_joint <= 5:
            joint_names = ["大拇指翻转", "大拇指弯曲", "食指弯曲", "中指弯曲", "无名指弯曲", "小拇指弯曲"]
            if not test_single_joint(driver, args.test_joint, joint_names[args.test_joint]):
                all_passed = False
        else:
            print(f"[ERROR] 无效的关节索引: {args.test_joint} (必须是0-5)")
            all_passed = False
    
    # 进入交互模式 (如果指定)
    if args.interactive:
        interactive_mode(driver)
    
    # 汇总
    print_separator("测试汇总")
    if all_passed:
        print("[OK] 所有基础测试通过!")
        print("\n下一步:")
        print("  1. 使用 --test_joint <0-5> 测试单个手指控制")
        print("  2. 使用 --interactive 进入交互模式")
    else:
        print("[FAIL] 部分测试失败，请检查硬件连接和配置")
    
    # 清理
    driver.disconnect()
    print("\n[INFO] 已断开连接")


if __name__ == '__main__':
    main()
