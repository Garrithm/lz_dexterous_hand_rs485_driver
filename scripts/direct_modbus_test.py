#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
直接使用pymodbus写入寄存器测试
Direct Modbus register write test using pymodbus
"""

import sys
import time

# 尝试导入pymodbus 3.x (新版本)
try:
    from pymodbus.client import ModbusSerialClient as ModbusClient
    from pymodbus.exceptions import ModbusException
    PYMODBUS_VERSION = 3
except ImportError:
    # 尝试导入pymodbus 2.x (旧版本)
    try:
        from pymodbus.client.sync import ModbusSerialClient as ModbusClient
        from pymodbus.exceptions import ModbusException
        PYMODBUS_VERSION = 2
    except ImportError:
        print("[ERROR] 未安装pymodbus，请运行: pip install pymodbus")
        sys.exit(1)

def main():
    port = '/dev/ttyUSB0'
    slave_id = 2  # 左手
    baudrate = 115200
    
    print("=" * 60)
    print("直接Modbus写入测试")
    print("=" * 60)
    print(f"串口: {port}")
    print(f"从机地址: {slave_id}")
    print(f"波特率: {baudrate}")
    print()
    
    # 创建Modbus客户端
    print("[1] 创建Modbus客户端...")
    if PYMODBUS_VERSION >= 3:
        # pymodbus 3.x API
        client = ModbusClient(
            port=port,
            baudrate=baudrate,
            timeout=1,
            parity='N',
            stopbits=1,
            bytesize=8,
            framer='rtu'
        )
    else:
        # pymodbus 2.x API
        client = ModbusClient(
            method='rtu',
            port=port,
            baudrate=baudrate,
            timeout=1,
            parity='N',
            stopbits=1,
            bytesize=8
        )
    
    # 连接
    print("[2] 连接串口...")
    if not client.connect():
        print(f"[ERROR] 无法连接到 {port}")
        print("可能的原因:")
        print("  1. 串口不存在或权限不足: sudo chmod 666 /dev/ttyUSB0")
        print("  2. 串口被其他程序占用")
        sys.exit(1)
    print("[OK] 连接成功")
    
    try:
        # 测试读取
        print("\n[3] 测试读取寄存器0-5 (位置控制寄存器)...")
        if PYMODBUS_VERSION >= 3:
            result = client.read_holding_registers(address=0, count=6, device_id=slave_id)
        else:
            result = client.read_holding_registers(0, 6, unit=slave_id)
        if result.isError():
            print(f"[ERROR] 读取失败: {result}")
        else:
            print(f"[OK] 当前位置寄存器值: {result.registers}")
        
        # 读取速度寄存器
        print("\n[4] 读取寄存器6-11 (速度控制寄存器)...")
        if PYMODBUS_VERSION >= 3:
            result = client.read_holding_registers(address=6, count=6, device_id=slave_id)
        else:
            result = client.read_holding_registers(6, 6, unit=slave_id)
        if result.isError():
            print(f"[ERROR] 读取失败: {result}")
        else:
            print(f"[OK] 当前速度寄存器值: {result.registers}")
        
        # 读取力寄存器
        print("\n[5] 读取寄存器12-17 (力控制寄存器)...")
        if PYMODBUS_VERSION >= 3:
            result = client.read_holding_registers(address=12, count=6, device_id=slave_id)
        else:
            result = client.read_holding_registers(12, 6, unit=slave_id)
        if result.isError():
            print(f"[ERROR] 读取失败: {result}")
        else:
            print(f"[OK] 当前力寄存器值: {result.registers}")
        
        # 写入速度寄存器（确保速度不是0）
        print("\n[6] 写入速度寄存器6-11 = 500...")
        if PYMODBUS_VERSION >= 3:
            result = client.write_registers(address=6, values=[500, 500, 500, 500, 500, 500], device_id=slave_id)
        else:
            result = client.write_registers(6, [500, 500, 500, 500, 500, 500], unit=slave_id)
        if result.isError():
            print(f"[ERROR] 写入速度失败: {result}")
        else:
            print("[OK] 速度写入成功")
            time.sleep(0.1)
            # 验证
            if PYMODBUS_VERSION >= 3:
                result2 = client.read_holding_registers(address=6, count=6, device_id=slave_id)
            else:
                result2 = client.read_holding_registers(6, 6, unit=slave_id)
            if not result2.isError():
                print(f"    验证: {result2.registers}")
        
        # 写入力寄存器
        print("\n[7] 写入力寄存器12-17 = 500...")
        if PYMODBUS_VERSION >= 3:
            result = client.write_registers(address=12, values=[500, 500, 500, 500, 500, 500], device_id=slave_id)
        else:
            result = client.write_registers(12, [500, 500, 500, 500, 500, 500], unit=slave_id)
        if result.isError():
            print(f"[ERROR] 写入力失败: {result}")
        else:
            print("[OK] 力写入成功")
            time.sleep(0.1)
            # 验证
            if PYMODBUS_VERSION >= 3:
                result2 = client.read_holding_registers(address=12, count=6, device_id=slave_id)
            else:
                result2 = client.read_holding_registers(12, 6, unit=slave_id)
            if not result2.isError():
                print(f"    验证: {result2.registers}")
        
        # 写入位置寄存器 - 先设为0
        print("\n[8] 写入位置寄存器0-5 = 全0 (张开手指)...")
        if PYMODBUS_VERSION >= 3:
            result = client.write_registers(address=0, values=[0, 0, 0, 0, 0, 0], device_id=slave_id)
        else:
            result = client.write_registers(0, [0, 0, 0, 0, 0, 0], unit=slave_id)
        if result.isError():
            print(f"[ERROR] 写入位置失败: {result}")
        else:
            print("[OK] 位置写入成功 (全0)")
            print("    等待2秒，观察手指是否张开...")
            time.sleep(2)
            # 验证
            if PYMODBUS_VERSION >= 3:
                result2 = client.read_holding_registers(address=0, count=6, device_id=slave_id)
            else:
                result2 = client.read_holding_registers(0, 6, unit=slave_id)
            if not result2.isError():
                print(f"    验证位置: {result2.registers}")
            # 读取电机反馈
            if PYMODBUS_VERSION >= 3:
                result3 = client.read_holding_registers(address=41, count=6, device_id=slave_id)
            else:
                result3 = client.read_holding_registers(41, 6, unit=slave_id)
            if not result3.isError():
                print(f"    电机反馈位置: {result3.registers}")
        
        # 写入位置寄存器 - 食指弯曲
        print("\n[9] 写入位置寄存器2 = 500 (食指弯曲)...")
        if PYMODBUS_VERSION >= 3:
            result = client.write_register(address=2, value=500, device_id=slave_id)
        else:
            result = client.write_register(2, 500, unit=slave_id)
        if result.isError():
            print(f"[ERROR] 写入位置失败: {result}")
        else:
            print("[OK] 位置写入成功 (食指=500)")
            print("    等待2秒，观察食指是否弯曲...")
            time.sleep(2)
            # 验证
            if PYMODBUS_VERSION >= 3:
                result2 = client.read_holding_registers(address=0, count=6, device_id=slave_id)
            else:
                result2 = client.read_holding_registers(0, 6, unit=slave_id)
            if not result2.isError():
                print(f"    验证位置: {result2.registers}")
            # 读取电机反馈
            if PYMODBUS_VERSION >= 3:
                result3 = client.read_holding_registers(address=41, count=6, device_id=slave_id)
            else:
                result3 = client.read_holding_registers(41, 6, unit=slave_id)
            if not result3.isError():
                print(f"    电机反馈位置: {result3.registers}")
        
        # 写入位置寄存器 - 食指完全弯曲
        print("\n[10] 写入位置寄存器2 = 1000 (食指完全弯曲)...")
        if PYMODBUS_VERSION >= 3:
            result = client.write_register(address=2, value=1000, device_id=slave_id)
        else:
            result = client.write_register(2, 1000, unit=slave_id)
        if result.isError():
            print(f"[ERROR] 写入位置失败: {result}")
        else:
            print("[OK] 位置写入成功 (食指=1000)")
            print("    等待2秒，观察食指是否完全弯曲...")
            time.sleep(2)
            # 验证
            if PYMODBUS_VERSION >= 3:
                result2 = client.read_holding_registers(address=0, count=6, device_id=slave_id)
            else:
                result2 = client.read_holding_registers(0, 6, unit=slave_id)
            if not result2.isError():
                print(f"    验证位置: {result2.registers}")
            # 读取电机反馈
            if PYMODBUS_VERSION >= 3:
                result3 = client.read_holding_registers(address=41, count=6, device_id=slave_id)
            else:
                result3 = client.read_holding_registers(41, 6, unit=slave_id)
            if not result3.isError():
                print(f"    电机反馈位置: {result3.registers}")
        
        # 批量写入所有位置
        print("\n[11] 批量写入所有位置 = [0, 0, 500, 0, 0, 0]...")
        if PYMODBUS_VERSION >= 3:
            result = client.write_registers(address=0, values=[0, 0, 500, 0, 0, 0], device_id=slave_id)
        else:
            result = client.write_registers(0, [0, 0, 500, 0, 0, 0], unit=slave_id)
        if result.isError():
            print(f"[ERROR] 批量写入失败: {result}")
        else:
            print("[OK] 批量写入成功")
            print("    等待2秒，观察手指动作...")
            time.sleep(2)
            # 验证
            if PYMODBUS_VERSION >= 3:
                result2 = client.read_holding_registers(address=0, count=6, device_id=slave_id)
            else:
                result2 = client.read_holding_registers(0, 6, unit=slave_id)
            if not result2.isError():
                print(f"    验证位置: {result2.registers}")
            # 读取电机反馈
            if PYMODBUS_VERSION >= 3:
                result3 = client.read_holding_registers(address=41, count=6, device_id=slave_id)
            else:
                result3 = client.read_holding_registers(41, 6, unit=slave_id)
            if not result3.isError():
                print(f"    电机反馈位置: {result3.registers}")
        
        print("\n" + "=" * 60)
        print("测试完成！")
        print("=" * 60)
        print("\n如果手指仍然不动，可能的原因:")
        print("  1. 设备需要额外的使能信号")
        print("  2. 设备处于锁定/安全模式")
        print("  3. 需要同时写入所有18个寄存器（位置+速度+力）")
        print("  4. 硬件故障或供电问题")
        
    except ModbusException as e:
        print(f"[ERROR] Modbus异常: {e}")
    except Exception as e:
        print(f"[ERROR] 其他错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n[清理] 关闭连接...")
        client.close()
        print("[OK] 已关闭")

if __name__ == '__main__':
    main()
