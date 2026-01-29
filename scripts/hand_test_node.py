#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
灵巧手ROS2测试节点（LZ Hand ROS2 Test Node）

灵巧手驱动交互测试。
Interactive testing for dexterous hand driver.

用法（Usage）:
    ros2 run lz_hand_rs485_driver hand_test_node.py --ros-args -p hand_id:=2
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sys
import threading
import time
from typing import Optional

from sensor_msgs.msg import JointState
from lz_hand_rs485_driver.msg import HandControl, JointControl, HandFeedback, ForceFeedback, MotorFeedback


class LZHandTestNode(Node):
    """灵巧手测试节点（Hand Test Node）"""
    
    # 关节名称（Joint names）
    JOINT_NAMES = ["大拇指翻转", "大拇指弯曲", "食指", "中指", "无名指", "小拇指"]
    
    def __init__(self):
        super().__init__('lz_hand_test')
        
        # 声明参数（Declare parameters）
        self.declare_parameter('hand_id', 1)
        self.declare_parameter('default_speed', 500)
        self.declare_parameter('default_force', 500)
        
        # 加载参数（Load parameters）
        param = self.get_parameter('hand_id').value
        self._hand_id = int(param) if isinstance(param, str) else param
        self._default_speed = self.get_parameter('default_speed').get_parameter_value().integer_value
        self._default_force = self.get_parameter('default_force').get_parameter_value().integer_value
        
        hand_name = '右手（Right）' if self._hand_id == 1 else '左手（Left）'
        self.get_logger().info(f'测试节点（Test node）: {hand_name} ID={self._hand_id}')
        
        # QoS配置（QoS profile）
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        
        # 发布者（Publishers）
        self._hand_pub = self.create_publisher(HandControl, 'hand_control', qos)
        self._joint_pub = self.create_publisher(JointControl, 'joint_control', qos)
        
        # 订阅者（Subscribers）
        self._feedback_sub = self.create_subscription(HandFeedback, 'hand_feedback', lambda m: setattr(self, '_feedback', m), qos)
        self._joint_sub = self.create_subscription(JointState, 'joint_states', lambda m: setattr(self, '_joints', m), qos)
        
        # 状态（State）
        self._feedback: Optional[HandFeedback] = None
        self._joints: Optional[JointState] = None
    
    def send_hand(self, positions, speeds=None, forces=None):
        """发送手部控制（Send hand control）"""
        msg = HandControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.hand_id = self._hand_id
        
        msg.thumb_rotation, msg.thumb_bend = positions[0], positions[1]
        msg.index_bend, msg.middle_bend = positions[2], positions[3]
        msg.ring_bend, msg.pinky_bend = positions[4], positions[5]
        
        s = speeds or [self._default_speed] * 6
        msg.thumb_rotation_speed, msg.thumb_bend_speed = s[0], s[1]
        msg.index_speed, msg.middle_speed, msg.ring_speed, msg.pinky_speed = s[2], s[3], s[4], s[5]
        
        f = forces or [self._default_force] * 6
        msg.thumb_rotation_force, msg.thumb_bend_force = f[0], f[1]
        msg.index_force, msg.middle_force, msg.ring_force, msg.pinky_force = f[2], f[3], f[4], f[5]
        
        self._hand_pub.publish(msg)
        self.get_logger().info(f'发送手部位置（Hand pos）: {positions}')
    
    def send_joint(self, joint, position, speed=0, force=0):
        """发送单关节控制（Send single joint control）"""
        if not 0 <= joint <= 5:
            return
        msg = JointControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.hand_id = self._hand_id
        msg.joint_index = joint
        msg.position = position
        msg.speed = speed if speed > 0 else self._default_speed
        msg.force = force if force > 0 else self._default_force
        self._joint_pub.publish(msg)
        self.get_logger().info(f'发送关节（Joint） {self.JOINT_NAMES[joint]}: {position}')
    
    # 手势（Gestures）
    def open_hand(self): self.send_hand([0, 0, 0, 0, 0, 0])           # 张开（Open）
    def close_hand(self): self.send_hand([1000, 1000, 1000, 1000, 1000, 1000])  # 握拳（Close）
    def thumbs_up(self): self.send_hand([0, 0, 1000, 1000, 1000, 1000])  # 竖大拇指（Thumbs up）
    def point(self): self.send_hand([500, 1000, 0, 1000, 1000, 1000])    # 指向（Point）
    def peace(self): self.send_hand([500, 1000, 0, 0, 1000, 1000])       # 和平（Peace）
    def rock(self): self.send_hand([500, 1000, 0, 1000, 1000, 0])        # 摇滚（Rock）
    def ok(self): self.send_hand([1000, 700, 700, 0, 0, 0])              # OK手势（OK）
    def pinch(self): self.send_hand([1000, 500, 500, 0, 0, 0])           # 捏取（Pinch）
    def three(self): self.send_hand([0, 0, 0, 0, 1000, 1000])            # 三指（Three）
    def four(self): self.send_hand([500, 1000, 0, 0, 0, 0])              # 四指（Four）
    
    def print_feedback(self):
        """打印反馈数据（Print feedback）"""
        if not self._feedback:
            print('\n[暂无反馈数据（No feedback yet）]')
            return
        fb = self._feedback
        print('\n' + '=' * 50)
        print('  手部反馈（Hand Feedback）')
        print('=' * 50)
        print(f'电机位置（Motor pos）: [{fb.thumb_rotation_pos}, {fb.thumb_bend_pos}, {fb.index_bend_pos}, {fb.middle_bend_pos}, {fb.ring_bend_pos}, {fb.pinky_bend_pos}]')
        print('=' * 50)
    
    def print_joints(self):
        """打印关节状态（Print joint states）"""
        if not self._joints:
            print('\n[暂无关节状态（No joint states yet）]')
            return
        print('\n' + '=' * 50)
        print('  关节状态（Joint States）')
        print('=' * 50)
        for i, name in enumerate(self._joints.name[:6]):
            print(f'  {name}: {self._joints.position[i]:.3f}')
        print('=' * 50)


def print_menu():
    """打印菜单（Print menu）"""
    print('\n' + '=' * 55)
    print('  灵巧手测试（LZ Hand Test）')
    print('=' * 55)
    print('  [1] 张开（Open）     [2] 握拳（Close）')
    print('  [3] 指向（Point）    [4] 竖拇指（Thumbs up）')
    print('  [5] OK手势（OK）     [6] 摇滚（Rock）')
    print('  [7] 和平（Peace）    [8] 捏取（Pinch）')
    print('  [9] 三指（Three）    [0] 四指（Four）')
    print('  [d] 演示（Demo）     [f] 反馈（Feedback）')
    print('  [j] 关节（Joints）   [h] 帮助（Help）')
    print('  [p] p <关节> <位置>  [a] a <位置>')
    print('  [m] m p0 p1 p2 p3 p4 p5')
    print('  [q] 退出（Quit）')
    print('=' * 55)


def run_demo(node):
    """运行演示序列（Run demo sequence）"""
    gestures = [node.open_hand, node.close_hand, node.open_hand, node.thumbs_up, 
                node.point, node.peace, node.rock, node.ok, node.pinch, node.open_hand]
    print('\n=== 演示开始（Demo Start） ===')
    for i, g in enumerate(gestures, 1):
        print(f'[{i}/{len(gestures)}]')
        g()
        time.sleep(2)
    print('=== 演示结束（Demo End） ===')


def run_interactive(node, executor):
    """交互模式（Interactive mode）"""
    print_menu()
    
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0.01)
            cmd = input('\n>>> ').strip()
            if not cmd:
                continue
            
            parts = cmd.split()
            c = parts[0].lower()
            
            if c in ['q', 'quit']: break
            elif c == '1': node.open_hand()
            elif c == '2': node.close_hand()
            elif c == '3': node.point()
            elif c == '4': node.thumbs_up()
            elif c == '5': node.ok()
            elif c == '6': node.rock()
            elif c == '7': node.peace()
            elif c == '8': node.pinch()
            elif c == '9': node.three()
            elif c == '0': node.four()
            elif c == 'd': run_demo(node)
            elif c == 'f': node.print_feedback()
            elif c == 'j': node.print_joints()
            elif c == 'h': print_menu()
            elif c == 'p' and len(parts) == 3:
                try:
                    j, p = int(parts[1]), int(parts[2])
                    if 0 <= j <= 5 and 0 <= p <= 1000:
                        node.send_joint(j, p)
                except: print('  格式（Format）: p <0-5> <0-1000>')
            elif c == 'a' and len(parts) == 2:
                try:
                    p = int(parts[1])
                    if 0 <= p <= 1000:
                        node.send_hand([p] * 6)
                except: print('  格式（Format）: a <0-1000>')
            elif c == 'm' and len(parts) == 7:
                try:
                    pos = [int(parts[i+1]) for i in range(6)]
                    if all(0 <= p <= 1000 for p in pos):
                        node.send_hand(pos)
                except: print('  格式（Format）: m p0 p1 p2 p3 p4 p5')
            else:
                print(f"  未知命令（Unknown）: '{cmd}'，输入'h'查看帮助（Type 'h' for help）")
                
        except (EOFError, KeyboardInterrupt):
            break
        except Exception as e:
            print(f'  错误（Error）: {e}')
    
    print('\n已退出（Exited）')


def main(args=None):
    """主入口（Main entry）"""
    rclpy.init(args=args)
    
    try:
        node = LZHandTestNode()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        # 解析命令，跳过ROS2参数（Parse command, skip ROS2 args）
        valid_cmds = ['demo', 'interactive', 'open', 'close']
        cmd = None
        skip = False
        for arg in sys.argv[1:]:
            if skip:
                skip = False
                continue
            if arg in ['--ros-args', '-p', '--param', '--remap']:
                skip = True
                continue
            if arg.startswith('--') or ':=' in arg:
                continue
            if arg in valid_cmds:
                cmd = arg
                break
        
        time.sleep(1.0)
        
        if cmd == 'demo':
            run_demo(node)
        elif cmd == 'open':
            node.open_hand()
            time.sleep(2)
        elif cmd == 'close':
            node.close_hand()
            time.sleep(2)
        else:
            run_interactive(node, executor)
            
    except KeyboardInterrupt:
        pass
    finally:
        if 'executor' in dir():
            executor.shutdown()
        if 'node' in dir():
            node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
