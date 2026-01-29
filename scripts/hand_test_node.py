#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LZ Hand Test Node for ROS2
灵巧手ROS2测试节点

Provides interactive testing of the dexterous hand driver.
Publishes test commands and displays feedback.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sys
import threading
from typing import Optional

# ROS2 messages
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

# Custom messages
from lz_hand_rs485_driver.msg import (
    HandControl,
    JointControl,
    HandFeedback,
    ForceFeedback,
    MotorFeedback,
)


class LZHandTestNode(Node):
    """
    灵巧手测试节点
    Test node for the hand driver
    """
    
    def __init__(self):
        """Initialize the test node"""
        super().__init__('lz_hand_test')
        
        # Parameters
        self.declare_parameter('hand_id', 1)
        # Handle both string and int types from launch file
        param_value = self.get_parameter('hand_id').value
        self._hand_id = int(param_value) if isinstance(param_value, str) else param_value
        self.get_logger().info(f'Test node using hand_id={self._hand_id}')
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self._hand_control_pub = self.create_publisher(
            HandControl,
            'hand_control',
            qos_profile
        )
        self._joint_control_pub = self.create_publisher(
            JointControl,
            'joint_control',
            qos_profile
        )
        
        # Subscribers
        self._hand_feedback_sub = self.create_subscription(
            HandFeedback,
            'hand_feedback',
            self._hand_feedback_callback,
            qos_profile
        )
        self._force_feedback_sub = self.create_subscription(
            ForceFeedback,
            'force_feedback',
            self._force_feedback_callback,
            qos_profile
        )
        self._joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self._joint_states_callback,
            qos_profile
        )
        
        # State
        self._latest_feedback: Optional[HandFeedback] = None
        self._latest_force: Optional[ForceFeedback] = None
        self._latest_joint_states: Optional[JointState] = None
        
        self.get_logger().info('LZ Hand Test Node initialized')
    
    def _hand_feedback_callback(self, msg: HandFeedback):
        """Store latest hand feedback"""
        self._latest_feedback = msg
    
    def _force_feedback_callback(self, msg: ForceFeedback):
        """Store latest force feedback"""
        self._latest_force = msg
    
    def _joint_states_callback(self, msg: JointState):
        """Store latest joint states"""
        self._latest_joint_states = msg
    
    def send_hand_control(
        self,
        positions: list,
        speeds: list = None,
        forces: list = None
    ):
        """
        Send hand control command
        发送手部控制命令
        """
        msg = HandControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.hand_id = self._hand_id
        
        # Positions
        msg.thumb_rotation = positions[0]
        msg.thumb_bend = positions[1]
        msg.index_bend = positions[2]
        msg.middle_bend = positions[3]
        msg.ring_bend = positions[4]
        msg.pinky_bend = positions[5]
        
        # Speeds
        if speeds:
            msg.thumb_rotation_speed = speeds[0]
            msg.thumb_bend_speed = speeds[1]
            msg.index_speed = speeds[2]
            msg.middle_speed = speeds[3]
            msg.ring_speed = speeds[4]
            msg.pinky_speed = speeds[5]
        
        # Forces
        if forces:
            msg.thumb_rotation_force = forces[0]
            msg.thumb_bend_force = forces[1]
            msg.index_force = forces[2]
            msg.middle_force = forces[3]
            msg.ring_force = forces[4]
            msg.pinky_force = forces[5]
        
        self._hand_control_pub.publish(msg)
        self.get_logger().info(f'Sent hand control: positions={positions}')
    
    def send_joint_control(
        self,
        joint_index: int,
        position: int,
        speed: int = 0,
        force: int = 0
    ):
        """
        Send single joint control command
        发送单关节控制命令
        """
        msg = JointControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.hand_id = self._hand_id
        msg.joint_index = joint_index
        msg.position = position
        msg.speed = speed
        msg.force = force
        
        self._joint_control_pub.publish(msg)
        self.get_logger().info(
            f'Sent joint control: joint={joint_index}, position={position}'
        )
    
    def open_hand(self, speed: int = 500):
        """Open the hand"""
        self.get_logger().info('Opening hand...')
        positions = [0, 0, 0, 0, 0, 0]
        speeds = [speed] * 6
        self.send_hand_control(positions, speeds)
    
    def close_hand(self, speed: int = 500, force: int = 500):
        """Close the hand"""
        self.get_logger().info('Closing hand...')
        positions = [1000, 1000, 1000, 1000, 1000, 1000]
        speeds = [speed] * 6
        forces = [force] * 6
        self.send_hand_control(positions, speeds, forces)
    
    def pinch_grip(self, strength: int = 500):
        """Pinch grip gesture"""
        self.get_logger().info('Pinch grip...')
        positions = [1000, 500, 500, 0, 0, 0]
        forces = [strength] * 6
        self.send_hand_control(positions, forces=forces)
    
    def point_gesture(self):
        """Point gesture"""
        self.get_logger().info('Point gesture...')
        positions = [500, 1000, 0, 1000, 1000, 1000]
        self.send_hand_control(positions)
    
    def ok_gesture(self):
        """OK gesture"""
        self.get_logger().info('OK gesture...')
        positions = [1000, 700, 700, 0, 0, 0]
        self.send_hand_control(positions)
    
    def thumbs_up(self):
        """Thumbs up gesture"""
        self.get_logger().info('Thumbs up...')
        positions = [0, 0, 1000, 1000, 1000, 1000]
        self.send_hand_control(positions)
    
    def rock_gesture(self):
        """Rock gesture (index and pinky extended)"""
        self.get_logger().info('Rock gesture...')
        positions = [500, 1000, 0, 1000, 1000, 0]
        self.send_hand_control(positions)
    
    def peace_gesture(self):
        """Peace gesture (index and middle extended)"""
        self.get_logger().info('Peace gesture...')
        positions = [500, 1000, 0, 0, 1000, 1000]
        self.send_hand_control(positions)
    
    def print_feedback(self):
        """Print current feedback"""
        if self._latest_feedback:
            fb = self._latest_feedback
            print('\n=== Hand Feedback ===')
            print(f'Motor Positions: '
                  f'Thumb-R={fb.thumb_rotation_pos}, '
                  f'Thumb-B={fb.thumb_bend_pos}, '
                  f'Index={fb.index_bend_pos}, '
                  f'Middle={fb.middle_bend_pos}, '
                  f'Ring={fb.ring_bend_pos}, '
                  f'Pinky={fb.pinky_bend_pos}')
            print(f'Force (Thumb): tip={fb.thumb_tip_force}g '
                  f'({"valid" if fb.thumb_tip_force_valid else "invalid"}), '
                  f'mid={fb.thumb_mid_force}g '
                  f'({"valid" if fb.thumb_mid_force_valid else "invalid"})')
        else:
            print('No feedback received yet')
    
    def print_joint_states(self):
        """Print current joint states"""
        if self._latest_joint_states:
            js = self._latest_joint_states
            print('\n=== Joint States ===')
            for i, name in enumerate(js.name):
                if i < len(js.position):
                    print(f'  {name}: {js.position[i]:.4f}')
        else:
            print('No joint states received yet')


def run_demo(node: LZHandTestNode):
    """Run a demonstration sequence"""
    import time
    
    node.get_logger().info('Starting demo sequence...')
    
    gestures = [
        ('Open hand', node.open_hand, []),
        ('Close hand', node.close_hand, []),
        ('Open hand', node.open_hand, []),
        ('Thumbs up', node.thumbs_up, []),
        ('Point', node.point_gesture, []),
        ('Peace', node.peace_gesture, []),
        ('Rock', node.rock_gesture, []),
        ('OK', node.ok_gesture, []),
        ('Pinch', node.pinch_grip, []),
        ('Open hand', node.open_hand, []),
    ]
    
    for name, func, args in gestures:
        node.get_logger().info(f'Demo: {name}')
        func(*args)
        time.sleep(2.0)  # Wait for movement
    
    node.get_logger().info('Demo completed')


def print_menu():
    """Print the interactive menu"""
    print('\n' + '='*50)
    print('  LZ Hand Interactive Test (ROS2)')
    print('='*50)
    print('  手势控制:')
    print('    [1] 张开手 (Open hand)')
    print('    [2] 握拳 (Close hand)')
    print('    [3] 指向手势 (Point gesture)')
    print('    [4] 竖大拇指 (Thumbs up)')
    print('    [5] OK手势 (OK gesture)')
    print('    [6] 摇滚手势 (Rock gesture)')
    print('    [7] 和平手势 (Peace gesture)')
    print('    [8] 捏取手势 (Pinch grip)')
    print('  功能:')
    print('    [9] 运行演示 (Run demo)')
    print('    [f] 显示反馈 (Print feedback)')
    print('    [j] 显示关节状态 (Print joint states)')
    print('  高级控制:')
    print('    [s] 设置单个关节位置')
    print('    [a] 设置所有关节位置')
    print('    [q] 退出 (Quit)')
    print('='*50)


def run_interactive(node: LZHandTestNode, executor):
    """Run interactive mode with menu selection"""
    print_menu()
    
    while rclpy.ok():
        try:
            # Process pending callbacks in a non-blocking way
            executor.spin_once(timeout_sec=0.01)
            
            # Use standard input (works better with ROS2 launch)
            try:
                cmd = input('\n>>> 请选择操作 (输入数字/字母, h=帮助, q=退出): ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                print('\n\n收到中断信号，退出中...')
                break
            
            if not cmd:
                continue
            
            # Handle command
            if cmd == 'q':
                print('\n退出中...')
                break
            elif cmd == '1' or cmd == 'o':
                node.open_hand()
                print('✓ 已发送: 张开手')
            elif cmd == '2' or cmd == 'c':
                node.close_hand()
                print('✓ 已发送: 握拳')
            elif cmd == '3' or cmd == 'p':
                node.point_gesture()
                print('✓ 已发送: 指向手势')
            elif cmd == '4' or cmd == 't':
                node.thumbs_up()
                print('✓ 已发送: 竖大拇指')
            elif cmd == '5' or cmd == 'k':
                node.ok_gesture()
                print('✓ 已发送: OK手势')
            elif cmd == '6' or cmd == 'r':
                node.rock_gesture()
                print('✓ 已发送: 摇滚手势')
            elif cmd == '7' or cmd == 'v':
                node.peace_gesture()
                print('✓ 已发送: 和平手势')
            elif cmd == '8' or cmd == 'g':
                node.pinch_grip()
                print('✓ 已发送: 捏取手势')
            elif cmd == '9' or cmd == 'd':
                print('\n开始运行演示序列...')
                run_demo(node)
                print('演示完成')
            elif cmd == 'f':
                node.print_feedback()
            elif cmd == 'j':
                node.print_joint_states()
            elif cmd == 's':
                try:
                    joint_str = input('  请输入关节索引 (0-5): ').strip()
                    pos_str = input('  请输入位置 (0-1000): ').strip()
                    joint = int(joint_str)
                    pos = int(pos_str)
                    if 0 <= joint <= 5 and 0 <= pos <= 1000:
                        node.send_joint_control(joint, pos)
                        print(f'✓ 已设置关节 {joint} 位置为 {pos}')
                    else:
                        print('✗ 无效范围: 关节 0-5, 位置 0-1000')
                except (ValueError, KeyboardInterrupt, EOFError):
                    print('✗ 输入无效或已取消')
            elif cmd == 'a':
                try:
                    pos_str = input('  请输入所有关节位置 (0-1000): ').strip()
                    pos = int(pos_str)
                    if 0 <= pos <= 1000:
                        node.send_hand_control([pos] * 6)
                        print(f'✓ 已设置所有关节位置为 {pos}')
                    else:
                        print('✗ 无效范围: 位置 0-1000')
                except (ValueError, KeyboardInterrupt, EOFError):
                    print('✗ 输入无效或已取消')
            elif cmd == 'h' or cmd == '?':
                print_menu()
            elif cmd:
                print(f"✗ 未知命令: '{cmd}'. 输入 'h' 查看菜单, 'q' 退出")
                
        except EOFError:
            break
        except KeyboardInterrupt:
            print('\n\n收到中断信号，退出中...')
            break
        except Exception as e:
            print(f'✗ 错误: {e}')
            import traceback
            traceback.print_exc()
    
    print('\n已退出交互模式')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = None
    executor = None
    spin_thread = None
    
    try:
        node = LZHandTestNode()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        
        # Start a thread for spinning
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        # Check command line arguments
        if len(sys.argv) > 1:
            cmd = sys.argv[1]
            if cmd == 'demo':
                import time
                time.sleep(1.0)  # Wait for connections
                run_demo(node)
            elif cmd == 'interactive':
                import time
                time.sleep(1.0)
                run_interactive(node, executor)
            elif cmd == 'open':
                import time
                time.sleep(1.0)
                node.open_hand()
                time.sleep(2.0)
            elif cmd == 'close':
                import time
                time.sleep(1.0)
                node.close_hand()
                time.sleep(2.0)
            else:
                print(f'Unknown command: {cmd}')
                print('Usage: hand_test_node.py [demo|interactive|open|close]')
        else:
            # Default to interactive mode
            import time
            time.sleep(1.0)
            run_interactive(node, executor)
            
    except KeyboardInterrupt:
        print('\n收到中断信号，正在关闭...')
    except Exception as e:
        print(f'错误: {e}')
        import traceback
        traceback.print_exc()
    finally:
        # Clean shutdown
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass  # Ignore if already shut down


if __name__ == '__main__':
    main()
