#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
灵巧手ROS2驱动节点（LZ Hand ROS2 Driver Node）

通过RS485 Modbus-RTU协议控制灵巧手。
Controls dexterous hand via RS485 Modbus-RTU.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import threading
from typing import Optional
import math
import time

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from lz_hand_rs485_driver.msg import (
    HandControl, JointControl, HandFeedback,
    JointFeedback, ForceFeedback, MotorFeedback,
)
try:
    from lz_hand_driver_cpp import (
        LZHandModbusDriver, HandConstants, RegisterMap,
        HandID, JointIndex, ModbusError,
    )
except ImportError:
    # 回退: 尝试自动查找C++驱动库（Fallback: auto-find C++ driver）
    import sys
    import os
    import glob
    import importlib.util

    _MODULE = 'lz_hand_driver_cpp'
    _PKG = 'lz_hand_rs485_driver'
    _SO = _MODULE + '.cpython-*.so'

    def _try_add(p):
        """尝试将路径加入 sys.path 并验证模块可导入"""
        if not p or not os.path.isdir(p):
            return False
        if not glob.glob(os.path.join(p, _SO)):
            return False
        if p not in sys.path:
            sys.path.insert(0, p)
        return importlib.util.find_spec(_MODULE) is not None

    def _find_in_workspace(ws_root):
        """在 workspace 根目录下的 install/ 和 build/ 中查找"""
        for sub in [
            os.path.join('install', _PKG, 'lib', _PKG),
            os.path.join('build', _PKG),
        ]:
            if _try_add(os.path.join(ws_root, sub)):
                return True
        return False

    def _walk_up_to_workspace(start_dir, max_levels=10):
        """从 start_dir 向上逐级查找 workspace 根目录"""
        current = os.path.abspath(start_dir)
        for _ in range(max_levels):
            if os.path.isdir(os.path.join(current, 'install')) or \
               os.path.isdir(os.path.join(current, 'build')):
                if _find_in_workspace(current):
                    return True
            parent = os.path.dirname(current)
            if parent == current:
                break
            current = parent
        return False

    # 查找驱动库
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 1. 脚本所在目录（install 后 .so 同目录）
    if _try_add(script_dir):
        pass  # 找到，继续导入
    # 2. AMENT_PREFIX_PATH
    elif any(_try_add(os.path.join(pfx, 'lib', _PKG)) 
             for pfx in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if pfx):
        pass  # 找到，继续导入
    # 3. 从脚本位置向上找 workspace
    elif _walk_up_to_workspace(script_dir):
        pass  # 找到，继续导入
    # 4. 从 CWD 向上找 workspace
    elif os.path.abspath(os.getcwd()) != os.path.abspath(script_dir) and _walk_up_to_workspace(os.getcwd()):
        pass  # 找到，继续导入
    else:
        raise ImportError(
            f"无法找到C++驱动库 {_MODULE}。"
            f"请先编译: colcon build --packages-select {_PKG} "
            f"然后: source install/setup.bash"
        )

    from lz_hand_driver_cpp import (
        LZHandModbusDriver, HandConstants, RegisterMap,
        HandID, JointIndex, ModbusError,
    )


class LZHandDriverNode(Node):
    """灵巧手ROS2驱动节点（LZ Hand ROS2 Driver Node）"""
    
    def __init__(self):
        super().__init__('lz_hand_driver')
        
        # 声明和加载参数（Declare and load parameters）
        self._declare_parameters()
        self._load_parameters()
        
        # 初始化驱动（Initialize driver）
        self._driver: Optional[LZHandModbusDriver] = None
        self._connected = False
        self._lock = threading.Lock()
        
        # 重连状态（Reconnection state）
        self._reconnect_attempts = 0
        self._last_reconnect_time = 0.0
        self._consecutive_errors = 0
        self._last_feedback_time = 0.0
        self._min_feedback_interval = 0.05  # 最小反馈间隔50ms（Min feedback interval）
        self._permission_error = False  # 权限错误标志（Permission error flag）
        self._write_pending = False  # 写优先标志（Write-priority flag）
        
        # QoS配置（QoS profile）
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 设置发布者和订阅者（Setup publishers and subscribers）
        self._setup_publishers(qos)
        self._setup_subscribers(qos)
        
        # 等待串口稳定（Wait for serial port to settle）
        time.sleep(0.5)
        if not self._connect():
            # 如果是权限错误，直接退出
            if self._permission_error:
                raise SystemExit()
        
        # 创建定时器（Create timers）
        self._feedback_timer = self.create_timer(1.0 / self._feedback_rate, self._feedback_callback)
        self._reconnect_timer = self.create_timer(5.0, self._reconnect_callback)
        
        hand_name = '右手' if self._hand_id == 1 else '左手'
        self.get_logger().info(f'驱动节点已初始化（Driver initialized）- {hand_name} ID={self._hand_id}')
    
    def _declare_parameters(self):
        """声明ROS2参数（Declare ROS2 parameters）"""
        # 串口配置（Serial port config）
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        
        # 手配置（Hand config）
        self.declare_parameter('hand_id', 1)
        
        # 话题名称（Topic names）
        self.declare_parameter('hand_control_topic', 'hand_control')
        self.declare_parameter('joint_control_topic', 'joint_control')
        self.declare_parameter('hand_feedback_topic', 'hand_feedback')
        self.declare_parameter('joint_feedback_topic', 'joint_feedback')
        self.declare_parameter('force_feedback_topic', 'force_feedback')
        self.declare_parameter('motor_feedback_topic', 'motor_feedback')
        self.declare_parameter('joint_states_topic', 'joint_states')
        
        # 更新频率（Update rate）
        self.declare_parameter('feedback_rate', 20.0)
        self.declare_parameter('gradual_step_size', 100)
        self.declare_parameter('frame_id', '')
        
        # 重连参数（Reconnection parameters）
        self.declare_parameter('auto_reconnect', True)
        self.declare_parameter('max_reconnect_attempts', 10)
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('max_consecutive_errors', 5)
    
    def _get_param_int(self, name):
        """获取整数参数（Get integer parameter）"""
        p = self.get_parameter(name)
        try:
            v = p.get_parameter_value().integer_value
            return v if v != 0 else int(p.get_parameter_value().string_value)
        except:
            return int(p.get_parameter_value().string_value)
    
    def _get_param_float(self, name):
        """获取浮点参数（Get float parameter）"""
        p = self.get_parameter(name)
        try:
            return p.get_parameter_value().double_value
        except:
            return float(p.get_parameter_value().string_value)
    
    def _get_param_bool(self, name):
        """获取布尔参数（Get bool parameter）"""
        p = self.get_parameter(name)
        try:
            return p.get_parameter_value().bool_value
        except:
            return p.get_parameter_value().string_value.lower() == 'true'
    
    def _load_parameters(self):
        """加载ROS2参数（Load ROS2 parameters）"""
        self._port = self.get_parameter('port').get_parameter_value().string_value
        self._baudrate = self._get_param_int('baudrate')
        self._timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self._hand_id = self._get_param_int('hand_id')
        
        self._hand_control_topic = self.get_parameter('hand_control_topic').get_parameter_value().string_value
        self._joint_control_topic = self.get_parameter('joint_control_topic').get_parameter_value().string_value
        self._hand_feedback_topic = self.get_parameter('hand_feedback_topic').get_parameter_value().string_value
        self._joint_feedback_topic = self.get_parameter('joint_feedback_topic').get_parameter_value().string_value
        self._force_feedback_topic = self.get_parameter('force_feedback_topic').get_parameter_value().string_value
        self._motor_feedback_topic = self.get_parameter('motor_feedback_topic').get_parameter_value().string_value
        self._joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        
        self._feedback_rate = self._get_param_float('feedback_rate')
        self._gradual_step_size = self._get_param_int('gradual_step_size')
        
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._frame_id = frame_id if frame_id else ('right_hand_link' if self._hand_id == 1 else 'left_hand_link')
        
        self._auto_reconnect = self._get_param_bool('auto_reconnect')
        self._max_reconnect_attempts = self._get_param_int('max_reconnect_attempts')
        self._reconnect_interval = self._get_param_float('reconnect_interval')
        self._max_consecutive_errors = self._get_param_int('max_consecutive_errors')
        
        hand_name = '右手' if self._hand_id == 1 else '左手'
        self.get_logger().info(f'配置（Config）: port={self._port}, baudrate={self._baudrate}, hand={hand_name}(ID={self._hand_id})')
    
    def _setup_publishers(self, qos):
        """设置发布者（Setup publishers）"""
        self._hand_feedback_pub = self.create_publisher(HandFeedback, self._hand_feedback_topic, qos)
        self._joint_feedback_pub = self.create_publisher(JointFeedback, self._joint_feedback_topic, qos)
        self._force_feedback_pub = self.create_publisher(ForceFeedback, self._force_feedback_topic, qos)
        self._motor_feedback_pub = self.create_publisher(MotorFeedback, self._motor_feedback_topic, qos)
        self._joint_states_pub = self.create_publisher(JointState, self._joint_states_topic, qos)
    
    def _setup_subscribers(self, qos):
        """设置订阅者（Setup subscribers）"""
        self._hand_control_sub = self.create_subscription(HandControl, self._hand_control_topic, self._hand_control_callback, qos)
        self._joint_control_sub = self.create_subscription(JointControl, self._joint_control_topic, self._joint_control_callback, qos)
    
    def _connect(self) -> bool:
        """连接灵巧手硬件（Connect to hand hardware）"""
        hand_name = '右手' if self._hand_id == 1 else '左手'
        
        try:
            if self._driver is not None:
                try:
                    self._driver.disconnect()
                except:
                    pass
                self._driver = None
                time.sleep(0.2)
            
            self.get_logger().info(f'正在连接（Connecting）{hand_name}: {self._port}, ID={self._hand_id}')
            self._driver = LZHandModbusDriver(self._port, self._hand_id, self._baudrate, True)
            
            if self._driver.is_connected():
                self._connected = True
                self._consecutive_errors = 0
                self._reconnect_attempts = 0
                self._driver.set_gradual_step_size(self._gradual_step_size)
                positions = self._driver.read_motor_positions()
                if positions:
                    self.get_logger().info(f'连接成功（Connected），电机位置: {list(positions)}')
                else:
                    self.get_logger().info(f'已连接（Connected）{hand_name}')
                return True
            else:
                self.get_logger().error(f'连接失败（Connection failed）: {self._port}')
                self._connected = False
                return False
                
        except Exception as e:
            self.get_logger().error(f'连接错误（Connection error）: {e}')
            self._permission_error = True
            self._connected = False
            return False
    
    def _reconnect_callback(self):
        """自动重连定时回调（Auto-reconnection timer callback）"""
        if self._permission_error:
            rclpy.shutdown()
            return
        
        if not self._auto_reconnect:
            return
        
        if self._connected and self._driver and self._driver.is_connected():
            return
        
        if self._consecutive_errors >= self._max_consecutive_errors:
            self._connected = False
        
        if not self._connected:
            current_time = time.time()
            if current_time - self._last_reconnect_time < self._reconnect_interval:
                return
            
            self._last_reconnect_time = current_time
            self._reconnect_attempts += 1
            
            if self._reconnect_attempts <= self._max_reconnect_attempts:
                self.get_logger().info(f'尝试重连（Reconnecting）({self._reconnect_attempts}/{self._max_reconnect_attempts})...')
                if self._connect():
                    self.get_logger().info('重连成功（Reconnected）!')
            elif self._reconnect_attempts == self._max_reconnect_attempts + 1:
                self.get_logger().error(f'已达最大重连次数（Max reconnect attempts reached）({self._max_reconnect_attempts})')
    
    def _hand_control_callback(self, msg: HandControl):
        """处理手部控制消息（Handle hand control message）"""
        if not self._connected:
            return
        
        if msg.hand_id != 0 and msg.hand_id != self._hand_id:
            return
        
        self._write_pending = True
        
        with self._lock:
            try:
                if self._driver is None or not self._driver.is_connected():
                    self._consecutive_errors += 1
                    return
                
                positions = [msg.thumb_rotation, msg.thumb_bend, msg.index_bend, 
                           msg.middle_bend, msg.ring_bend, msg.pinky_bend]
                
                speeds = [s if s > 0 else 500 for s in [
                    msg.thumb_rotation_speed, msg.thumb_bend_speed, msg.index_speed,
                    msg.middle_speed, msg.ring_speed, msg.pinky_speed]]
                
                forces = [f if f > 0 else 500 for f in [
                    msg.thumb_rotation_force, msg.thumb_bend_force, msg.index_force,
                    msg.middle_force, msg.ring_force, msg.pinky_force]]
                
                self._driver.set_all_speeds(speeds, False)
                self._driver.set_all_forces(forces, False)
                self._driver.set_all_positions(positions)
                self._consecutive_errors = 0
                
            except Exception as e:
                self._consecutive_errors += 1
                self.get_logger().error(f'控制错误（Control error）: {e}')
            finally:
                self._write_pending = False
    
    def _joint_control_callback(self, msg: JointControl):
        """处理单关节控制消息（Handle single joint control message）"""
        if not self._connected:
            return
        
        if msg.hand_id != 0 and msg.hand_id != self._hand_id:
            return
        
        if msg.joint_index > 5:
            return
        
        self._write_pending = True
        
        with self._lock:
            try:
                if self._driver is None or not self._driver.is_connected():
                    self._consecutive_errors += 1
                    return
                
                self._driver.set_joint_speed(msg.joint_index, msg.speed if msg.speed > 0 else 500, False)
                self._driver.set_joint_force(msg.joint_index, msg.force if msg.force > 0 else 500, False)
                self._driver.set_joint_position(msg.joint_index, msg.position)
                self._consecutive_errors = 0
                
            except Exception as e:
                self._consecutive_errors += 1
                self.get_logger().error(f'关节控制错误（Joint control error）: {e}')
            finally:
                self._write_pending = False
    
    def _read_feedback_batch(self):
        """一次读29个寄存器（Single read of 29 registers）"""
        all_fb = self._driver.read_all_feedback()
        if all_fb is None:
            return None
        force_values = list(all_fb.force_values)
        force_valid = list(all_fb.force_valid)
        return {
            'force_feedback': force_values[:10],
            'force_valid': force_valid[:10],
            'palm_forces': force_values[10:13],
            'palm_valid': force_valid[10:13],
            'joint_angles': list(all_fb.joint_angles),
            'motor_positions': list(all_fb.motor_positions),
        }
    
    def _read_feedback_individual(self):
        """分3次读，作为回退（Fallback: 3 separate reads）"""
        motor_positions = self._driver.read_motor_positions()
        if motor_positions is None:
            return None
        force_result = self._driver.read_force_feedback()
        if force_result:
            force_values, force_valid = list(force_result[0]), list(force_result[1])
        else:
            force_values, force_valid = [0] * 13, [False] * 13
        joint_angles = self._driver.read_joint_angles()
        if joint_angles is None:
            joint_angles = [0.0] * 10
        return {
            'force_feedback': force_values[:10],
            'force_valid': force_valid[:10],
            'palm_forces': force_values[10:13],
            'palm_valid': force_valid[10:13],
            'joint_angles': list(joint_angles),
            'motor_positions': list(motor_positions),
        }
    
    def _feedback_callback(self):
        """读取并发布反馈数据（Read and publish feedback）"""
        if not self._connected:
            return
        
        if self._write_pending:
            return
        
        current_time = time.time()
        if current_time - self._last_feedback_time < self._min_feedback_interval:
            return
        
        with self._lock:
            try:
                if self._driver is None or not self._driver.is_connected():
                    self._consecutive_errors += 1
                    return
                
                self._last_feedback_time = current_time
                
                # 优先一次读，失败则回退分次读（Batch read, fallback to individual）
                feedback = self._read_feedback_batch()
                if feedback is None:
                    feedback = self._read_feedback_individual()
                if feedback is None:
                    self._consecutive_errors += 1
                    return
                
                self._consecutive_errors = 0
                
                now = self.get_clock().now().to_msg()
                self._publish_hand_feedback(feedback, now)
                self._publish_force_feedback(feedback, now)
                self._publish_motor_feedback(feedback, now)
                self._publish_joint_states(feedback, now)
                
            except Exception as e:
                self._consecutive_errors += 1
    
    def _publish_hand_feedback(self, fb: dict, stamp):
        """发布完整手部反馈（Publish complete hand feedback）"""
        msg = HandFeedback()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.hand_id = self._hand_id
        
        mp = fb['motor_positions']
        msg.thumb_rotation_pos, msg.thumb_bend_pos = mp[0], mp[1]
        msg.index_bend_pos, msg.middle_bend_pos = mp[2], mp[3]
        msg.ring_bend_pos, msg.pinky_bend_pos = mp[4], mp[5]
        
        angles = fb['joint_angles']
        msg.thumb_mid_angle, msg.thumb_root_angle = int(angles[0]*10), int(angles[1]*10)
        msg.index_mid_angle, msg.index_root_angle = int(angles[2]*10), int(angles[3]*10)
        msg.middle_mid_angle, msg.middle_root_angle = int(angles[4]*10), int(angles[5]*10)
        msg.ring_mid_angle, msg.ring_root_angle = int(angles[6]*10), int(angles[7]*10)
        msg.pinky_mid_angle, msg.pinky_root_angle = int(angles[8]*10), int(angles[9]*10)
        
        forces, valid = fb['force_feedback'], fb['force_valid']
        msg.thumb_tip_force, msg.thumb_mid_force = forces[0], forces[1]
        msg.index_tip_force, msg.index_mid_force = forces[2], forces[3]
        msg.middle_tip_force, msg.middle_mid_force = forces[4], forces[5]
        msg.ring_tip_force, msg.ring_mid_force = forces[6], forces[7]
        msg.pinky_tip_force, msg.pinky_mid_force = forces[8], forces[9]
        
        msg.thumb_tip_force_valid, msg.thumb_mid_force_valid = valid[0], valid[1]
        msg.index_tip_force_valid, msg.index_mid_force_valid = valid[2], valid[3]
        msg.middle_tip_force_valid, msg.middle_mid_force_valid = valid[4], valid[5]
        msg.ring_tip_force_valid, msg.ring_mid_force_valid = valid[6], valid[7]
        msg.pinky_tip_force_valid, msg.pinky_mid_force_valid = valid[8], valid[9]
        
        palm, palm_valid = fb['palm_forces'], fb['palm_valid']
        msg.palm_force_1, msg.palm_force_2, msg.palm_force_3 = palm[0], palm[1], palm[2]
        msg.palm_force_1_valid, msg.palm_force_2_valid, msg.palm_force_3_valid = palm_valid[0], palm_valid[1], palm_valid[2]
        
        self._hand_feedback_pub.publish(msg)
    
    def _publish_force_feedback(self, fb: dict, stamp):
        """发布力反馈（Publish force feedback）"""
        msg = ForceFeedback()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.hand_id = self._hand_id
        
        forces, palm = fb['force_feedback'], fb['palm_forces']
        msg.thumb_tip, msg.thumb_mid = forces[0], forces[1]
        msg.index_tip, msg.index_mid = forces[2], forces[3]
        msg.middle_tip, msg.middle_mid = forces[4], forces[5]
        msg.ring_tip, msg.ring_mid = forces[6], forces[7]
        msg.pinky_tip, msg.pinky_mid = forces[8], forces[9]
        msg.palm_1, msg.palm_2, msg.palm_3 = palm[0], palm[1], palm[2]
        msg.valid_flags = fb['force_valid'] + fb['palm_valid']
        
        self._force_feedback_pub.publish(msg)
    
    def _publish_motor_feedback(self, fb: dict, stamp):
        """发布电机位置反馈（Publish motor position feedback）"""
        msg = MotorFeedback()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.hand_id = self._hand_id
        
        mp = fb['motor_positions']
        msg.thumb_rotation, msg.thumb_bend = mp[0], mp[1]
        msg.index_bend, msg.middle_bend = mp[2], mp[3]
        msg.ring_bend, msg.pinky_bend = mp[4], mp[5]
        
        self._motor_feedback_pub.publish(msg)
    
    def _publish_joint_states(self, fb: dict, stamp):
        """发布标准ROS关节状态（Publish standard ROS joint states）"""
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        
        prefix = 'right_' if self._hand_id == 1 else 'left_'
        motor_names = [f'{prefix}{j}_joint' for j in ['thumb_rotation', 'thumb_bend', 'index_bend', 'middle_bend', 'ring_bend', 'pinky_bend']]
        angle_names = [f'{prefix}{j}_joint' for j in ['thumb_mid', 'thumb_root', 'index_mid', 'index_root', 'middle_mid', 'middle_root', 'ring_mid', 'ring_root', 'pinky_mid', 'pinky_root']]
        
        msg.name = motor_names + angle_names
        msg.position = [p / 1000.0 for p in fb['motor_positions']] + [math.radians(a) for a in fb['joint_angles']]
        msg.velocity = []
        msg.effort = []
        
        self._joint_states_pub.publish(msg)
    
    def destroy_node(self):
        """清理节点（Cleanup node）"""
        if hasattr(self, '_feedback_timer'):
            self._feedback_timer.cancel()
        if hasattr(self, '_reconnect_timer'):
            self._reconnect_timer.cancel()
        if self._driver:
            try:
                self._driver.disconnect()
            except:
                pass
        super().destroy_node()


def main(args=None):
    """主入口（Main entry）"""
    rclpy.init(args=args)
    node = LZHandDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
