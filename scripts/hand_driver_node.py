#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LZ Hand ROS2 Driver Node
灵巧手ROS2驱动节点

Main ROS2 node for controlling the dexterous hand via RS485 Modbus-RTU.
Provides topics for control input and feedback output.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import threading
from typing import Optional
import math

# ROS2 messages
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

# Custom messages - will be generated
from lz_hand_rs485_driver.msg import (
    HandControl,
    JointControl,
    HandFeedback,
    JointFeedback,
    ForceFeedback,
    MotorFeedback,
)

# Import C++ driver bindings 
from lz_hand_driver_cpp import (
    LZHandModbusDriver,
    HandConstants,
    RegisterMap,
    HandID,
    JointIndex,
    ModbusError,
)


class LZHandDriverNode(Node):
    """
    灵巧手ROS2驱动节点
    LZ Hand ROS2 Driver Node
    
    Subscribes to control topics and publishes feedback data.
    """
    
    def __init__(self):
        """Initialize the ROS2 node"""
        super().__init__('lz_hand_driver')
        
        # Declare and load parameters
        self._declare_parameters()
        self._load_parameters()
        
        # Initialize driver
        self._driver: Optional[LZHandModbusDriver] = None
        self._connected = False
        
        # Thread lock
        self._lock = threading.Lock()
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Setup publishers
        self._setup_publishers(qos_profile)
        
        # Setup subscribers
        self._setup_subscribers(qos_profile)
        
        # Connect to hardware
        self._connect()
        
        # Create feedback timer
        timer_period = 1.0 / self._feedback_rate
        self._feedback_timer = self.create_timer(
            timer_period,
            self._feedback_callback
        )
        
        self.get_logger().info('LZ Hand Driver Node initialized')
    
    def _declare_parameters(self):
        """Declare ROS2 parameters"""
        # Serial port configuration
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        
        # Hand configuration
        self.declare_parameter('hand_id', 1)  # 1=right, 2=left
        
        # Topic names
        self.declare_parameter('hand_control_topic', 'hand_control')
        self.declare_parameter('joint_control_topic', 'joint_control')
        self.declare_parameter('hand_feedback_topic', 'hand_feedback')
        self.declare_parameter('joint_feedback_topic', 'joint_feedback')
        self.declare_parameter('force_feedback_topic', 'force_feedback')
        self.declare_parameter('motor_feedback_topic', 'motor_feedback')
        self.declare_parameter('joint_states_topic', 'joint_states')
        
        # Update rates
        self.declare_parameter('feedback_rate', 50.0)
        
        # Control parameters
        self.declare_parameter('gradual_step_size', 100)
        
        # Frame ID
        self.declare_parameter('frame_id', '')
    
    def _load_parameters(self):
        """Load ROS2 parameters"""
        self._port = self.get_parameter('port').get_parameter_value().string_value
        self._baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self._timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        self._hand_id = self.get_parameter('hand_id').get_parameter_value().integer_value
        
        self._hand_control_topic = self.get_parameter('hand_control_topic').get_parameter_value().string_value
        self._joint_control_topic = self.get_parameter('joint_control_topic').get_parameter_value().string_value
        self._hand_feedback_topic = self.get_parameter('hand_feedback_topic').get_parameter_value().string_value
        self._joint_feedback_topic = self.get_parameter('joint_feedback_topic').get_parameter_value().string_value
        self._force_feedback_topic = self.get_parameter('force_feedback_topic').get_parameter_value().string_value
        self._motor_feedback_topic = self.get_parameter('motor_feedback_topic').get_parameter_value().string_value
        self._joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        
        self._feedback_rate = self.get_parameter('feedback_rate').get_parameter_value().double_value
        self._gradual_step_size = self.get_parameter('gradual_step_size').get_parameter_value().integer_value
        
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        if not frame_id:
            self._frame_id = 'right_hand_link' if self._hand_id == 1 else 'left_hand_link'
        else:
            self._frame_id = frame_id
        
        self.get_logger().info(
            f'Configuration: port={self._port}, baudrate={self._baudrate}, hand_id={self._hand_id}'
        )
    
    def _setup_publishers(self, qos_profile):
        """Setup ROS2 publishers"""
        # Complete hand feedback
        self._hand_feedback_pub = self.create_publisher(
            HandFeedback,
            self._hand_feedback_topic,
            qos_profile
        )
        
        # Individual joint feedback
        self._joint_feedback_pub = self.create_publisher(
            JointFeedback,
            self._joint_feedback_topic,
            qos_profile
        )
        
        # Force feedback
        self._force_feedback_pub = self.create_publisher(
            ForceFeedback,
            self._force_feedback_topic,
            qos_profile
        )
        
        # Motor feedback
        self._motor_feedback_pub = self.create_publisher(
            MotorFeedback,
            self._motor_feedback_topic,
            qos_profile
        )
        
        # Standard joint states
        self._joint_states_pub = self.create_publisher(
            JointState,
            self._joint_states_topic,
            qos_profile
        )
        
        self.get_logger().info('Publishers initialized')
    
    def _setup_subscribers(self, qos_profile):
        """Setup ROS2 subscribers"""
        # Hand control (full hand)
        self._hand_control_sub = self.create_subscription(
            HandControl,
            self._hand_control_topic,
            self._hand_control_callback,
            qos_profile
        )
        
        # Joint control (single joint)
        self._joint_control_sub = self.create_subscription(
            JointControl,
            self._joint_control_topic,
            self._joint_control_callback,
            qos_profile
        )
        
        self.get_logger().info('Subscribers initialized')
    
    def _connect(self):
        """Connect to the hand hardware"""
        try:
            # C++ driver: port, hand_id, baudrate, auto_connect
            self._driver = LZHandModbusDriver(
                self._port,
                self._hand_id,
                self._baudrate,
                True  # auto_connect
            )
            
            if self._driver.is_connected():
                self._connected = True
                self._driver.set_gradual_step_size(self._gradual_step_size)
                self.get_logger().info(f'Connected to hand on {self._port}')
            else:
                self.get_logger().error(f'Failed to connect to hand on {self._port}')
                
        except Exception as e:
            self.get_logger().error(f'Connection error: {e}')
            self._connected = False
    
    def _hand_control_callback(self, msg: HandControl):
        """
        Handle hand control messages
        处理手部控制消息
        """
        if not self._connected:
            self.get_logger().warn('Not connected to hand, ignoring control command')
            return
        
        # Check hand_id matches
        if msg.hand_id != 0 and msg.hand_id != self._hand_id:
            return  # Message not for this hand
        
        with self._lock:
            try:
                # Set positions
                positions = [
                    msg.thumb_rotation,
                    msg.thumb_bend,
                    msg.index_bend,
                    msg.middle_bend,
                    msg.ring_bend,
                    msg.pinky_bend,
                ]
                
                # Set speeds if non-zero
                speeds = [
                    msg.thumb_rotation_speed,
                    msg.thumb_bend_speed,
                    msg.index_speed,
                    msg.middle_speed,
                    msg.ring_speed,
                    msg.pinky_speed,
                ]
                if any(s > 0 for s in speeds):
                    self._driver.set_all_speeds(speeds)
                
                # Set forces if non-zero
                forces = [
                    msg.thumb_rotation_force,
                    msg.thumb_bend_force,
                    msg.index_force,
                    msg.middle_force,
                    msg.ring_force,
                    msg.pinky_force,
                ]
                if any(f > 0 for f in forces):
                    self._driver.set_all_forces(forces)
                
                # Set positions
                self._driver.set_all_positions(positions)
                
                self.get_logger().debug(f'Set hand pose: {positions}')
                
            except ModbusError as e:
                self.get_logger().error(f'Modbus error in hand control: {e}')
    
    def _joint_control_callback(self, msg: JointControl):
        """
        Handle single joint control messages
        处理单关节控制消息
        """
        if not self._connected:
            self.get_logger().warn('Not connected to hand, ignoring control command')
            return
        
        # Check hand_id matches
        if msg.hand_id != 0 and msg.hand_id != self._hand_id:
            return
        
        if msg.joint_index > 5:
            self.get_logger().warn(f'Invalid joint index: {msg.joint_index}')
            return
        
        with self._lock:
            try:
                # Set speed if specified
                if msg.speed > 0:
                    self._driver.set_joint_speed(msg.joint_index, msg.speed)
                
                # Set force if specified
                if msg.force > 0:
                    self._driver.set_joint_force(msg.joint_index, msg.force)
                
                # Set position
                self._driver.set_joint_position(msg.joint_index, msg.position)
                
                self.get_logger().debug(
                    f'Set joint {msg.joint_index} to position {msg.position}'
                )
                
            except ModbusError as e:
                self.get_logger().error(f'Modbus error in joint control: {e}')
    
    def _feedback_callback(self):
        """
        Timer callback for reading and publishing feedback
        定时读取并发布反馈数据
        """
        if not self._connected:
            return
        
        with self._lock:
            try:
                # Read all feedback at once (C++ returns FeedbackData struct)
                feedback_opt = self._driver.read_all_feedback()
                if feedback_opt is None:
                    self.get_logger().warn('Failed to read feedback')
                    return
                
                # Convert FeedbackData struct to dict for compatibility
                feedback = {
                    'force_feedback': list(feedback_opt.force_values)[:10],
                    'force_valid': list(feedback_opt.force_valid)[:10],
                    'palm_forces': list(feedback_opt.force_values)[10:13],
                    'palm_valid': list(feedback_opt.force_valid)[10:13],
                    'joint_angles': list(feedback_opt.joint_angles),
                    'motor_positions': list(feedback_opt.motor_positions),
                }
                
                now = self.get_clock().now().to_msg()
                
                # Publish complete hand feedback
                self._publish_hand_feedback(feedback, now)
                
                # Publish force feedback
                self._publish_force_feedback(feedback, now)
                
                # Publish motor feedback
                self._publish_motor_feedback(feedback, now)
                
                # Publish standard joint states
                self._publish_joint_states(feedback, now)
                
            except ModbusError as e:
                self.get_logger().error(f'Modbus error reading feedback: {e}')
            except Exception as e:
                self.get_logger().error(f'Error in feedback callback: {e}')
    
    def _publish_hand_feedback(self, feedback: dict, stamp):
        """Publish complete hand feedback"""
        msg = HandFeedback()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.hand_id = self._hand_id
        
        # Motor positions
        motor_pos = feedback['motor_positions']
        msg.thumb_rotation_pos = motor_pos[0]
        msg.thumb_bend_pos = motor_pos[1]
        msg.index_bend_pos = motor_pos[2]
        msg.middle_bend_pos = motor_pos[3]
        msg.ring_bend_pos = motor_pos[4]
        msg.pinky_bend_pos = motor_pos[5]
        
        # Joint angles (in 0.1 degree units, convert back to raw)
        angles = feedback['joint_angles']
        msg.thumb_mid_angle = int(angles[0] * 10)
        msg.thumb_root_angle = int(angles[1] * 10)
        msg.index_mid_angle = int(angles[2] * 10)
        msg.index_root_angle = int(angles[3] * 10)
        msg.middle_mid_angle = int(angles[4] * 10)
        msg.middle_root_angle = int(angles[5] * 10)
        msg.ring_mid_angle = int(angles[6] * 10)
        msg.ring_root_angle = int(angles[7] * 10)
        msg.pinky_mid_angle = int(angles[8] * 10)
        msg.pinky_root_angle = int(angles[9] * 10)
        
        # Force feedback
        forces = feedback['force_feedback']
        valid = feedback['force_valid']
        msg.thumb_tip_force = forces[0]
        msg.thumb_mid_force = forces[1]
        msg.index_tip_force = forces[2]
        msg.index_mid_force = forces[3]
        msg.middle_tip_force = forces[4]
        msg.middle_mid_force = forces[5]
        msg.ring_tip_force = forces[6]
        msg.ring_mid_force = forces[7]
        msg.pinky_tip_force = forces[8]
        msg.pinky_mid_force = forces[9]
        
        msg.thumb_tip_force_valid = valid[0]
        msg.thumb_mid_force_valid = valid[1]
        msg.index_tip_force_valid = valid[2]
        msg.index_mid_force_valid = valid[3]
        msg.middle_tip_force_valid = valid[4]
        msg.middle_mid_force_valid = valid[5]
        msg.ring_tip_force_valid = valid[6]
        msg.ring_mid_force_valid = valid[7]
        msg.pinky_tip_force_valid = valid[8]
        msg.pinky_mid_force_valid = valid[9]
        
        # Palm forces
        palm = feedback['palm_forces']
        palm_valid = feedback['palm_valid']
        msg.palm_force_1 = palm[0]
        msg.palm_force_2 = palm[1]
        msg.palm_force_3 = palm[2]
        msg.palm_force_1_valid = palm_valid[0]
        msg.palm_force_2_valid = palm_valid[1]
        msg.palm_force_3_valid = palm_valid[2]
        
        self._hand_feedback_pub.publish(msg)
    
    def _publish_force_feedback(self, feedback: dict, stamp):
        """Publish force feedback"""
        msg = ForceFeedback()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.hand_id = self._hand_id
        
        forces = feedback['force_feedback']
        palm = feedback['palm_forces']
        valid = feedback['force_valid'] + feedback['palm_valid']
        
        msg.thumb_tip = forces[0]
        msg.thumb_mid = forces[1]
        msg.index_tip = forces[2]
        msg.index_mid = forces[3]
        msg.middle_tip = forces[4]
        msg.middle_mid = forces[5]
        msg.ring_tip = forces[6]
        msg.ring_mid = forces[7]
        msg.pinky_tip = forces[8]
        msg.pinky_mid = forces[9]
        msg.palm_1 = palm[0]
        msg.palm_2 = palm[1]
        msg.palm_3 = palm[2]
        msg.valid_flags = valid
        
        self._force_feedback_pub.publish(msg)
    
    def _publish_motor_feedback(self, feedback: dict, stamp):
        """Publish motor position feedback"""
        msg = MotorFeedback()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.hand_id = self._hand_id
        
        motor_pos = feedback['motor_positions']
        msg.thumb_rotation = motor_pos[0]
        msg.thumb_bend = motor_pos[1]
        msg.index_bend = motor_pos[2]
        msg.middle_bend = motor_pos[3]
        msg.ring_bend = motor_pos[4]
        msg.pinky_bend = motor_pos[5]
        
        self._motor_feedback_pub.publish(msg)
    
    def _publish_joint_states(self, feedback: dict, stamp):
        """Publish standard ROS joint states"""
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        
        # Joint names (motor joints + angle joints)
        hand_prefix = 'right_' if self._hand_id == 1 else 'left_'
        
        # Motor joint names
        motor_names = [
            f'{hand_prefix}thumb_rotation_joint',
            f'{hand_prefix}thumb_bend_joint',
            f'{hand_prefix}index_bend_joint',
            f'{hand_prefix}middle_bend_joint',
            f'{hand_prefix}ring_bend_joint',
            f'{hand_prefix}pinky_bend_joint',
        ]
        
        # Angle joint names
        angle_names = [
            f'{hand_prefix}thumb_mid_joint',
            f'{hand_prefix}thumb_root_joint',
            f'{hand_prefix}index_mid_joint',
            f'{hand_prefix}index_root_joint',
            f'{hand_prefix}middle_mid_joint',
            f'{hand_prefix}middle_root_joint',
            f'{hand_prefix}ring_mid_joint',
            f'{hand_prefix}ring_root_joint',
            f'{hand_prefix}pinky_mid_joint',
            f'{hand_prefix}pinky_root_joint',
        ]
        
        msg.name = motor_names + angle_names
        
        # Motor positions (normalized to 0-1 range)
        motor_pos = feedback['motor_positions']
        motor_normalized = [float(p) / 1000.0 for p in motor_pos]
        
        # Joint angles in radians
        angles = feedback['joint_angles']
        angles_rad = [math.radians(a) for a in angles]
        
        msg.position = motor_normalized + angles_rad
        
        # Velocity and effort are not available from hardware
        msg.velocity = []
        msg.effort = []
        
        self._joint_states_pub.publish(msg)
    
    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.get_logger().info('Shutting down LZ Hand Driver Node')
        
        if self._driver:
            self._driver.disconnect()
        
        super().destroy_node()


def main(args=None):
    """Main entry point"""
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
