# LZ Hand RS485 Driver (ROS2 Humble)

灵巧手RS485 Modbus-RTU ROS2驱动

[English](#english) | [中文](#chinese)

---

<a name="chinese"></a>
## 中文文档

### 概述

本ROS2包提供了通过RS485 Modbus-RTU协议控制灵巧手的完整驱动程序。支持：

- 6自由度手指控制（大拇指翻转 + 5手指弯曲）
- 位置、速度、力三维控制
- 实时力反馈读取（13个传感器）
- 关节角度反馈（10个角度）
- 电机位置反馈
- 预设手势动作

### 硬件要求

| 项目 | 规格 |
|------|------|
| 供电 | 48V DC，最大2.5A |
| 通信接口 | RS485 |
| 波特率 | 115200 |
| 协议 | Modbus-RTU |
| 从机地址 | 右手=1，左手=2 |

### 软件依赖

- ROS2 Humble
- Python 3.10+

```bash
# ROS2依赖（通常已随ROS2安装）
sudo apt-get install ros-humble-rclpy ros-humble-std-msgs ros-humble-sensor-msgs

# Python依赖
pip install pymodbus>=2.5.0 pyserial>=3.5
```

### 安装

```bash
# 进入ROS2工作空间
cd ~/ros2_ws/src

# 克隆或复制本包
# git clone <repository_url> lz_hand_rs485_driver
# 或者直接复制

# 编译
cd ~/ros2_ws
colcon build --packages-select lz_hand_rs485_driver

# 刷新环境
source install/setup.bash
```

### 测试

#### 1. 验证编译和安装

```bash
# 检查编译是否成功
cd ~/ros2_ws
colcon build --packages-select lz_hand_rs485_driver

# 检查安装文件
ls -la install/lz_hand_rs485_driver/lib/lz_hand_rs485_driver/
# 应该看到：
# - hand_driver_node.py
# - hand_test_node.py
# - lz_hand_driver_cpp.cpython-*.so (Python绑定模块)
```

#### 2. 测试 Python 绑定模块导入

```bash
# 刷新环境
source ~/ros2_ws/install/setup.bash

# 测试 Python 绑定模块是否能正常导入
python3 -c "
import sys
sys.path.insert(0, '/home/garry/ros2_ws/install/lz_hand_rs485_driver/lib/lz_hand_rs485_driver')
try:
    from lz_hand_driver_cpp import LZHandModbusDriver, HandConstants, HandID, JointIndex
    print('✓ Python 绑定模块导入成功！')
    print(f'  - HandID: {HandID.RIGHT_HAND}, {HandID.LEFT_HAND}')
    print(f'  - JointIndex: {JointIndex.THUMB_ROTATION}, {JointIndex.INDEX_BEND}')
except ImportError as e:
    print(f'✗ 导入失败: {e}')
    sys.exit(1)
"
```

#### 3. 测试驱动节点启动（无硬件）

```bash
# 测试节点是否能正常启动（即使没有硬件连接）
# 这可以验证 ROS2 节点和消息类型是否正确
ros2 launch lz_hand_rs485_driver hand_driver.launch.py port:=/dev/ttyUSB0 hand_id:=1

# 在另一个终端检查节点是否运行
ros2 node list
# 应该看到: /lz_hand_driver

# 检查话题
ros2 topic list
# 应该看到:
# /hand_control
# /hand_feedback
# /force_feedback
# /joint_control
# /joint_feedback
# /joint_states
# /motor_feedback
```

#### 4. 测试消息类型

```bash
# 检查消息类型定义
ros2 interface show lz_hand_rs485_driver/msg/HandControl
ros2 interface show lz_hand_rs485_driver/msg/HandFeedback
ros2 interface show lz_hand_rs485_driver/msg/JointControl
```

#### 5. 测试通信连接（需要硬件）

```bash
# 确保串口设备存在
ls -l /dev/ttyUSB*

# 检查串口权限
ls -l /dev/ttyUSB0
# 应该显示用户有读写权限

# 启动驱动节点
ros2 launch lz_hand_rs485_driver hand_driver.launch.py port:=/dev/ttyUSB0 hand_id:=1

# 查看节点日志，应该看到：
# - "Connected to hand" 或类似的连接成功消息
# - 如果没有硬件，会看到超时或连接失败的错误（这是正常的）
```

#### 6. 功能测试（需要硬件）

```bash
# 方式1: 使用测试节点（交互模式）
ros2 launch lz_hand_rs485_driver hand_test.launch.py port:=/dev/ttyUSB0 hand_id:=1 mode:=interactive

# 方式2: 使用测试节点（演示模式）
ros2 launch lz_hand_rs485_driver hand_test.launch.py port:=/dev/ttyUSB0 hand_id:=1 mode:=demo

# 方式3: 手动发送控制命令
# 终端1: 启动驱动
ros2 launch lz_hand_rs485_driver hand_driver.launch.py port:=/dev/ttyUSB0 hand_id:=1

# 终端2: 查看反馈
ros2 topic echo /hand_feedback

# 终端3: 发送控制命令
# 张开手
ros2 topic pub --once /hand_control lz_hand_rs485_driver/msg/HandControl \
  "{hand_id: 1, thumb_rotation: 0, thumb_bend: 0, index_bend: 0, middle_bend: 0, ring_bend: 0, pinky_bend: 0}"

# 握拳
ros2 topic pub --once /hand_control lz_hand_rs485_driver/msg/HandControl \
  "{hand_id: 1, thumb_rotation: 500, thumb_bend: 1000, index_bend: 1000, middle_bend: 1000, ring_bend: 1000, pinky_bend: 1000}"
```

#### 7. 测试双手驱动（需要两个硬件）

```bash
# 启动双手驱动
ros2 launch lz_hand_rs485_driver dual_hand_driver.launch.py \
  right_port:=/dev/ttyUSB0 left_port:=/dev/ttyUSB1

# 检查节点
ros2 node list
# 应该看到: /right_hand/lz_hand_driver 和 /left_hand/lz_hand_driver

# 检查话题（带命名空间）
ros2 topic list
# 应该看到: /right_hand/hand_control, /left_hand/hand_control 等
```

#### 8. 常见测试问题排查

```bash
# 问题1: ModuleNotFoundError: No module named 'lz_hand_driver_cpp'
# 解决: 重新编译并确保模块安装到正确位置
cd ~/ros2_ws
colcon build --packages-select lz_hand_rs485_driver
source install/setup.bash

# 问题2: 找不到串口设备
# 解决: 检查设备连接和权限
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0

# 问题3: 通信超时
# 解决: 检查硬件连接、波特率、从机地址
# 确认硬件已上电并正确连接

# 问题4: 话题不存在
# 解决: 确认节点已启动
ros2 node list
ros2 topic list
```

### 串口权限设置

```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 注销并重新登录使更改生效

# 或者临时设置权限
sudo chmod 666 /dev/ttyUSB0

# 创建udev规则（推荐）
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", MODE="0666", SYMLINK+="lz_hand_right"' | sudo tee /etc/udev/rules.d/99-lz-hand.rules
sudo udevadm control --reload-rules
```

### 使用方法

#### 1. 启动单手驱动

```bash
# 启动右手驱动（默认）
ros2 launch lz_hand_rs485_driver hand_driver.launch.py

# 指定串口和手ID
ros2 launch lz_hand_rs485_driver hand_driver.launch.py port:=/dev/ttyUSB0 hand_id:=1

# 启动左手驱动
ros2 launch lz_hand_rs485_driver hand_driver.launch.py port:=/dev/ttyUSB1 hand_id:=2
```

#### 2. 启动双手驱动

```bash
ros2 launch lz_hand_rs485_driver dual_hand_driver.launch.py right_port:=/dev/ttyUSB0 left_port:=/dev/ttyUSB1
```

#### 3. 运行测试节点

```bash
# 交互模式
ros2 launch lz_hand_rs485_driver hand_test.launch.py mode:=interactive

# 演示模式
ros2 launch lz_hand_rs485_driver hand_test.launch.py mode:=demo

# 单独运行测试节点
ros2 run lz_hand_rs485_driver hand_test_node.py interactive
```

### ROS2话题

#### 控制话题（订阅）

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/hand_control` | `lz_hand_rs485_driver/msg/HandControl` | 完整手部控制 |
| `/joint_control` | `lz_hand_rs485_driver/msg/JointControl` | 单关节控制 |

#### 反馈话题（发布）

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/hand_feedback` | `lz_hand_rs485_driver/msg/HandFeedback` | 完整手部反馈 |
| `/force_feedback` | `lz_hand_rs485_driver/msg/ForceFeedback` | 力反馈数据 |
| `/motor_feedback` | `lz_hand_rs485_driver/msg/MotorFeedback` | 电机位置反馈 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 标准关节状态 |

### 消息格式

#### HandControl（手部控制）

```yaml
std_msgs/Header header
uint8 hand_id           # 1=右手, 2=左手
uint16 thumb_rotation   # 大拇指翻转 (0-1000)
uint16 thumb_bend       # 大拇指弯曲 (0-1000)
uint16 index_bend       # 食指弯曲 (0-1000)
uint16 middle_bend      # 中指弯曲 (0-1000)
uint16 ring_bend        # 无名指弯曲 (0-1000)
uint16 pinky_bend       # 小拇指弯曲 (0-1000)
# 速度和力参数同理
```

#### 示例代码 (Python)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lz_hand_rs485_driver.msg import HandControl

class HandControlExample(Node):
    def __init__(self):
        super().__init__('hand_control_example')
        self.publisher = self.create_publisher(HandControl, 'hand_control', 10)
        
    def send_command(self):
        msg = HandControl()
        msg.hand_id = 1  # 右手
        
        # 设置握拳姿态
        msg.thumb_rotation = 500
        msg.thumb_bend = 1000
        msg.index_bend = 1000
        msg.middle_bend = 1000
        msg.ring_bend = 1000
        msg.pinky_bend = 1000
        
        # 设置速度
        msg.thumb_rotation_speed = 500
        msg.thumb_bend_speed = 500
        msg.index_speed = 500
        msg.middle_speed = 500
        msg.ring_speed = 500
        msg.pinky_speed = 500
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = HandControlExample()
    node.send_command()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 命令行发布示例

```bash
# 张开手
ros2 topic pub /hand_control lz_hand_rs485_driver/msg/HandControl "{hand_id: 1, thumb_rotation: 0, thumb_bend: 0, index_bend: 0, middle_bend: 0, ring_bend: 0, pinky_bend: 0}"

# 握拳
ros2 topic pub /hand_control lz_hand_rs485_driver/msg/HandControl "{hand_id: 1, thumb_rotation: 500, thumb_bend: 1000, index_bend: 1000, middle_bend: 1000, ring_bend: 1000, pinky_bend: 1000}"

# 查看反馈
ros2 topic echo /hand_feedback
ros2 topic echo /joint_states
```

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port` | `/dev/ttyUSB0` | 串口设备 |
| `baudrate` | `115200` | 波特率 |
| `hand_id` | `1` | 从机地址 |
| `feedback_rate` | `50.0` | 反馈频率(Hz) |
| `gradual_step_size` | `100` | 渐进调节步长 |

### 注意事项

1. **力反馈有效范围**：750-3000g，超出范围的数值无效
2. **角度单位**：0.1°，需乘以0.1转换为度数
3. **渐进调节**：速度和力参数会自动进行渐进调节，避免冲击
4. **厂家寄存器**：寄存器47-57为厂家专用，禁止操作

### 故障排除

1. **无法连接串口**
   - 检查串口设备是否存在：`ls -l /dev/ttyUSB*`
   - 检查权限：`sudo chmod 666 /dev/ttyUSB0`
   - 确认波特率设置正确

2. **通信超时**
   - 检查RS485接线是否正确
   - 确认从机地址设置正确
   - 增加timeout参数值

3. **力反馈数据无效**
   - 正常现象，当力值超出750-3000g范围时返回无效

4. **编译错误**
   - 确保已安装所有依赖：`rosdep install --from-paths src --ignore-src -r -y`

---

<a name="english"></a>
## English Documentation

### Overview

This ROS2 package provides a complete driver for controlling a dexterous hand via RS485 Modbus-RTU protocol. Features:

- 6-DOF finger control (thumb rotation + 5 finger bending)
- Position, speed, and force control
- Real-time force feedback (13 sensors)
- Joint angle feedback (10 angles)
- Motor position feedback
- Preset gestures

### Hardware Requirements

| Item | Specification |
|------|---------------|
| Power | 48V DC, max 2.5A |
| Interface | RS485 |
| Baud Rate | 115200 |
| Protocol | Modbus-RTU |
| Slave Address | Right=1, Left=2 |

### Software Dependencies

- ROS2 Humble
- Python 3.10+

```bash
# Python dependencies
pip install pymodbus>=2.5.0 pyserial>=3.5
```

### Installation

```bash
# Enter ROS2 workspace
cd ~/ros2_ws/src

# Clone or copy this package
cd ~/ros2_ws
colcon build --packages-select lz_hand_rs485_driver

# Source the workspace
source install/setup.bash
```

### Testing

#### 1. Verify Build and Installation

```bash
# Check if build succeeded
cd ~/ros2_ws
colcon build --packages-select lz_hand_rs485_driver

# Check installed files
ls -la install/lz_hand_rs485_driver/lib/lz_hand_rs485_driver/
# Should see:
# - hand_driver_node.py
# - hand_test_node.py
# - lz_hand_driver_cpp.cpython-*.so (Python bindings module)
```

#### 2. Test Python Bindings Import

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Test if Python bindings can be imported
python3 -c "
import sys
sys.path.insert(0, '/home/garry/ros2_ws/install/lz_hand_rs485_driver/lib/lz_hand_rs485_driver')
try:
    from lz_hand_driver_cpp import LZHandModbusDriver, HandConstants, HandID, JointIndex
    print('✓ Python bindings import successful!')
    print(f'  - HandID: {HandID.RIGHT_HAND}, {HandID.LEFT_HAND}')
    print(f'  - JointIndex: {JointIndex.THUMB_ROTATION}, {JointIndex.INDEX_BEND}')
except ImportError as e:
    print(f'✗ Import failed: {e}')
    sys.exit(1)
"
```

#### 3. Test Driver Node Launch (No Hardware)

```bash
# Test if node can start (even without hardware connection)
# This verifies ROS2 node and message types are correct
ros2 launch lz_hand_rs485_driver hand_driver.launch.py port:=/dev/ttyUSB0 hand_id:=1

# In another terminal, check if node is running
ros2 node list
# Should see: /lz_hand_driver

# Check topics
ros2 topic list
# Should see control and feedback topics
```

#### 4. Test Message Types

```bash
# Check message type definitions
ros2 interface show lz_hand_rs485_driver/msg/HandControl
ros2 interface show lz_hand_rs485_driver/msg/HandFeedback
```

#### 5. Test Communication (Hardware Required)

```bash
# Ensure serial device exists
ls -l /dev/ttyUSB*

# Check serial permissions
ls -l /dev/ttyUSB0

# Launch driver node
ros2 launch lz_hand_rs485_driver hand_driver.launch.py port:=/dev/ttyUSB0 hand_id:=1

# Check node logs for connection status
```

#### 6. Functional Testing (Hardware Required)

```bash
# Method 1: Use test node (interactive mode)
ros2 launch lz_hand_rs485_driver hand_test.launch.py port:=/dev/ttyUSB0 hand_id:=1 mode:=interactive

# Method 2: Use test node (demo mode)
ros2 launch lz_hand_rs485_driver hand_test.launch.py port:=/dev/ttyUSB0 hand_id:=1 mode:=demo

# Method 3: Manual command sending
# Terminal 1: Launch driver
ros2 launch lz_hand_rs485_driver hand_driver.launch.py port:=/dev/ttyUSB0 hand_id:=1

# Terminal 2: View feedback
ros2 topic echo /hand_feedback

# Terminal 3: Send control commands
ros2 topic pub --once /hand_control lz_hand_rs485_driver/msg/HandControl \
  "{hand_id: 1, thumb_rotation: 0, thumb_bend: 0, index_bend: 0, middle_bend: 0, ring_bend: 0, pinky_bend: 0}"
```

### Usage

#### 1. Launch Single Hand Driver

```bash
# Launch right hand (default)
ros2 launch lz_hand_rs485_driver hand_driver.launch.py

# Specify port and hand ID
ros2 launch lz_hand_rs485_driver hand_driver.launch.py port:=/dev/ttyUSB0 hand_id:=1
```

#### 2. Launch Dual Hand Driver

```bash
ros2 launch lz_hand_rs485_driver dual_hand_driver.launch.py
```

#### 3. Run Test Node

```bash
# Interactive mode
ros2 launch lz_hand_rs485_driver hand_test.launch.py mode:=interactive

# Demo mode
ros2 launch lz_hand_rs485_driver hand_test.launch.py mode:=demo
```

### ROS2 Topics

#### Control Topics (Subscribe)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/hand_control` | `HandControl` | Full hand control |
| `/joint_control` | `JointControl` | Single joint control |

#### Feedback Topics (Publish)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/hand_feedback` | `HandFeedback` | Full hand feedback |
| `/force_feedback` | `ForceFeedback` | Force feedback data |
| `/motor_feedback` | `MotorFeedback` | Motor position feedback |
| `/joint_states` | `sensor_msgs/msg/JointState` | Standard joint states |

### Notes

1. **Force feedback valid range**: 750-3000g
2. **Angle unit**: 0.1°, multiply by 0.1 to get degrees
3. **Gradual adjustment**: Speed and force changes are automatically smoothed
4. **Factory registers**: Registers 47-57 are factory-reserved, DO NOT modify

---

## License

BSD-3-Clause

## Author

Garry

## Version

1.0.0
