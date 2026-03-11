# LZ Hand RS485 Driver

灵掌灵巧手RS485 Modbus-RTU ROS2驱动包

---

### [中文文档](#中文文档) | [English Documentation](#english-documentation)

---

<a name="中文文档"></a>
## 中文文档

### 概述

通过RS485 Modbus-RTU协议控制灵巧手的ROS2驱动包。

**功能：**
- 6自由度手指控制（大拇指翻转 + 5手指弯曲）
- 位置、速度、力三维控制
- 力反馈（13个传感器）、角度反馈（10个角度）
- 断线自动重连

### 硬件要求

| 项目 | 规格 |
|------|------|
| 供电 | 48V DC，最大2.5A |
| 通信 | RS485，115200bps，Modbus-RTU |
| 地址 | 右手=1，左手=2 |

### 连接方式

| 连接方式 | 设备路径示例 | 适用场景 |
|---------|-------------|---------|
| **USB转串口** | `/dev/ttyUSB0`、`/dev/ttyUSB1` | 独立使用，通过USB转485模块连接 |
| **直接485接口** | `/dev/ttyS0`、`/dev/ttyAMA0` | 集成到机械臂/机器人，通过板载485接口连接 |

> **注意**：两种方式使用相同的驱动代码，只需在配置中指定正确的设备路径即可。

### 安装

```bash
# 安装依赖
sudo apt-get update
sudo apt-get install -y libmodbus-dev libmodbus5 pybind11-dev python3-dev

# 编译
cd <your_ros2_workspace>
colcon build --packages-select lz_hand_rs485_driver

# 使用前必须 source
source install/setup.bash
```

### 快速开始

#### 方式一：USB转串口连接（需要检测端口）

```bash
# 1. 检测串口对应的hand_id（首次使用或设备重新插拔后）
cd <your_ros2_workspace>/src/lz_hand_rs485_driver
chmod +x tool/detect_hand_ports.sh && ./tool/detect_hand_ports.sh

# 2. 根据输出结果更新 config/hand_config.yaml 中的 serial 部分
#    例如：serial:
#           right_hand_port: "/dev/ttyUSB1"
#           left_hand_port: "/dev/ttyUSB0"

# 3. 启动驱动节点（指定hand_id，自动从配置文件读取串口）
ros2 launch lz_hand_rs485_driver hand_driver.launch.py hand_id:=2
```

#### 方式二：直接485接口连接（端口固定，无需检测）

```bash
# 直接指定设备路径启动（适用于集成到机械臂/机器人的场景）
# 单手
ros2 launch lz_hand_rs485_driver hand_driver.launch.py port:=/dev/ttyS0 hand_id:=1

# 双手
ros2 launch lz_hand_rs485_driver dual_hand_driver.launch.py \
    right_port:=/dev/ttyS0 left_port:=/dev/ttyS1

# 或者直接修改 config/hand_config.yaml 中的端口配置为实际的485设备路径
```

#### 测试与控制

```bash
# 单手测试（默认右手，单手驱动模式下使用）
ros2 run lz_hand_rs485_driver hand_test_node.py

# 单手测试（左手）
ros2 run lz_hand_rs485_driver hand_test_node.py --ros-args -p hand_id:=2

# 双手同时测试（命令同时发给两只手，双手驱动模式下使用）
ros2 run lz_hand_rs485_driver hand_test_node.py --ros-args -p dual:=true

# 双手模式下只控制右手
ros2 run lz_hand_rs485_driver hand_test_node.py --ros-args -p dual:=true -p hand_id:=1

# 双手模式下只控制左手
ros2 run lz_hand_rs485_driver hand_test_node.py --ros-args -p dual:=true -p hand_id:=2
```

**测试节点交互命令：**

| 命令 | 说明 |
|------|------|
| `1` | 张开（Open） |
| `2` | 竖拇指（Thumbs up） |
| `3` | 和平（Peace） |
| `4` | 捏取（Pinch） |
| `5` | 三指（Three） |
| `6` | 四指（Four） |
| `d` | 运行演示序列 |
| `f` | 查看反馈数据 |
| `j` | 查看关节状态 |
| `p <关节0-5> <位置0-1000>` | 控制单个关节 |
| `a <位置0-1000>` | 所有关节移到同一位置 |
| `m p0 p1 p2 p3 p4 p5` | 分别设置6个关节位置 |
| `q` | 退出 |

#### 命令行控制

```bash
ros2 topic pub --once /lz/hand_control lz_hand_rs485_driver/msg/HandControl \
  "{hand_id: 1, thumb_rotation: 0, thumb_bend: 0, index_bend: 0, middle_bend: 0, ring_bend: 0, pinky_bend: 0}"
```

### ROS2 话题

所有话题带 `/lz/` 前缀，避免与机械臂等其他设备冲突。

**单手模式**：

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/lz/hand_control` | HandControl | 订阅 | 6关节位置/速度/力控制 |
| `/lz/joint_control` | JointControl | 订阅 | 单关节控制 |
| `/lz/hand_feedback` | HandFeedback | 发布 | 完整反馈数据 |
| `/lz/motor_feedback` | MotorFeedback | 发布 | 电机位置反馈 |
| `/lz/force_feedback` | ForceFeedback | 发布 | 力反馈数据 |
| `/lz/joint_states` | JointState | 发布 | ROS标准关节状态 |

**双手模式**：话题在 `/lz/` 下按 `right_hand`/`left_hand` 区分，如 `/lz/right_hand/hand_control`、`/lz/left_hand/hand_control`。

### Launch文件

| 文件 | 说明 |
|------|------|
| `hand_driver.launch.py` | 单手驱动节点 |
| `dual_hand_driver.launch.py` | 双手驱动节点 |

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port` | `/dev/ttyUSB0` | 串口路径 |
| `hand_id` | `1` | 手ID：1=右手，2=左手 |
| `feedback_rate` | `20.0` | 反馈频率（Hz） |
| `auto_reconnect` | `true` | 自动重连 |
| `namespace` | `lz` | 话题命名空间前缀 |

### 配置文件

`config/hand_config.yaml` 包含串口映射、控制参数等。

**USB转串口用户：**
1. 运行 `./tool/detect_hand_ports.sh`
2. 更新 `config/hand_config.yaml` 中的端口
3. 启动时只需指定 `hand_id`

**直接485接口用户：**
1. 在 `config/hand_config.yaml` 中配置设备路径，或启动时 `port:=/dev/ttyS0`
2. 无需检测工具

### 自己写节点控制

驱动启动后，往 `/lz/hand_control` 或 `/lz/joint_control` 发消息即可控制。

- **HandControl**：6个关节一次给齐。位置 0=张开，1000=弯曲；速度/力 0-1000
- **JointControl**：单关节控制。`joint_index` 0-5 = 大拇指翻转/大拇指弯曲/食指/中指/无名指/小拇指

```python
import rclpy
from rclpy.node import Node
from lz_hand_rs485_driver.msg import HandControl

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.pub = self.create_publisher(HandControl, '/lz/hand_control', 10)
        msg = HandControl()
        msg.hand_id = 1
        msg.thumb_rotation = msg.thumb_bend = 1000
        msg.index_bend = msg.middle_bend = msg.ring_bend = msg.pinky_bend = 1000
        msg.thumb_rotation_speed = msg.thumb_bend_speed = 500
        msg.index_speed = msg.middle_speed = msg.ring_speed = msg.pinky_speed = 500
        msg.thumb_rotation_force = msg.thumb_bend_force = 500
        msg.index_force = msg.middle_force = msg.ring_force = msg.pinky_force = 500
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MyController()
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 注意事项

- 位置范围：0-1000（0=张开，1000=弯曲）
- 速度/力范围：0-1000（默认500）
- 力反馈有效范围：750-3000g

### 故障排除

1. **串口权限不足**：
   - USB转串口：执行 `sudo chmod 666 /dev/ttyUSB0`（每次插拔后需重新执行）
   - 直接485：检查用户是否在 `dialout` 组，或配置 udev 规则

2. **USB转串口设备不存在（/dev/ttyUSB*）**：
   - 检查USB连接和电源
   - 检查驱动：`lsmod | grep ch341`
   - 如设备被 `brltty` 抢占，执行 `sudo apt remove brltty`

3. **直接485设备路径**：
   - 板载串口通常为 `/dev/ttyS0`、`/dev/ttyS1`
   - 树莓派/ARM设备通常为 `/dev/ttyAMA0`
   - 具体路径请参考机械臂/机器人厂商文档

---

<a name="english-documentation"></a>
## English Documentation

### Overview

ROS2 driver for LZ dexterous hand via RS485 Modbus-RTU protocol.

**Features:**
- 6-DOF finger control (thumb rotation + 5 finger bending)
- Position, speed, and force control
- Force feedback (13 sensors), angle feedback (10 angles)
- Auto-reconnect on disconnect

### Hardware Requirements

| Item | Specification |
|------|---------------|
| Power | 48V DC, max 2.5A |
| Communication | RS485, 115200bps, Modbus-RTU |
| Address | Right=1, Left=2 |

### Connection Methods

| Connection | Device Path Example | Use Case |
|------------|--------------------|-----------| 
| **USB-to-Serial** | `/dev/ttyUSB0`, `/dev/ttyUSB1` | Standalone use via USB-to-RS485 adapter |
| **Direct RS485** | `/dev/ttyS0`, `/dev/ttyAMA0` | Integrated into robot arm via onboard RS485 port |

> **Note**: Both methods use the same driver code. Just specify the correct device path in configuration.

### Installation

```bash
# Install dependencies
sudo apt-get update
sudo apt-get install -y libmodbus-dev libmodbus5 pybind11-dev python3-dev

# Build
cd <your_ros2_workspace>
colcon build --packages-select lz_hand_rs485_driver

# Must source before use
source install/setup.bash
```

### Quick Start

#### Option 1: USB-to-Serial (requires port detection)

```bash
# 1. Detect hand_id for each serial port (first time or after replugging)
cd <your_ros2_workspace>/src/lz_hand_rs485_driver
chmod +x tool/detect_hand_ports.sh && ./tool/detect_hand_ports.sh

# 2. Update serial section in config/hand_config.yaml based on output
#    Example: serial:
#              right_hand_port: "/dev/ttyUSB1"
#              left_hand_port: "/dev/ttyUSB0"

# 3. Launch driver node (specify hand_id, port auto-read from config)
ros2 launch lz_hand_rs485_driver hand_driver.launch.py hand_id:=2
```

#### Option 2: Direct RS485 (fixed port, no detection needed)

```bash
# Specify device path directly (for robot arm integration)
# Single hand
ros2 launch lz_hand_rs485_driver hand_driver.launch.py port:=/dev/ttyS0 hand_id:=1

# Dual hands
ros2 launch lz_hand_rs485_driver dual_hand_driver.launch.py \
    right_port:=/dev/ttyS0 left_port:=/dev/ttyS1

# Or directly modify config/hand_config.yaml with actual RS485 device paths
```

#### Testing and Control

```bash
# Single hand test (default right hand, use with single hand driver)
ros2 run lz_hand_rs485_driver hand_test_node.py

# Single hand test (left hand)
ros2 run lz_hand_rs485_driver hand_test_node.py --ros-args -p hand_id:=2

# Dual hand test (commands sent to both hands, use with dual hand driver)
ros2 run lz_hand_rs485_driver hand_test_node.py --ros-args -p dual:=true

# Dual mode - control right hand only
ros2 run lz_hand_rs485_driver hand_test_node.py --ros-args -p dual:=true -p hand_id:=1

# Dual mode - control left hand only
ros2 run lz_hand_rs485_driver hand_test_node.py --ros-args -p dual:=true -p hand_id:=2
```

**Test node interactive commands:**

| Command | Description |
|---------|-------------|
| `1` | Open hand |
| `2` | Thumbs up |
| `3` | Peace sign |
| `4` | Pinch |
| `5` | Three fingers |
| `6` | Four fingers |
| `d` | Run demo sequence |
| `f` | View feedback data |
| `j` | View joint states |
| `p <joint0-5> <pos0-1000>` | Control single joint |
| `a <pos0-1000>` | Move all joints to same position |
| `m p0 p1 p2 p3 p4 p5` | Set 6 joint positions individually |
| `q` | Quit |

#### Command Line Control

```bash
ros2 topic pub --once /lz/hand_control lz_hand_rs485_driver/msg/HandControl \
  "{hand_id: 1, thumb_rotation: 0, thumb_bend: 0, index_bend: 0, middle_bend: 0, ring_bend: 0, pinky_bend: 0}"
```

### ROS2 Topics

All topics are under `/lz/` namespace to avoid conflicts with other devices.

**Single hand**:

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/lz/hand_control` | HandControl | Subscribe | 6-joint position/speed/force |
| `/lz/joint_control` | JointControl | Subscribe | Single joint control |
| `/lz/hand_feedback` | HandFeedback | Publish | Complete feedback |
| `/lz/motor_feedback` | MotorFeedback | Publish | Motor position feedback |
| `/lz/force_feedback` | ForceFeedback | Publish | Force feedback |
| `/lz/joint_states` | JointState | Publish | Standard ROS joint states |

**Dual hand**: Topics split by `right_hand`/`left_hand` under `/lz/`, e.g. `/lz/right_hand/hand_control`, `/lz/left_hand/hand_control`.

### Launch Files

| File | Description |
|------|-------------|
| `hand_driver.launch.py` | Single hand driver node |
| `dual_hand_driver.launch.py` | Dual hand driver node |

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `/dev/ttyUSB0` | Serial port path |
| `hand_id` | `1` | Hand ID: 1=right, 2=left |
| `feedback_rate` | `20.0` | Feedback rate (Hz) |
| `auto_reconnect` | `true` | Auto-reconnect |
| `namespace` | `lz` | Topic namespace prefix |

### Configuration

`config/hand_config.yaml` contains port mapping, control parameters, etc.

**For USB-to-Serial users:**
1. Run `./tool/detect_hand_ports.sh`
2. Update ports in `config/hand_config.yaml`
3. Only specify `hand_id` when launching

**For Direct RS485 users:**
1. Configure device path in `config/hand_config.yaml`, or use `port:=/dev/ttyS0` at launch
2. No detection tool needed

### Writing Your Own Control Node

After launching the driver, publish to `/lz/hand_control` or `/lz/joint_control`.

- **HandControl**: All 6 joints at once. Position 0=open, 1000=close; speed/force 0-1000
- **JointControl**: Single joint. `joint_index` 0-5 = thumb rotation/thumb bend/index/middle/ring/pinky

```python
import rclpy
from rclpy.node import Node
from lz_hand_rs485_driver.msg import HandControl

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.pub = self.create_publisher(HandControl, '/lz/hand_control', 10)
        msg = HandControl()
        msg.hand_id = 1
        msg.thumb_rotation = msg.thumb_bend = 1000
        msg.index_bend = msg.middle_bend = msg.ring_bend = msg.pinky_bend = 1000
        msg.thumb_rotation_speed = msg.thumb_bend_speed = 500
        msg.index_speed = msg.middle_speed = msg.ring_speed = msg.pinky_speed = 500
        msg.thumb_rotation_force = msg.thumb_bend_force = 500
        msg.index_force = msg.middle_force = msg.ring_force = msg.pinky_force = 500
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MyController()
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Notes

- Position range: 0-1000 (0=open, 1000=close)
- Speed/Force range: 0-1000 (default 500)
- Force feedback valid range: 750-3000g

### Troubleshooting

1. **Serial port permission denied**:
   - USB-to-Serial: `sudo chmod 666 /dev/ttyUSB0` (need to re-execute after each plug/unplug)
   - Direct RS485: Check if user is in `dialout` group, or configure udev rules

2. **USB-to-Serial device not found (/dev/ttyUSB*)**:
   - Check USB connection and power
   - Check driver: `lsmod | grep ch341`
   - If device is claimed by `brltty`: `sudo apt remove brltty`

3. **Direct RS485 device paths**:
   - Onboard serial ports are usually `/dev/ttyS0`, `/dev/ttyS1`
   - Raspberry Pi/ARM devices are usually `/dev/ttyAMA0`
   - Refer to robot arm manufacturer documentation for specific paths

---

**License:** Copyright © 灵掌机器人 | **Author:** Garry | **Version:** 1.2.0
