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

### 安装

```bash
# 安装依赖
sudo apt-get install libmodbus-dev pybind11-dev

# 编译
cd <your_ros2_workspace>
colcon build --packages-select lz_hand_rs485_driver
source install/setup.bash
```

### 快速开始

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

# 4. 交互测试（另开终端）
ros2 run lz_hand_rs485_driver hand_test_node.py --ros-args -p hand_id:=2

# 5. 命令行控制
ros2 topic pub --once /hand_control lz_hand_rs485_driver/msg/HandControl \
  "{hand_id: 2, thumb_rotation: 0, thumb_bend: 0, index_bend: 0, middle_bend: 0, ring_bend: 0, pinky_bend: 0}"
```

### Launch文件

| 文件 | 说明 |
|------|------|
| `hand_driver.launch.py` | 单手驱动节点（支持从配置文件读取串口） |
| `dual_hand_driver.launch.py` | 双手驱动节点（支持从配置文件读取串口） |

### 配置文件

| 文件 | 说明 | 修改后的影响 |
|------|------|------------|
| `config/hand_config.yaml` | 完整配置文件（包含串口映射、控制参数等） | Launch文件会自动读取串口配置，无需手动指定port参数 |

**使用流程：**
1. 运行检测工具：`./tool/detect_hand_ports.sh`
2. 根据输出结果更新 `config/hand_config.yaml` 中的 `serial.right_hand_port` 和 `serial.left_hand_port`
3. 启动节点时只需指定 `hand_id`，串口会自动从配置文件读取

### ROS2话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `hand_control` | HandControl | 订阅 | 6关节位置/速度/力控制 |
| `joint_control` | JointControl | 订阅 | 单关节控制 |
| `hand_feedback` | HandFeedback | 发布 | 完整反馈数据 |
| `motor_feedback` | MotorFeedback | 发布 | 电机位置反馈 |
| `force_feedback` | ForceFeedback | 发布 | 力反馈数据 |
| `joint_states` | JointState | 发布 | ROS标准关节状态 |

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port` | `/dev/ttyUSB0` | 串口路径 |
| `hand_id` | `1` | 手ID：1=右手，2=左手 |
| `feedback_rate` | `20.0` | 反馈频率（Hz） |
| `auto_reconnect` | `true` | 自动重连 |

### 注意事项

- 位置范围：0-1000（0=张开，1000=弯曲）
- 速度/力范围：0-1000（默认500）
- 力反馈有效范围：750-3000g

### 故障排除

1. **串口权限不足**：执行 `sudo chmod 666 /dev/ttyUSB0`。注意：每次插拔设备后需要重新执行。

2. **串口设备不存在**：检查设备连接和电源。

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

### Installation

```bash
# Install dependencies
sudo apt-get install libmodbus-dev pybind11-dev

# Build
cd <your_ros2_workspace>
colcon build --packages-select lz_hand_rs485_driver
source install/setup.bash
```

### Quick Start

```bash
# 1. Detect hand_id for each serial port (first time or after replugging devices)
cd <your_ros2_workspace>/src/lz_hand_rs485_driver
chmod +x tool/detect_hand_ports.sh && ./tool/detect_hand_ports.sh

# 2. Update serial section in config/hand_config.yaml based on output
#    Example: serial:
#              right_hand_port: "/dev/ttyUSB1"
#              left_hand_port: "/dev/ttyUSB0"

# 3. Launch driver node (specify hand_id, port will be automatically read from config)
ros2 launch lz_hand_rs485_driver hand_driver.launch.py hand_id:=2

# 4. Interactive test (in another terminal)
ros2 run lz_hand_rs485_driver hand_test_node.py --ros-args -p hand_id:=2

# 5. Command line control
ros2 topic pub --once /hand_control lz_hand_rs485_driver/msg/HandControl \
  "{hand_id: 2, thumb_rotation: 0, thumb_bend: 0, index_bend: 0, middle_bend: 0, ring_bend: 0, pinky_bend: 0}"
```

### Launch Files

| File | Description |
|------|-------------|
| `hand_driver.launch.py` | Single hand driver (supports reading port from config) |
| `dual_hand_driver.launch.py` | Dual hand driver (supports reading ports from config) |

### Configuration Files

| File | Description | Effect After Modification |
|------|-------------|---------------------------|
| `config/hand_config.yaml` | Complete configuration file (includes port mapping, control parameters, etc.) | Launch files will automatically read port config and default parameters, no need to manually specify port |

**Usage Flow:**
1. Run detection tool: `./tool/detect_hand_ports.sh`
2. Update `serial.right_hand_port` and `serial.left_hand_port` in `config/hand_config.yaml` based on output
3. When launching nodes, only specify `hand_id`, port will be automatically read from config

### ROS2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `hand_control` | HandControl | Subscribe | 6-joint position/speed/force |
| `joint_control` | JointControl | Subscribe | Single joint control |
| `hand_feedback` | HandFeedback | Publish | Complete feedback |
| `motor_feedback` | MotorFeedback | Publish | Motor position feedback |
| `force_feedback` | ForceFeedback | Publish | Force feedback |
| `joint_states` | JointState | Publish | Standard ROS joint states |

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `/dev/ttyUSB0` | Serial port path |
| `hand_id` | `1` | Hand ID: 1=right, 2=left |
| `feedback_rate` | `20.0` | Feedback rate (Hz) |
| `auto_reconnect` | `true` | Auto-reconnect |

### Notes

- Position range: 0-1000 (0=open, 1000=close)
- Speed/Force range: 0-1000 (default 500)
- Force feedback valid range: 750-3000g

### Troubleshooting

1. **Serial Port Permission Denied**: Execute `sudo chmod 666 /dev/ttyUSB0`. Note: Need to re-execute after each device plug/unplug.

2. **Serial Port Not Found**: Check device connection and power.

---

**License:** Copyright © 灵掌机器人 | **Author:** Garry | **Version:** 1.1.0
