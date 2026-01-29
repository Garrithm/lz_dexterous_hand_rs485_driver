# JointControl.msg
# 单关节控制消息
# Control message for individual joint

# Header for timestamp and frame_id
std_msgs/Header header

# Hand selection: 1=right hand, 2=left hand
uint8 hand_id

# Joint index (0-5)
# 0: thumb_rotation (大拇指翻转)
# 1: thumb_bend (大拇指弯曲)
# 2: index_bend (食指弯曲)
# 3: middle_bend (中指弯曲)
# 4: ring_bend (无名指弯曲)
# 5: pinky_bend (小拇指弯曲)
uint8 joint_index

# Position (0-1000, unit: 1/1000 of max stroke)
uint16 position

# Speed (0-1000, unit: 1/1000 of rated speed, optional)
uint16 speed

# Force (0-1000, unit: 1/1000 of rated force, optional)
uint16 force
