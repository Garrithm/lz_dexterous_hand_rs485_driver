# HandControl.msg
# 机械手整体控制消息
# Control message for the dexterous hand

# Header for timestamp and frame_id
std_msgs/Header header

# Hand selection: 1=right hand, 2=left hand
uint8 hand_id

# Position control (0-1000, unit: 1/1000 of max stroke)
uint16 thumb_rotation      # 大拇指翻转 (0=不翻转, 1000=完全翻转)
uint16 thumb_bend          # 大拇指弯曲 (0=伸直, 1000=完全弯曲)
uint16 index_bend          # 食指弯曲
uint16 middle_bend         # 中指弯曲
uint16 ring_bend           # 无名指弯曲
uint16 pinky_bend          # 小拇指弯曲

# Speed control (0-1000, unit: 1/1000 of rated speed)
uint16 thumb_rotation_speed
uint16 thumb_bend_speed
uint16 index_speed
uint16 middle_speed
uint16 ring_speed
uint16 pinky_speed

# Force control (0-1000, unit: 1/1000 of rated force)
uint16 thumb_rotation_force
uint16 thumb_bend_force
uint16 index_force
uint16 middle_force
uint16 ring_force
uint16 pinky_force
