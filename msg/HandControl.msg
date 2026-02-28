# HandControl.msg
# 机械手整体控制消息（Complete hand control message）

# 消息头（Header）
std_msgs/Header header

# 手ID：1=右手，2=左手（Hand ID: 1=right, 2=left）
uint8 hand_id

# 位置控制（Position control）：0-1000
uint16 thumb_rotation
uint16 thumb_bend
uint16 index_bend
uint16 middle_bend
uint16 ring_bend
uint16 pinky_bend

# 速度控制（Speed control）：0-1000
uint16 thumb_rotation_speed
uint16 thumb_bend_speed
uint16 index_speed
uint16 middle_speed
uint16 ring_speed
uint16 pinky_speed

# 力控制（Force control）：0-1000
uint16 thumb_rotation_force
uint16 thumb_bend_force
uint16 index_force
uint16 middle_force
uint16 ring_force
uint16 pinky_force
