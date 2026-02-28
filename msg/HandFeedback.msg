# HandFeedback.msg
# 机械手完整反馈消息（Complete hand feedback message）

# 消息头（Header）
std_msgs/Header header

# 手ID：1=右手，2=左手（Hand ID: 1=right, 2=left）
uint8 hand_id

# 电机位置反馈（Motor position feedback）：0-1000
uint16 thumb_rotation_pos
uint16 thumb_bend_pos
uint16 index_bend_pos
uint16 middle_bend_pos
uint16 ring_bend_pos
uint16 pinky_bend_pos

# 关节角度反馈（Joint angle feedback）：单位0.1度（unit: 0.1 degree）
uint16 thumb_mid_angle
uint16 thumb_root_angle
uint16 index_mid_angle
uint16 index_root_angle
uint16 middle_mid_angle
uint16 middle_root_angle
uint16 ring_mid_angle
uint16 ring_root_angle
uint16 pinky_mid_angle
uint16 pinky_root_angle

# 力反馈（Force feedback）：单位克（unit: gram），有效范围750-3000
uint16 thumb_tip_force
uint16 thumb_mid_force
uint16 index_tip_force
uint16 index_mid_force
uint16 middle_tip_force
uint16 middle_mid_force
uint16 ring_tip_force
uint16 ring_mid_force
uint16 pinky_tip_force
uint16 pinky_mid_force
uint16 palm_force_1
uint16 palm_force_2
uint16 palm_force_3

# 力反馈有效标志（Force validity flags）
bool thumb_tip_force_valid
bool thumb_mid_force_valid
bool index_tip_force_valid
bool index_mid_force_valid
bool middle_tip_force_valid
bool middle_mid_force_valid
bool ring_tip_force_valid
bool ring_mid_force_valid
bool pinky_tip_force_valid
bool pinky_mid_force_valid
bool palm_force_1_valid
bool palm_force_2_valid
bool palm_force_3_valid
