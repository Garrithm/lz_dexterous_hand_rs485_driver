# HandFeedback.msg
# 机械手完整反馈消息
# Complete feedback message from the dexterous hand

# Header for timestamp and frame_id
std_msgs/Header header

# Hand selection: 1=right hand, 2=left hand
uint8 hand_id

# Motor position feedback (0-1000, unit: 1/1000 of max stroke)
uint16 thumb_rotation_pos      # 大拇指翻转电机位置
uint16 thumb_bend_pos          # 大拇指弯曲电机位置
uint16 index_bend_pos          # 食指弯曲电机位置
uint16 middle_bend_pos         # 中指弯曲电机位置
uint16 ring_bend_pos           # 无名指弯曲电机位置
uint16 pinky_bend_pos          # 小拇指弯曲电机位置

# Joint angle feedback (unit: 0.1 degree)
uint16 thumb_mid_angle         # 大拇指指中关节角度
uint16 thumb_root_angle        # 大拇指指根关节角度
uint16 index_mid_angle         # 食指指中关节角度
uint16 index_root_angle        # 食指指根关节角度
uint16 middle_mid_angle        # 中指指中关节角度
uint16 middle_root_angle       # 中指指根关节角度
uint16 ring_mid_angle          # 无名指指中关节角度
uint16 ring_root_angle         # 无名指指根关节角度
uint16 pinky_mid_angle         # 小拇指指中关节角度
uint16 pinky_root_angle        # 小拇指指根关节角度

# Force feedback (unit: gram, valid range: 750-3000)
uint16 thumb_tip_force         # 大拇指指尖力反馈
uint16 thumb_mid_force         # 大拇指指中力反馈
uint16 index_tip_force         # 食指指尖力反馈
uint16 index_mid_force         # 食指指中力反馈
uint16 middle_tip_force        # 中指指尖力反馈
uint16 middle_mid_force        # 中指指中力反馈
uint16 ring_tip_force          # 无名指指尖力反馈
uint16 ring_mid_force          # 无名指指中力反馈
uint16 pinky_tip_force         # 小拇指指尖力反馈
uint16 pinky_mid_force         # 小拇指指中力反馈
uint16 palm_force_1            # 手掌力反馈(1)
uint16 palm_force_2            # 手掌力反馈(2)
uint16 palm_force_3            # 手掌力反馈(3)

# Validity flags for force feedback (true if value is in valid range 750-3000)
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
