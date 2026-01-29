# HandFeedback.msg
# 机械手完整反馈消息（Complete hand feedback message）

# 消息头（Header）
std_msgs/Header header

# 手ID：1=右手，2=左手（Hand ID: 1=right, 2=left）
uint8 hand_id

# 电机位置反馈（Motor position feedback）：0-1000
uint16 thumb_rotation_pos      # 大拇指翻转（Thumb rotation）
uint16 thumb_bend_pos          # 大拇指弯曲（Thumb bend）
uint16 index_bend_pos          # 食指弯曲（Index bend）
uint16 middle_bend_pos         # 中指弯曲（Middle bend）
uint16 ring_bend_pos           # 无名指弯曲（Ring bend）
uint16 pinky_bend_pos          # 小拇指弯曲（Pinky bend）

# 关节角度反馈（Joint angle feedback）：单位0.1度（unit: 0.1 degree）
uint16 thumb_mid_angle         # 大拇指指中角度（Thumb mid joint）
uint16 thumb_root_angle        # 大拇指指根角度（Thumb root joint）
uint16 index_mid_angle         # 食指指中角度（Index mid joint）
uint16 index_root_angle        # 食指指根角度（Index root joint）
uint16 middle_mid_angle        # 中指指中角度（Middle mid joint）
uint16 middle_root_angle       # 中指指根角度（Middle root joint）
uint16 ring_mid_angle          # 无名指指中角度（Ring mid joint）
uint16 ring_root_angle         # 无名指指根角度（Ring root joint）
uint16 pinky_mid_angle         # 小拇指指中角度（Pinky mid joint）
uint16 pinky_root_angle        # 小拇指指根角度（Pinky root joint）

# 力反馈（Force feedback）：单位克（unit: gram），有效范围750-3000
uint16 thumb_tip_force         # 大拇指指尖力（Thumb tip）
uint16 thumb_mid_force         # 大拇指指中力（Thumb mid）
uint16 index_tip_force         # 食指指尖力（Index tip）
uint16 index_mid_force         # 食指指中力（Index mid）
uint16 middle_tip_force        # 中指指尖力（Middle tip）
uint16 middle_mid_force        # 中指指中力（Middle mid）
uint16 ring_tip_force          # 无名指指尖力（Ring tip）
uint16 ring_mid_force          # 无名指指中力（Ring mid）
uint16 pinky_tip_force         # 小拇指指尖力（Pinky tip）
uint16 pinky_mid_force         # 小拇指指中力（Pinky mid）
uint16 palm_force_1            # 手掌力1（Palm 1）
uint16 palm_force_2            # 手掌力2（Palm 2）
uint16 palm_force_3            # 手掌力3（Palm 3）

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
