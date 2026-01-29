# ForceFeedback.msg
# 力反馈专用消息
# Force feedback message

# Header for timestamp and frame_id
std_msgs/Header header

# Hand selection: 1=right hand, 2=left hand
uint8 hand_id

# Force feedback values (unit: gram)
# Valid range: 750-3000, values outside this range are invalid

# Finger tip forces (指尖力)
uint16 thumb_tip         # 大拇指指尖
uint16 index_tip         # 食指指尖
uint16 middle_tip        # 中指指尖
uint16 ring_tip          # 无名指指尖
uint16 pinky_tip         # 小拇指指尖

# Finger mid forces (指中力)
uint16 thumb_mid         # 大拇指指中
uint16 index_mid         # 食指指中
uint16 middle_mid        # 中指指中
uint16 ring_mid          # 无名指指中
uint16 pinky_mid         # 小拇指指中

# Palm forces (手掌力)
uint16 palm_1            # 手掌力反馈(1)
uint16 palm_2            # 手掌力反馈(2)
uint16 palm_3            # 手掌力反馈(3)

# Validity flags (true if in valid range 750-3000)
bool[] valid_flags       # 13 flags corresponding to above values
