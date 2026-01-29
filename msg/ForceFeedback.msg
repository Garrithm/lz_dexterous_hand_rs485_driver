# ForceFeedback.msg
# 力反馈消息（Force feedback message）

# 消息头（Header）
std_msgs/Header header

# 手ID：1=右手，2=左手（Hand ID: 1=right, 2=left）
uint8 hand_id

# 指尖力（Fingertip forces）：单位克（unit: gram），有效范围750-3000
uint16 thumb_tip         # 大拇指指尖（Thumb tip）
uint16 index_tip         # 食指指尖（Index tip）
uint16 middle_tip        # 中指指尖（Middle tip）
uint16 ring_tip          # 无名指指尖（Ring tip）
uint16 pinky_tip         # 小拇指指尖（Pinky tip）

# 指中力（Mid-finger forces）
uint16 thumb_mid         # 大拇指指中（Thumb mid）
uint16 index_mid         # 食指指中（Index mid）
uint16 middle_mid        # 中指指中（Middle mid）
uint16 ring_mid          # 无名指指中（Ring mid）
uint16 pinky_mid         # 小拇指指中（Pinky mid）

# 手掌力（Palm forces）
uint16 palm_1            # 手掌力1（Palm 1）
uint16 palm_2            # 手掌力2（Palm 2）
uint16 palm_3            # 手掌力3（Palm 3）

# 有效标志（Validity flags）：共13个，对应上述13个力值
bool[] valid_flags
