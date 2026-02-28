# ForceFeedback.msg
# 力反馈消息（Force feedback message）

# 消息头（Header）
std_msgs/Header header

# 手ID：1=右手，2=左手（Hand ID: 1=right, 2=left）
uint8 hand_id

# 指尖力（Fingertip forces）：单位克（unit: gram），有效范围750-3000
uint16 thumb_tip
uint16 index_tip
uint16 middle_tip
uint16 ring_tip
uint16 pinky_tip

# 指中力（Mid-finger forces）
uint16 thumb_mid
uint16 index_mid
uint16 middle_mid
uint16 ring_mid
uint16 pinky_mid

# 手掌力（Palm forces）
uint16 palm_1
uint16 palm_2
uint16 palm_3

# 有效标志（Validity flags）：共13个，对应上述13个力值
bool[] valid_flags
