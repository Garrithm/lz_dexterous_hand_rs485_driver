# MotorFeedback.msg
# 电机位置反馈消息（Motor position feedback message）

# 消息头（Header）
std_msgs/Header header

# 手ID：1=右手，2=左手（Hand ID: 1=right, 2=left）
uint8 hand_id

# 电机位置（Motor positions）：0-1000
uint16 thumb_rotation
uint16 thumb_bend
uint16 index_bend
uint16 middle_bend
uint16 ring_bend
uint16 pinky_bend
