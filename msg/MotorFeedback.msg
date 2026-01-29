# MotorFeedback.msg
# 电机位置反馈消息
# Motor position feedback message

# Header for timestamp and frame_id
std_msgs/Header header

# Hand selection: 1=right hand, 2=left hand
uint8 hand_id

# Motor positions (0-1000, unit: 1/1000 of max stroke)
uint16 thumb_rotation    # 大拇指翻转电机位置
uint16 thumb_bend        # 大拇指弯曲电机位置
uint16 index_bend        # 食指弯曲电机位置
uint16 middle_bend       # 中指弯曲电机位置
uint16 ring_bend         # 无名指弯曲电机位置
uint16 pinky_bend        # 小拇指弯曲电机位置
