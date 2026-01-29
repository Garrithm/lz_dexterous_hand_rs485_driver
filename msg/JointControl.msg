# JointControl.msg
# 单关节控制消息（Single joint control message）

# 消息头（Header）
std_msgs/Header header

# 手ID：1=右手，2=左手（Hand ID: 1=right, 2=left）
uint8 hand_id

# 关节索引（Joint index）：0-5
# 0: 大拇指翻转（thumb rotation）
# 1: 大拇指弯曲（thumb bend）
# 2: 食指弯曲（index bend）
# 3: 中指弯曲（middle bend）
# 4: 无名指弯曲（ring bend）
# 5: 小拇指弯曲（pinky bend）
uint8 joint_index

# 位置（Position）：0-1000
uint16 position

# 速度（Speed）：0-1000，可选
uint16 speed

# 力（Force）：0-1000，可选
uint16 force
