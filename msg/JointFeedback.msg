# JointFeedback.msg
# 单关节反馈消息（Single joint feedback message）

# 消息头（Header）
std_msgs/Header header

# 手ID：1=右手，2=左手（Hand ID: 1=right, 2=left）
uint8 hand_id

# 关节名称（Joint name）
string joint_name

# 电机位置（Motor position）：0-1000
uint16 motor_position

# 关节角度（Joint angles）：单位0.1度（unit: 0.1 degree）
uint16 mid_joint_angle     # 指中关节角度（Mid joint angle）
uint16 root_joint_angle    # 指根关节角度（Root joint angle）

# 力反馈（Force feedback）：单位克（unit: gram）
uint16 tip_force           # 指尖力（Tip force）
uint16 mid_force           # 指中力（Mid force）

# 有效标志（Validity flags）
bool tip_force_valid
bool mid_force_valid
