# JointFeedback.msg
# 关节反馈消息（简化版，用于单独查询）
# Joint feedback message (simplified, for individual queries)

# Header for timestamp and frame_id
std_msgs/Header header

# Hand selection: 1=right hand, 2=left hand
uint8 hand_id

# Joint name
string joint_name

# Motor position (0-1000, unit: 1/1000 of max stroke)
uint16 motor_position

# Joint angles (unit: 0.1 degree)
# For thumb: mid_angle and root_angle both available
# For other fingers: mid_angle and root_angle
uint16 mid_joint_angle
uint16 root_joint_angle

# Force feedback at fingertip and mid-joint (unit: gram)
uint16 tip_force
uint16 mid_force

# Validity flags
bool tip_force_valid
bool mid_force_valid
