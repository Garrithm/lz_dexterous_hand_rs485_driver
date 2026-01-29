/**
 * @file hand_constants.hpp
 * @brief 灵巧手常量定义和寄存器映射（LZ Hand Constants and Register Map）
 */

#ifndef LZ_HAND_RS485_DRIVER__HAND_CONSTANTS_HPP_
#define LZ_HAND_RS485_DRIVER__HAND_CONSTANTS_HPP_

#include <cstdint>
#include <array>
#include <string>
#include <vector>

namespace lz_hand
{

/**
 * @brief 手ID/从机地址（Hand ID / Slave Address）
 */
enum class HandID : uint8_t
{
  RIGHT_HAND = 1,  // 右手（Right hand）
  LEFT_HAND = 2    // 左手（Left hand）
};

/**
 * @brief 关节索引（Joint Index）
 */
enum class JointIndex : uint8_t
{
  THUMB_ROTATION = 0,  // 大拇指翻转（Thumb rotation）
  THUMB_BEND = 1,      // 大拇指弯曲（Thumb bend）
  INDEX_BEND = 2,      // 食指弯曲（Index bend）
  MIDDLE_BEND = 3,     // 中指弯曲（Middle bend）
  RING_BEND = 4,       // 无名指弯曲（Ring bend）
  PINKY_BEND = 5       // 小拇指弯曲（Pinky bend）
};

/**
 * @brief 寄存器映射（Register Map）
 */
struct RegisterMap
{
  // 位置控制寄存器（Position control registers）：0-5
  static constexpr uint16_t POS_START = 0;
  static constexpr uint16_t POS_COUNT = 6;

  // 速度控制寄存器（Speed control registers）：6-11
  static constexpr uint16_t SPEED_START = 6;
  static constexpr uint16_t SPEED_COUNT = 6;

  // 力控制寄存器（Force control registers）：12-17
  static constexpr uint16_t FORCE_START = 12;
  static constexpr uint16_t FORCE_COUNT = 6;

  // 力反馈寄存器（Force feedback registers）：18-30
  static constexpr uint16_t FB_FORCE_START = 18;
  static constexpr uint16_t FB_FORCE_COUNT = 13;

  // 角度反馈寄存器（Angle feedback registers）：31-40
  static constexpr uint16_t FB_ANGLE_START = 31;
  static constexpr uint16_t FB_ANGLE_COUNT = 10;

  // 电机位置反馈寄存器（Motor position feedback registers）：41-46
  static constexpr uint16_t FB_MOTOR_START = 41;
  static constexpr uint16_t FB_MOTOR_COUNT = 6;

  // 所有反馈寄存器（All feedback registers）：18-46
  static constexpr uint16_t ALL_FEEDBACK_START = 18;
  static constexpr uint16_t ALL_FEEDBACK_COUNT = 29;
};

/**
 * @brief 机械手常量（Hand Constants）
 */
struct HandConstants
{
  // 通信参数（Communication parameters）
  static constexpr int DEFAULT_BAUDRATE = 115200;
  static constexpr int DEFAULT_BYTESIZE = 8;
  static constexpr char DEFAULT_PARITY = 'N';
  static constexpr int DEFAULT_STOPBITS = 1;
  static constexpr double DEFAULT_TIMEOUT = 0.1;

  // 数值范围（Value ranges）
  static constexpr int POSITION_MIN = 0;
  static constexpr int POSITION_MAX = 1000;
  static constexpr int SPEED_MIN = 0;
  static constexpr int SPEED_MAX = 1000;
  static constexpr int FORCE_MIN = 0;
  static constexpr int FORCE_MAX = 1000;

  // 力反馈有效范围克（Force feedback valid range in grams）
  static constexpr int FORCE_FEEDBACK_MIN = 750;
  static constexpr int FORCE_FEEDBACK_MAX = 3000;

  // 角度单位：0.1度（Angle unit: 0.1 degree）
  static constexpr double ANGLE_UNIT = 0.1;

  // 关节/传感器数量（Number of joints/sensors）
  static constexpr int NUM_JOINTS = 6;
  static constexpr int NUM_FORCE_SENSORS = 13;
  static constexpr int NUM_ANGLE_SENSORS = 10;

  // 工具函数（Utility functions）
  static inline bool is_force_valid(int force_value)
  {
    return force_value >= FORCE_FEEDBACK_MIN && force_value <= FORCE_FEEDBACK_MAX;
  }

  static inline int clamp_position(int value)
  {
    return std::max(POSITION_MIN, std::min(value, POSITION_MAX));
  }

  static inline int clamp_speed(int value)
  {
    return std::max(SPEED_MIN, std::min(value, SPEED_MAX));
  }

  static inline int clamp_force(int value)
  {
    return std::max(FORCE_MIN, std::min(value, FORCE_MAX));
  }

  static inline double angle_to_degrees(int raw_value)
  {
    return raw_value * ANGLE_UNIT;
  }
};

/**
 * @brief 机械手状态数据结构（Hand State Data Structure）
 */
struct HandState
{
  int hand_id = 1;
  bool connected = false;

  std::array<int, 6> positions = {0};
  std::array<int, 6> motor_positions = {0};
  std::array<int, 6> speeds = {500, 500, 500, 500, 500, 500};
  std::array<int, 6> forces = {500, 500, 500, 500, 500, 500};

  std::array<double, 10> joint_angles = {0.0};

  std::array<int, 10> force_feedback = {0};
  std::array<bool, 10> force_feedback_valid = {false};

  std::array<int, 3> palm_forces = {0};
  std::array<bool, 3> palm_forces_valid = {false};
};

/**
 * @brief 反馈数据结构（Feedback Data Structure）
 */
struct FeedbackData
{
  std::array<int, 13> force_values = {0};
  std::array<bool, 13> force_valid = {false};
  std::array<double, 10> joint_angles = {0.0};
  std::array<int, 6> motor_positions = {0};
};

}  // namespace lz_hand

#endif  // LZ_HAND_RS485_DRIVER__HAND_CONSTANTS_HPP_
