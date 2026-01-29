/**
 * @file hand_constants.hpp
 * @brief LZ Hand Constants and Register Map
 * 灵巧手常量定义和寄存器映射
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
 * @brief Hand ID (Slave Address)
 * 机械手从机地址
 */
enum class HandID : uint8_t
{
  RIGHT_HAND = 1,  // 右手
  LEFT_HAND = 2    // 左手
};

/**
 * @brief Joint Index
 * 关节索引
 */
enum class JointIndex : uint8_t
{
  THUMB_ROTATION = 0,  // 大拇指翻转
  THUMB_BEND = 1,      // 大拇指弯曲
  INDEX_BEND = 2,      // 食指弯曲
  MIDDLE_BEND = 3,     // 中指弯曲
  RING_BEND = 4,       // 无名指弯曲
  PINKY_BEND = 5       // 小拇指弯曲
};

/**
 * @brief Register Map
 * 寄存器地址映射
 */
struct RegisterMap
{
  // Position Control Registers (0-5)
  static constexpr uint16_t POS_START = 0;
  static constexpr uint16_t POS_COUNT = 6;

  // Speed Control Registers (6-11)
  static constexpr uint16_t SPEED_START = 6;
  static constexpr uint16_t SPEED_COUNT = 6;

  // Force Control Registers (12-17)
  static constexpr uint16_t FORCE_START = 12;
  static constexpr uint16_t FORCE_COUNT = 6;

  // Force Feedback Registers (18-30)
  static constexpr uint16_t FB_FORCE_START = 18;
  static constexpr uint16_t FB_FORCE_COUNT = 13;

  // Angle Feedback Registers (31-40)
  static constexpr uint16_t FB_ANGLE_START = 31;
  static constexpr uint16_t FB_ANGLE_COUNT = 10;

  // Motor Position Feedback Registers (41-46)
  static constexpr uint16_t FB_MOTOR_START = 41;
  static constexpr uint16_t FB_MOTOR_COUNT = 6;

  // All Feedback Registers (18-46)
  static constexpr uint16_t ALL_FEEDBACK_START = 18;
  static constexpr uint16_t ALL_FEEDBACK_COUNT = 29;
};

/**
 * @brief Hand Constants
 * 机械手常量
 */
struct HandConstants
{
  // Communication parameters
  static constexpr int DEFAULT_BAUDRATE = 115200;
  static constexpr int DEFAULT_BYTESIZE = 8;
  static constexpr char DEFAULT_PARITY = 'N';
  static constexpr int DEFAULT_STOPBITS = 1;
  static constexpr double DEFAULT_TIMEOUT = 0.1;

  // Value ranges
  static constexpr int POSITION_MIN = 0;
  static constexpr int POSITION_MAX = 1000;
  static constexpr int SPEED_MIN = 0;
  static constexpr int SPEED_MAX = 1000;
  static constexpr int FORCE_MIN = 0;
  static constexpr int FORCE_MAX = 1000;

  // Force feedback valid range
  static constexpr int FORCE_FEEDBACK_MIN = 750;
  static constexpr int FORCE_FEEDBACK_MAX = 3000;

  // Angle unit: 0.1 degree
  static constexpr double ANGLE_UNIT = 0.1;

  // Number of joints/sensors
  static constexpr int NUM_JOINTS = 6;
  static constexpr int NUM_FORCE_SENSORS = 13;
  static constexpr int NUM_ANGLE_SENSORS = 10;

  // Utility functions
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
 * @brief Hand State Data Structure
 * 机械手状态数据结构
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
 * @brief Feedback Data Structure
 * 反馈数据结构
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
