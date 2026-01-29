/**
 * @file modbus_driver.cpp
 * @brief 灵巧手Modbus-RTU通信驱动实现（LZ Hand Modbus-RTU Driver Implementation）
 */

#include "lz_hand_rs485_driver/modbus_driver.hpp"
#include <cstring>
#include <algorithm>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <chrono>

namespace lz_hand
{

LZHandModbusDriver::LZHandModbusDriver(
  const std::string & port,
  int hand_id,
  int baudrate,
  bool auto_connect)
: port_(port),
  hand_id_(hand_id),
  baudrate_(baudrate)
{
  if (auto_connect) {
    connect();
  }
}

LZHandModbusDriver::~LZHandModbusDriver()
{
  disconnect();
}

bool LZHandModbusDriver::connect()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (connected_) {
    return true;
  }

  // 创建Modbus RTU上下文（Create Modbus RTU context）
  ctx_ = modbus_new_rtu(
    port_.c_str(),
    baudrate_,
    HandConstants::DEFAULT_PARITY,
    HandConstants::DEFAULT_BYTESIZE,
    HandConstants::DEFAULT_STOPBITS);

  if (ctx_ == nullptr) {
    throw ModbusError("Failed to create Modbus context: " + std::string(modbus_strerror(errno)));
  }
  
  if (debug_enabled_) {
    fprintf(stderr, "[MODBUS] RTU context: port=%s, baudrate=%d\n", port_.c_str(), baudrate_);
  }

  // 设置从机地址（Set slave address）
  if (modbus_set_slave(ctx_, hand_id_) == -1) {
    modbus_free(ctx_);
    ctx_ = nullptr;
    throw ModbusError("Failed to set slave address: " + std::string(modbus_strerror(errno)));
  }

  // 设置超时（Set timeouts）
  modbus_set_response_timeout(ctx_, 0, 100000);  // 100ms响应超时（response timeout）
  modbus_set_byte_timeout(ctx_, 0, 1000);        // 1ms字节超时（byte timeout）

  // 连接（Connect）
  if (modbus_connect(ctx_) == -1) {
    modbus_free(ctx_);
    ctx_ = nullptr;
    throw ModbusError("Failed to connect: " + std::string(modbus_strerror(errno)));
  }

  // 设置RS485模式（Set RS485 mode）
  if (modbus_rtu_set_serial_mode(ctx_, MODBUS_RTU_RS485) == -1) {
    fprintf(stderr, "[WARNING] Failed to set RS485 mode: %s (continuing anyway)\n", 
            modbus_strerror(errno));
  }

  // 设置RTS模式（Set RTS mode）
  if (modbus_rtu_set_rts(ctx_, MODBUS_RTU_RTS_UP) == -1) {
    if (modbus_rtu_set_rts(ctx_, MODBUS_RTU_RTS_DOWN) == -1) {
      fprintf(stderr, "[WARNING] Failed to set RTS mode: %s (continuing anyway)\n", 
              modbus_strerror(errno));
    }
  }

  // 测试连接（Test connection）
  uint16_t test_reg;
  if (modbus_read_registers(ctx_, RegisterMap::FB_MOTOR_START, 1, &test_reg) == -1) {
    modbus_close(ctx_);
    modbus_free(ctx_);
    ctx_ = nullptr;
    throw ModbusError("Connection test failed: " + std::string(modbus_strerror(errno)));
  }

  connected_ = true;
  return true;
}

void LZHandModbusDriver::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (ctx_ != nullptr) {
    modbus_close(ctx_);
    modbus_free(ctx_);
    ctx_ = nullptr;
  }
  connected_ = false;
}

bool LZHandModbusDriver::write_register(uint16_t address, uint16_t value)
{
  if (!connected_ || ctx_ == nullptr) {
    return false;
  }

  modbus_flush(ctx_);  // 清空缓冲区（Flush buffer）

  int result = modbus_write_register(ctx_, address, value);
  
  if (result == -1) {
    if (debug_enabled_) {
      fprintf(stderr, "[MODBUS] write_register(addr=%d, value=%d) FAILED: %s\n",
              address, value, modbus_strerror(errno));
    }
    return false;
  }

  // 等待数据发送完成（Wait for data to be sent）
  int fd = modbus_get_socket(ctx_);
  if (fd >= 0) {
    tcdrain(fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // 验证写入（Verify write）
  uint16_t read_value;
  if (modbus_read_registers(ctx_, address, 1, &read_value) != -1) {
    if (read_value != value && debug_enabled_) {
      fprintf(stderr, "[MODBUS] write verification: wrote=%d, read=%d\n", value, read_value);
    }
  }

  if (debug_enabled_) {
    fprintf(stderr, "[MODBUS] write_register(addr=%d, value=%d) OK\n", address, value);
  }
  
  return true;
}

bool LZHandModbusDriver::write_registers(uint16_t address, const uint16_t * values, int count)
{
  if (!connected_ || ctx_ == nullptr) {
    return false;
  }

  modbus_flush(ctx_);

  int result = modbus_write_registers(ctx_, address, count, values);
  
  if (result == -1) {
    if (debug_enabled_) {
      fprintf(stderr, "[MODBUS] write_registers(addr=%d, count=%d) FAILED: %s\n",
              address, count, modbus_strerror(errno));
    }
    return false;
  }

  int fd = modbus_get_socket(ctx_);
  if (fd >= 0) {
    tcdrain(fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (debug_enabled_) {
    fprintf(stderr, "[MODBUS] write_registers(addr=%d, count=%d) OK\n", address, count);
  }
  
  return true;
}

bool LZHandModbusDriver::read_registers(uint16_t address, uint16_t * values, int count)
{
  if (!connected_ || ctx_ == nullptr) {
    return false;
  }

  int result = modbus_read_registers(ctx_, address, count, values);
  return result != -1;
}

int LZHandModbusDriver::apply_gradual(int target, int current) const
{
  int diff = target - current;
  if (std::abs(diff) > max_step_size_) {
    return current + (diff > 0 ? max_step_size_ : -max_step_size_);
  }
  return target;
}

// ==================== 位置控制（Position Control） ====================

bool LZHandModbusDriver::set_joint_position(int joint_index, int position)
{
  if (joint_index < 0 || joint_index >= 6) {
    return false;
  }

  position = HandConstants::clamp_position(position);

  std::lock_guard<std::mutex> lock(mutex_);
  return write_register(RegisterMap::POS_START + joint_index, static_cast<uint16_t>(position));
}

bool LZHandModbusDriver::set_all_positions(const std::array<int, 6> & positions)
{
  std::array<uint16_t, 6> values;
  for (size_t i = 0; i < 6; ++i) {
    values[i] = static_cast<uint16_t>(HandConstants::clamp_position(positions[i]));
  }

  std::lock_guard<std::mutex> lock(mutex_);
  return write_registers(RegisterMap::POS_START, values.data(), 6);
}

// ==================== 速度控制（Speed Control） ====================

bool LZHandModbusDriver::set_joint_speed(int joint_index, int speed, bool gradual)
{
  if (joint_index < 0 || joint_index >= 6) {
    return false;
  }

  speed = HandConstants::clamp_speed(speed);

  if (gradual) {
    speed = apply_gradual(speed, last_speeds_[joint_index]);
  }
  last_speeds_[joint_index] = speed;

  std::lock_guard<std::mutex> lock(mutex_);
  return write_register(RegisterMap::SPEED_START + joint_index, static_cast<uint16_t>(speed));
}

bool LZHandModbusDriver::set_all_speeds(const std::array<int, 6> & speeds, bool gradual)
{
  std::array<uint16_t, 6> values;
  for (size_t i = 0; i < 6; ++i) {
    int speed = HandConstants::clamp_speed(speeds[i]);
    if (gradual) {
      speed = apply_gradual(speed, last_speeds_[i]);
    }
    last_speeds_[i] = speed;
    values[i] = static_cast<uint16_t>(speed);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  return write_registers(RegisterMap::SPEED_START, values.data(), 6);
}

// ==================== 力控制（Force Control） ====================

bool LZHandModbusDriver::set_joint_force(int joint_index, int force, bool gradual)
{
  if (joint_index < 0 || joint_index >= 6) {
    return false;
  }

  force = HandConstants::clamp_force(force);

  if (gradual) {
    force = apply_gradual(force, last_forces_[joint_index]);
  }
  last_forces_[joint_index] = force;

  std::lock_guard<std::mutex> lock(mutex_);
  return write_register(RegisterMap::FORCE_START + joint_index, static_cast<uint16_t>(force));
}

bool LZHandModbusDriver::set_all_forces(const std::array<int, 6> & forces, bool gradual)
{
  std::array<uint16_t, 6> values;
  for (size_t i = 0; i < 6; ++i) {
    int force = HandConstants::clamp_force(forces[i]);
    if (gradual) {
      force = apply_gradual(force, last_forces_[i]);
    }
    last_forces_[i] = force;
    values[i] = static_cast<uint16_t>(force);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  return write_registers(RegisterMap::FORCE_START, values.data(), 6);
}

// ==================== 反馈读取（Feedback Reading） ====================

std::optional<std::array<int, 6>> LZHandModbusDriver::read_motor_positions()
{
  std::array<uint16_t, 6> regs;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!read_registers(RegisterMap::FB_MOTOR_START, regs.data(), 6)) {
    return std::nullopt;
  }

  std::array<int, 6> result;
  for (size_t i = 0; i < 6; ++i) {
    result[i] = static_cast<int>(regs[i]);
  }
  return result;
}

std::optional<std::array<double, 10>> LZHandModbusDriver::read_joint_angles()
{
  std::array<uint16_t, 10> regs;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!read_registers(RegisterMap::FB_ANGLE_START, regs.data(), 10)) {
    return std::nullopt;
  }

  std::array<double, 10> result;
  for (size_t i = 0; i < 10; ++i) {
    result[i] = HandConstants::angle_to_degrees(static_cast<int>(regs[i]));
  }
  return result;
}

std::optional<FeedbackData> LZHandModbusDriver::read_all_feedback()
{
  std::array<uint16_t, 29> regs;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!read_registers(RegisterMap::ALL_FEEDBACK_START, regs.data(), 29)) {
      return std::nullopt;
    }
  }

  FeedbackData data;

  // 解析力反馈（Parse force feedback）：寄存器18-30
  for (int i = 0; i < 13; ++i) {
    data.force_values[i] = static_cast<int>(regs[i]);
    data.force_valid[i] = HandConstants::is_force_valid(data.force_values[i]);
  }

  // 解析角度反馈（Parse angle feedback）：寄存器31-40
  for (int i = 0; i < 10; ++i) {
    data.joint_angles[i] = HandConstants::angle_to_degrees(static_cast<int>(regs[13 + i]));
  }

  // 解析电机位置（Parse motor positions）：寄存器41-46
  for (int i = 0; i < 6; ++i) {
    data.motor_positions[i] = static_cast<int>(regs[23 + i]);
  }

  return data;
}

// ==================== 高级控制（High-Level Control） ====================

bool LZHandModbusDriver::set_hand_pose(
  const std::array<int, 6> & positions,
  const std::array<int, 6> * speeds,
  const std::array<int, 6> * forces)
{
  bool success = true;

  if (speeds != nullptr) {
    success = success && set_all_speeds(*speeds);
  }

  if (forces != nullptr) {
    success = success && set_all_forces(*forces);
  }

  success = success && set_all_positions(positions);

  return success;
}

bool LZHandModbusDriver::open_hand(int speed)
{
  std::array<int, 6> speeds = {speed, speed, speed, speed, speed, speed};
  std::array<int, 6> positions = {0, 0, 0, 0, 0, 0};
  return set_hand_pose(positions, &speeds, nullptr);
}

bool LZHandModbusDriver::close_hand(int speed, int force)
{
  std::array<int, 6> speeds = {speed, speed, speed, speed, speed, speed};
  std::array<int, 6> forces = {force, force, force, force, force, force};
  std::array<int, 6> positions = {1000, 1000, 1000, 1000, 1000, 1000};
  return set_hand_pose(positions, &speeds, &forces);
}

bool LZHandModbusDriver::pinch_grip(int strength)
{
  std::array<int, 6> positions = {1000, 500, 500, 0, 0, 0};
  std::array<int, 6> forces = {strength, strength, strength, strength, strength, strength};
  return set_hand_pose(positions, nullptr, &forces);
}

bool LZHandModbusDriver::point_gesture()
{
  std::array<int, 6> positions = {500, 1000, 0, 1000, 1000, 1000};
  return set_all_positions(positions);
}

bool LZHandModbusDriver::ok_gesture()
{
  std::array<int, 6> positions = {1000, 700, 700, 0, 0, 0};
  return set_all_positions(positions);
}

bool LZHandModbusDriver::thumbs_up()
{
  std::array<int, 6> positions = {0, 0, 1000, 1000, 1000, 1000};
  return set_all_positions(positions);
}

std::optional<std::pair<std::array<int, 13>, std::array<bool, 13>>> LZHandModbusDriver::read_force_feedback()
{
  std::array<uint16_t, 13> regs;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!read_registers(RegisterMap::FB_FORCE_START, regs.data(), 13)) {
    return std::nullopt;
  }

  std::array<int, 13> forces;
  std::array<bool, 13> valid;
  for (size_t i = 0; i < 13; ++i) {
    forces[i] = static_cast<int>(regs[i]);
    valid[i] = HandConstants::is_force_valid(forces[i]);
  }

  return std::make_pair(forces, valid);
}

std::optional<std::tuple<std::array<int, 6>, std::array<int, 6>, std::array<int, 6>>> LZHandModbusDriver::read_control_state()
{
  std::array<uint16_t, 18> regs;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!read_registers(RegisterMap::POS_START, regs.data(), 18)) {
    return std::nullopt;
  }

  std::array<int, 6> positions, speeds, forces;
  for (size_t i = 0; i < 6; ++i) {
    positions[i] = static_cast<int>(regs[i]);
    speeds[i] = static_cast<int>(regs[6 + i]);
    forces[i] = static_cast<int>(regs[12 + i]);
  }

  return std::make_tuple(positions, speeds, forces);
}

void LZHandModbusDriver::set_gradual_step_size(int step_size)
{
  max_step_size_ = std::max(1, std::min(step_size, 1000));
}

void LZHandModbusDriver::set_debug(bool enable)
{
  debug_enabled_ = enable;
  fprintf(stderr, "[MODBUS] Debug %s\n", enable ? "ENABLED" : "DISABLED");
}

void LZHandModbusDriver::set_raw_debug(bool enable)
{
  if (ctx_ != nullptr) {
    modbus_set_debug(ctx_, enable ? TRUE : FALSE);
    fprintf(stderr, "[MODBUS] Raw debug %s\n", enable ? "ENABLED" : "DISABLED");
  }
}

}  // namespace lz_hand
