/**
 * @file modbus_driver.cpp
 * @brief LZ Hand Modbus-RTU Driver Implementation
 * 灵巧手Modbus-RTU通信驱动实现
 */

#include "lz_hand_rs485_driver/modbus_driver.hpp"
#include <cstring>
#include <algorithm>
#include <unistd.h>  // for tcdrain
#include <termios.h> // for tcdrain
#include <thread>    // for std::this_thread::sleep_for
#include <chrono>    // for std::chrono

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

  // Create Modbus RTU context
  // 创建Modbus RTU上下文，波特率115200
  ctx_ = modbus_new_rtu(
    port_.c_str(),
    baudrate_,
    HandConstants::DEFAULT_PARITY,
    HandConstants::DEFAULT_BYTESIZE,
    HandConstants::DEFAULT_STOPBITS);

  if (ctx_ == nullptr) {
    throw ModbusError("Failed to create Modbus context: " + std::string(modbus_strerror(errno)));
  }
  
  // 验证波特率设置（调试用）
  if (debug_enabled_) {
    fprintf(stderr, "[MODBUS] RTU context created: port=%s, baudrate=%d, parity=%c, bytesize=%d, stopbits=%d\n",
            port_.c_str(), baudrate_, HandConstants::DEFAULT_PARITY,
            HandConstants::DEFAULT_BYTESIZE, HandConstants::DEFAULT_STOPBITS);
  }

  // Set slave address
  if (modbus_set_slave(ctx_, hand_id_) == -1) {
    modbus_free(ctx_);
    ctx_ = nullptr;
    throw ModbusError("Failed to set slave address: " + std::string(modbus_strerror(errno)));
  }

  // Set response timeout (100ms)
  // 设置响应超时，参考新驱动使用100ms超时
  modbus_set_response_timeout(ctx_, 0, 100000);
  
  // Set byte timeout (1ms between bytes)
  // 设置字节间超时，确保能正确接收完整响应
  modbus_set_byte_timeout(ctx_, 0, 1000);

  // Connect (must be called before setting RS485 mode)
  // 连接（必须在设置RS485模式之前调用）
  if (modbus_connect(ctx_) == -1) {
    modbus_free(ctx_);
    ctx_ = nullptr;
    throw ModbusError("Failed to connect: " + std::string(modbus_strerror(errno)));
  }


  if (modbus_rtu_set_serial_mode(ctx_, MODBUS_RTU_RS485) == -1) {
    fprintf(stderr, "[WARNING] Failed to set RS485 mode: %s (continuing anyway)\n", 
            modbus_strerror(errno));
  }

  // Set RTS (Request To Send) mode for RS485 direction control
  // 设置RTS模式以控制RS485发送/接收方向
  // MODBUS_RTU_RTS_UP: RTS高电平有效（通常用于RS485收发器）
  // MODBUS_RTU_RTS_DOWN: RTS低电平有效
  if (modbus_rtu_set_rts(ctx_, MODBUS_RTU_RTS_UP) == -1) {
    // If RTS setting fails, try DOWN mode (some adapters use inverted RTS)
    // 如果UP模式失败，尝试DOWN模式（某些适配器使用反向RTS）
    if (modbus_rtu_set_rts(ctx_, MODBUS_RTU_RTS_DOWN) == -1) {
      // If both fail, log warning but continue (some adapters handle RTS automatically)
      // 如果都失败，记录警告但继续（某些适配器自动处理RTS）
      fprintf(stderr, "[WARNING] Failed to set RTS mode: %s (continuing anyway)\n", 
              modbus_strerror(errno));
    }
  }

  // Test connection by reading motor positions
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

  // Flush any pending data before writing
  // 写入前清空缓冲区
  modbus_flush(ctx_);

  // 写入寄存器
  int result = modbus_write_register(ctx_, address, value);
  
  if (result == -1) {
    if (debug_enabled_) {
      fprintf(stderr, "[MODBUS] write_register(addr=%d, value=%d) FAILED: %s (errno=%d)\n",
              address, value, modbus_strerror(errno), errno);
    }
    return false;
  }

  // 关键优化：等待数据发送完成（参考新驱动的tcdrain）
  // 这对于RS485半双工通信非常重要，确保数据完全发送后再切换方向
  int fd = modbus_get_socket(ctx_);
  if (fd >= 0) {
    tcdrain(fd);  // 等待所有数据发送完成
    // 小延迟确保设备有时间处理请求（参考新驱动的做法）
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // 验证写入响应（写单个寄存器时，响应应该与请求相同）
  // 读取响应来验证写入是否成功
  uint16_t read_value;
  if (modbus_read_registers(ctx_, address, 1, &read_value) != -1) {
    if (read_value != value) {
      if (debug_enabled_) {
        fprintf(stderr, "[MODBUS] write_register(addr=%d) verification failed: wrote=%d, read=%d\n",
                address, value, read_value);
      }
      // 不返回false，因为写入可能成功但设备还没更新
    }
  }

  if (debug_enabled_) {
    fprintf(stderr, "[MODBUS] write_register(addr=%d, value=%d) OK\n",
            address, value);
  }
  
  return true;
}

bool LZHandModbusDriver::write_registers(uint16_t address, const uint16_t * values, int count)
{
  if (!connected_ || ctx_ == nullptr) {
    return false;
  }

  // Flush any pending data before writing
  // 写入前清空缓冲区
  modbus_flush(ctx_);

  // 写入多个寄存器
  int result = modbus_write_registers(ctx_, address, count, values);
  
  if (result == -1) {
    if (debug_enabled_) {
      fprintf(stderr, "[MODBUS] write_registers(addr=%d, count=%d) FAILED: %s (errno=%d)\n",
              address, count, modbus_strerror(errno), errno);
    }
    return false;
  }

  int fd = modbus_get_socket(ctx_);
  if (fd >= 0) {
    tcdrain(fd);  // 等待所有数据发送完成
    // 小延迟确保设备有时间处理请求（参考新驱动的做法）
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // 验证写入响应（写多个寄存器时，可以读取第一个寄存器验证）
  if (count > 0) {
    uint16_t read_value;
    if (modbus_read_registers(ctx_, address, 1, &read_value) != -1) {
      if (read_value != values[0]) {
        if (debug_enabled_) {
          fprintf(stderr, "[MODBUS] write_registers(addr=%d) verification failed: wrote=%d, read=%d\n",
                  address, values[0], read_value);
        }
        // 不返回false，因为写入可能成功但设备还没更新
      }
    }
  }

  if (debug_enabled_) {
    fprintf(stderr, "[MODBUS] write_registers(addr=%d, count=%d) OK\n",
            address, count);
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

// ==================== Position Control ====================

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

// ==================== Speed Control ====================

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

// ==================== Force Control ====================

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

// ==================== Feedback Reading ====================

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

  // Parse force feedback (registers 18-30, offset 0-12)
  for (int i = 0; i < 13; ++i) {
    data.force_values[i] = static_cast<int>(regs[i]);
    data.force_valid[i] = HandConstants::is_force_valid(data.force_values[i]);
  }

  // Parse angle feedback (registers 31-40, offset 13-22)
  for (int i = 0; i < 10; ++i) {
    data.joint_angles[i] = HandConstants::angle_to_degrees(static_cast<int>(regs[13 + i]));
  }

  // Parse motor positions (registers 41-46, offset 23-28)
  for (int i = 0; i < 6; ++i) {
    data.motor_positions[i] = static_cast<int>(regs[23 + i]);
  }

  return data;
}

// ==================== High-Level Control ====================

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
  std::array<int, 6> positions = {
    1000,  // Thumb rotation (fully rotated for opposition)
    500,   // Thumb bend
    500,   // Index bend
    0,     // Middle straight
    0,     // Ring straight
    0      // Pinky straight
  };
  std::array<int, 6> forces = {strength, strength, strength, strength, strength, strength};
  return set_hand_pose(positions, nullptr, &forces);
}

bool LZHandModbusDriver::point_gesture()
{
  std::array<int, 6> positions = {
    500,   // Thumb rotation
    1000,  // Thumb bent
    0,     // Index straight (pointing)
    1000,  // Middle bent
    1000,  // Ring bent
    1000   // Pinky bent
  };
  return set_all_positions(positions);
}

bool LZHandModbusDriver::ok_gesture()
{
  std::array<int, 6> positions = {
    1000,  // Thumb rotation
    700,   // Thumb bend (touching index)
    700,   // Index bend (touching thumb)
    0,     // Middle straight
    0,     // Ring straight
    0      // Pinky straight
  };
  return set_all_positions(positions);
}

bool LZHandModbusDriver::thumbs_up()
{
  std::array<int, 6> positions = {
    0,     // Thumb rotation (not rotated)
    0,     // Thumb straight (up)
    1000,  // Index bent
    1000,  // Middle bent
    1000,  // Ring bent
    1000   // Pinky bent
  };
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
  fprintf(stderr, "[MODBUS] Debug mode %s\n", enable ? "ENABLED" : "DISABLED");
}

void LZHandModbusDriver::set_raw_debug(bool enable)
{
  if (ctx_ != nullptr) {
    modbus_set_debug(ctx_, enable ? TRUE : FALSE);
    fprintf(stderr, "[MODBUS] Raw debug mode %s (shows all bytes on stderr)\n", 
            enable ? "ENABLED" : "DISABLED");
  }
}

}  // namespace lz_hand
