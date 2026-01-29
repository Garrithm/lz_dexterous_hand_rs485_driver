/**
 * @file modbus_driver.hpp
 * @brief 灵巧手Modbus-RTU通信驱动（LZ Hand Modbus-RTU Driver）
 */

#ifndef LZ_HAND_RS485_DRIVER__MODBUS_DRIVER_HPP_
#define LZ_HAND_RS485_DRIVER__MODBUS_DRIVER_HPP_

#include <modbus/modbus.h>
#include <string>
#include <mutex>
#include <memory>
#include <optional>
#include <array>

#include "lz_hand_rs485_driver/hand_constants.hpp"

namespace lz_hand
{

/**
 * @brief Modbus通信异常（Modbus Communication Exception）
 */
class ModbusError : public std::runtime_error
{
public:
  explicit ModbusError(const std::string & message)
  : std::runtime_error(message) {}
};

/**
 * @brief 灵巧手Modbus-RTU驱动类（LZ Hand Modbus-RTU Driver Class）
 */
class LZHandModbusDriver
{
public:
  /**
   * @brief 构造函数（Constructor）
   * @param port 串口路径（Serial port path），如"/dev/ttyUSB0"
   * @param hand_id 手从机地址（Hand slave address）：1=右手，2=左手
   * @param baudrate 波特率（Baud rate）
   * @param auto_connect 是否自动连接（Whether to auto connect）
   */
  LZHandModbusDriver(
    const std::string & port,
    int hand_id = 1,
    int baudrate = HandConstants::DEFAULT_BAUDRATE,
    bool auto_connect = true);

  /**
   * @brief 析构函数（Destructor）
   */
  ~LZHandModbusDriver();

  // 禁用拷贝（Disable copy）
  LZHandModbusDriver(const LZHandModbusDriver &) = delete;
  LZHandModbusDriver & operator=(const LZHandModbusDriver &) = delete;

  /**
   * @brief 连接灵巧手（Connect to hand）
   * @return 成功返回true（Returns true if successful）
   */
  bool connect();

  /**
   * @brief 断开连接（Disconnect）
   */
  void disconnect();

  /**
   * @brief 检查连接状态（Check connection status）
   */
  bool is_connected() const { return connected_; }

  // ==================== 位置控制（Position Control） ====================

  /**
   * @brief 设置单关节位置（Set single joint position）
   * @param joint_index 关节索引（Joint index）：0-5
   * @param position 目标位置（Target position）：0-1000
   */
  bool set_joint_position(int joint_index, int position);

  /**
   * @brief 设置所有关节位置（Set all joint positions）
   * @param positions 6个位置值数组（Array of 6 position values）
   */
  bool set_all_positions(const std::array<int, 6> & positions);

  // ==================== 速度控制（Speed Control） ====================

  /**
   * @brief 设置单关节速度（Set single joint speed）
   * @param joint_index 关节索引（Joint index）：0-5
   * @param speed 目标速度（Target speed）：0-1000
   * @param gradual 是否渐进变化（Whether to change gradually）
   */
  bool set_joint_speed(int joint_index, int speed, bool gradual = true);

  /**
   * @brief 设置所有关节速度（Set all joint speeds）
   * @param speeds 6个速度值数组（Array of 6 speed values）
   * @param gradual 是否渐进变化（Whether to change gradually）
   */
  bool set_all_speeds(const std::array<int, 6> & speeds, bool gradual = true);

  // ==================== 力控制（Force Control） ====================

  /**
   * @brief 设置单关节力（Set single joint force）
   * @param joint_index 关节索引（Joint index）：0-5
   * @param force 目标力（Target force）：0-1000
   * @param gradual 是否渐进变化（Whether to change gradually）
   */
  bool set_joint_force(int joint_index, int force, bool gradual = true);

  /**
   * @brief 设置所有关节力（Set all joint forces）
   * @param forces 6个力值数组（Array of 6 force values）
   * @param gradual 是否渐进变化（Whether to change gradually）
   */
  bool set_all_forces(const std::array<int, 6> & forces, bool gradual = true);

  // ==================== 反馈读取（Feedback Reading） ====================

  /**
   * @brief 读取电机位置（Read motor positions）
   * @return 6个电机位置，失败返回nullopt（6 motor positions, or nullopt if failed）
   */
  std::optional<std::array<int, 6>> read_motor_positions();

  /**
   * @brief 读取关节角度（Read joint angles）
   * @return 10个角度值（度），失败返回nullopt（10 angles in degrees, or nullopt if failed）
   */
  std::optional<std::array<double, 10>> read_joint_angles();

  /**
   * @brief 读取所有反馈数据（Read all feedback data）
   * @return FeedbackData结构，失败返回nullopt（FeedbackData structure, or nullopt if failed）
   */
  std::optional<FeedbackData> read_all_feedback();

  /**
   * @brief 读取力反馈（Read force feedback）
   * @return 力值和有效标志，失败返回nullopt（Force values and validity flags, or nullopt if failed）
   */
  std::optional<std::pair<std::array<int, 13>, std::array<bool, 13>>> read_force_feedback();

  /**
   * @brief 读取控制状态（Read control state）
   * @return 位置、速度、力，失败返回nullopt（Positions, speeds, forces, or nullopt if failed）
   */
  std::optional<std::tuple<std::array<int, 6>, std::array<int, 6>, std::array<int, 6>>> read_control_state();

  // ==================== 高级控制（High-Level Control） ====================

  /**
   * @brief 设置完整手部姿态（Set complete hand pose）
   * @param positions 位置值（Position values）
   * @param speeds 可选速度值（Optional speed values）
   * @param forces 可选力值（Optional force values）
   */
  bool set_hand_pose(
    const std::array<int, 6> & positions,
    const std::array<int, 6> * speeds = nullptr,
    const std::array<int, 6> * forces = nullptr);

  /**
   * @brief 张开手（Open hand）
   * @param speed 运动速度（Movement speed）
   */
  bool open_hand(int speed = 500);

  /**
   * @brief 握拳（Close hand）
   * @param speed 运动速度（Movement speed）
   * @param force 握力（Grip force）
   */
  bool close_hand(int speed = 500, int force = 500);

  /**
   * @brief 捏取姿态（Pinch grip pose）
   * @param strength 握力强度（Grip strength）：0-1000
   */
  bool pinch_grip(int strength = 500);

  /**
   * @brief 指向姿态（Point gesture）
   */
  bool point_gesture();

  /**
   * @brief OK手势（OK gesture）
   */
  bool ok_gesture();

  /**
   * @brief 竖大拇指（Thumbs up gesture）
   */
  bool thumbs_up();

  // ==================== 配置（Configuration） ====================

  /**
   * @brief 设置渐进调节步长（Set gradual adjustment step size）
   * @param step_size 每次更新的最大变化量（Maximum change per update）
   */
  void set_gradual_step_size(int step_size);

  /**
   * @brief 获取手ID（Get hand ID）
   */
  int get_hand_id() const { return hand_id_; }

  /**
   * @brief 启用/禁用调试输出（Enable/disable debug output）
   */
  void set_debug(bool enable);
  
  /**
   * @brief 启用libmodbus原始调试（Enable libmodbus raw debug）
   */
  void set_raw_debug(bool enable);

private:
  std::string port_;
  int hand_id_;
  int baudrate_;

  modbus_t * ctx_ = nullptr;
  bool connected_ = false;
  mutable std::mutex mutex_;

  // 渐进调节状态（Gradual adjustment state）
  std::array<int, 6> last_speeds_ = {500, 500, 500, 500, 500, 500};
  std::array<int, 6> last_forces_ = {500, 500, 500, 500, 500, 500};
  int max_step_size_ = 100;
  
  // 调试标志（Debug flags）
  bool debug_enabled_ = false;

  /**
   * @brief 写入单个寄存器（Write single register）
   */
  bool write_register(uint16_t address, uint16_t value);

  /**
   * @brief 写入多个寄存器（Write multiple registers）
   */
  bool write_registers(uint16_t address, const uint16_t * values, int count);

  /**
   * @brief 读取多个寄存器（Read multiple registers）
   */
  bool read_registers(uint16_t address, uint16_t * values, int count);

  /**
   * @brief 应用渐进调节（Apply gradual adjustment）
   */
  int apply_gradual(int target, int current) const;
};

}  // namespace lz_hand

#endif  // LZ_HAND_RS485_DRIVER__MODBUS_DRIVER_HPP_
