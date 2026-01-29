/**
 * @file modbus_driver.hpp
 * @brief LZ Hand Modbus-RTU Driver
 * 灵巧手Modbus-RTU通信驱动
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
 * @brief Modbus Communication Exception
 */
class ModbusError : public std::runtime_error
{
public:
  explicit ModbusError(const std::string & message)
  : std::runtime_error(message) {}
};

/**
 * @brief LZ Hand Modbus-RTU Driver Class
 * 灵巧手Modbus-RTU驱动类
 */
class LZHandModbusDriver
{
public:
  /**
   * @brief Constructor
   * @param port Serial port path (e.g., "/dev/ttyUSB0")
   * @param hand_id Hand slave address (1=right, 2=left)
   * @param baudrate Communication baud rate
   * @param auto_connect Whether to connect automatically
   */
  LZHandModbusDriver(
    const std::string & port,
    int hand_id = 1,
    int baudrate = HandConstants::DEFAULT_BAUDRATE,
    bool auto_connect = true);

  /**
   * @brief Destructor
   */
  ~LZHandModbusDriver();

  // Disable copy
  LZHandModbusDriver(const LZHandModbusDriver &) = delete;
  LZHandModbusDriver & operator=(const LZHandModbusDriver &) = delete;

  /**
   * @brief Connect to the hand
   * @return true if successful
   */
  bool connect();

  /**
   * @brief Disconnect from the hand
   */
  void disconnect();

  /**
   * @brief Check connection status
   */
  bool is_connected() const { return connected_; }

  // ==================== Position Control ====================

  /**
   * @brief Set single joint position
   * @param joint_index Joint index (0-5)
   * @param position Target position (0-1000)
   * @return true if successful
   */
  bool set_joint_position(int joint_index, int position);

  /**
   * @brief Set all joint positions
   * @param positions Array of 6 position values
   * @return true if successful
   */
  bool set_all_positions(const std::array<int, 6> & positions);

  // ==================== Speed Control ====================

  /**
   * @brief Set single joint speed
   * @param joint_index Joint index (0-5)
   * @param speed Target speed (0-1000)
   * @param gradual Whether to change gradually
   * @return true if successful
   */
  bool set_joint_speed(int joint_index, int speed, bool gradual = true);

  /**
   * @brief Set all joint speeds
   * @param speeds Array of 6 speed values
   * @param gradual Whether to change gradually
   * @return true if successful
   */
  bool set_all_speeds(const std::array<int, 6> & speeds, bool gradual = true);

  // ==================== Force Control ====================

  /**
   * @brief Set single joint force
   * @param joint_index Joint index (0-5)
   * @param force Target force (0-1000)
   * @param gradual Whether to change gradually
   * @return true if successful
   */
  bool set_joint_force(int joint_index, int force, bool gradual = true);

  /**
   * @brief Set all joint forces
   * @param forces Array of 6 force values
   * @param gradual Whether to change gradually
   * @return true if successful
   */
  bool set_all_forces(const std::array<int, 6> & forces, bool gradual = true);

  // ==================== Feedback Reading ====================

  /**
   * @brief Read motor positions
   * @return Array of 6 motor positions, or nullopt if failed
   */
  std::optional<std::array<int, 6>> read_motor_positions();

  /**
   * @brief Read joint angles
   * @return Array of 10 angles in degrees, or nullopt if failed
   */
  std::optional<std::array<double, 10>> read_joint_angles();

  /**
   * @brief Read all feedback data
   * @return FeedbackData structure, or nullopt if failed
   */
  std::optional<FeedbackData> read_all_feedback();

  // ==================== High-Level Control ====================

  /**
   * @brief Set complete hand pose
   * @param positions Position values
   * @param speeds Optional speed values
   * @param forces Optional force values
   * @return true if successful
   */
  bool set_hand_pose(
    const std::array<int, 6> & positions,
    const std::array<int, 6> * speeds = nullptr,
    const std::array<int, 6> * forces = nullptr);

  /**
   * @brief Open the hand
   * @param speed Movement speed
   */
  bool open_hand(int speed = 500);

  /**
   * @brief Close the hand
   * @param speed Movement speed
   * @param force Grip force
   */
  bool close_hand(int speed = 500, int force = 500);

  /**
   * @brief Pinch grip pose (thumb opposes index finger)
   * 捏取姿态
   * @param strength Grip strength (0-1000)
   */
  bool pinch_grip(int strength = 500);

  /**
   * @brief Pointing gesture (index extended, others bent)
   * 指向姿态
   */
  bool point_gesture();

  /**
   * @brief OK gesture
   * OK手势
   */
  bool ok_gesture();

  /**
   * @brief Thumbs up gesture
   * 竖大拇指
   */
  bool thumbs_up();

  // ==================== Additional Feedback ====================

  /**
   * @brief Read force feedback
   * @return Force values and validity flags, or nullopt if failed
   */
  std::optional<std::pair<std::array<int, 13>, std::array<bool, 13>>> read_force_feedback();

  /**
   * @brief Read current control state (positions, speeds, forces)
   * @return Control state data, or nullopt if failed
   */
  std::optional<std::tuple<std::array<int, 6>, std::array<int, 6>, std::array<int, 6>>> read_control_state();

  // ==================== Configuration ====================

  /**
   * @brief Set gradual adjustment step size
   * @param step_size Maximum change per update
   */
  void set_gradual_step_size(int step_size);

  /**
   * @brief Get hand ID
   */
  int get_hand_id() const { return hand_id_; }

  /**
   * @brief Enable/disable debug output
   * @param enable true to enable debug messages
   */
  void set_debug(bool enable);
  
  /**
   * @brief Enable libmodbus raw debug (shows all bytes)
   * @param enable true to enable
   */
  void set_raw_debug(bool enable);

private:
  std::string port_;
  int hand_id_;
  int baudrate_;

  modbus_t * ctx_ = nullptr;
  bool connected_ = false;
  mutable std::mutex mutex_;

  // Gradual adjustment state
  std::array<int, 6> last_speeds_ = {500, 500, 500, 500, 500, 500};
  std::array<int, 6> last_forces_ = {500, 500, 500, 500, 500, 500};
  int max_step_size_ = 100;
  
  // Debug flags
  bool debug_enabled_ = false;

  /**
   * @brief Write single register
   */
  bool write_register(uint16_t address, uint16_t value);

  /**
   * @brief Write multiple registers
   */
  bool write_registers(uint16_t address, const uint16_t * values, int count);

  /**
   * @brief Read multiple registers
   */
  bool read_registers(uint16_t address, uint16_t * values, int count);

  /**
   * @brief Apply gradual adjustment
   */
  int apply_gradual(int target, int current) const;
};

}  // namespace lz_hand

#endif  // LZ_HAND_RS485_DRIVER__MODBUS_DRIVER_HPP_
