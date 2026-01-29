/**
 * @file python_bindings.cpp
 * @brief Python bindings for LZ Hand Modbus Driver using pybind11
 * 使用pybind11创建的Python绑定
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

#include "lz_hand_rs485_driver/modbus_driver.hpp"
#include "lz_hand_rs485_driver/hand_constants.hpp"

namespace py = pybind11;
using namespace lz_hand;

PYBIND11_MODULE(lz_hand_driver_cpp, m) {
  m.doc() = "LZ Hand Modbus Driver C++ bindings (灵巧手Modbus驱动C++绑定)";

  // ==================== Enums ====================

  py::enum_<HandID>(m, "HandID", "Hand ID / 机械手ID")
    .value("RIGHT_HAND", HandID::RIGHT_HAND, "右手")
    .value("LEFT_HAND", HandID::LEFT_HAND, "左手")
    .export_values();

  py::enum_<JointIndex>(m, "JointIndex", "Joint Index / 关节索引")
    .value("THUMB_ROTATION", JointIndex::THUMB_ROTATION, "大拇指翻转")
    .value("THUMB_BEND", JointIndex::THUMB_BEND, "大拇指弯曲")
    .value("INDEX_BEND", JointIndex::INDEX_BEND, "食指弯曲")
    .value("MIDDLE_BEND", JointIndex::MIDDLE_BEND, "中指弯曲")
    .value("RING_BEND", JointIndex::RING_BEND, "无名指弯曲")
    .value("PINKY_BEND", JointIndex::PINKY_BEND, "小拇指弯曲")
    .export_values();

  // ==================== RegisterMap ====================

  py::class_<RegisterMap>(m, "RegisterMap", "Register Map / 寄存器映射")
    .def_readonly_static("POS_START", &RegisterMap::POS_START)
    .def_readonly_static("POS_COUNT", &RegisterMap::POS_COUNT)
    .def_readonly_static("SPEED_START", &RegisterMap::SPEED_START)
    .def_readonly_static("SPEED_COUNT", &RegisterMap::SPEED_COUNT)
    .def_readonly_static("FORCE_START", &RegisterMap::FORCE_START)
    .def_readonly_static("FORCE_COUNT", &RegisterMap::FORCE_COUNT)
    .def_readonly_static("FB_FORCE_START", &RegisterMap::FB_FORCE_START)
    .def_readonly_static("FB_FORCE_COUNT", &RegisterMap::FB_FORCE_COUNT)
    .def_readonly_static("FB_ANGLE_START", &RegisterMap::FB_ANGLE_START)
    .def_readonly_static("FB_ANGLE_COUNT", &RegisterMap::FB_ANGLE_COUNT)
    .def_readonly_static("FB_MOTOR_START", &RegisterMap::FB_MOTOR_START)
    .def_readonly_static("FB_MOTOR_COUNT", &RegisterMap::FB_MOTOR_COUNT)
    .def_readonly_static("ALL_FEEDBACK_START", &RegisterMap::ALL_FEEDBACK_START)
    .def_readonly_static("ALL_FEEDBACK_COUNT", &RegisterMap::ALL_FEEDBACK_COUNT);

  // ==================== HandConstants ====================

  py::class_<HandConstants>(m, "HandConstants", "Hand Constants / 机械手常量")
    .def_readonly_static("DEFAULT_BAUDRATE", &HandConstants::DEFAULT_BAUDRATE)
    .def_readonly_static("DEFAULT_BYTESIZE", &HandConstants::DEFAULT_BYTESIZE)
    .def_readonly_static("DEFAULT_PARITY", &HandConstants::DEFAULT_PARITY)
    .def_readonly_static("DEFAULT_STOPBITS", &HandConstants::DEFAULT_STOPBITS)
    .def_readonly_static("DEFAULT_TIMEOUT", &HandConstants::DEFAULT_TIMEOUT)
    .def_readonly_static("POSITION_MIN", &HandConstants::POSITION_MIN)
    .def_readonly_static("POSITION_MAX", &HandConstants::POSITION_MAX)
    .def_readonly_static("SPEED_MIN", &HandConstants::SPEED_MIN)
    .def_readonly_static("SPEED_MAX", &HandConstants::SPEED_MAX)
    .def_readonly_static("FORCE_MIN", &HandConstants::FORCE_MIN)
    .def_readonly_static("FORCE_MAX", &HandConstants::FORCE_MAX)
    .def_readonly_static("FORCE_FEEDBACK_MIN", &HandConstants::FORCE_FEEDBACK_MIN)
    .def_readonly_static("FORCE_FEEDBACK_MAX", &HandConstants::FORCE_FEEDBACK_MAX)
    .def_readonly_static("ANGLE_UNIT", &HandConstants::ANGLE_UNIT)
    .def_readonly_static("NUM_JOINTS", &HandConstants::NUM_JOINTS)
    .def_readonly_static("NUM_FORCE_SENSORS", &HandConstants::NUM_FORCE_SENSORS)
    .def_readonly_static("NUM_ANGLE_SENSORS", &HandConstants::NUM_ANGLE_SENSORS)
    .def_static("is_force_valid", &HandConstants::is_force_valid, py::arg("force_value"),
                "Check if force feedback value is valid / 检查力反馈值是否有效")
    .def_static("clamp_position", &HandConstants::clamp_position, py::arg("value"),
                "Clamp position value to valid range / 限制位置值在有效范围内")
    .def_static("clamp_speed", &HandConstants::clamp_speed, py::arg("value"),
                "Clamp speed value to valid range / 限制速度值在有效范围内")
    .def_static("clamp_force", &HandConstants::clamp_force, py::arg("value"),
                "Clamp force value to valid range / 限制力值在有效范围内")
    .def_static("angle_to_degrees", &HandConstants::angle_to_degrees, py::arg("raw_value"),
                "Convert raw angle value to degrees / 将原始角度值转换为度数");

  // ==================== FeedbackData ====================

  py::class_<FeedbackData>(m, "FeedbackData", "Feedback Data Structure / 反馈数据结构")
    .def(py::init<>())
    .def_readwrite("force_values", &FeedbackData::force_values)
    .def_readwrite("force_valid", &FeedbackData::force_valid)
    .def_readwrite("joint_angles", &FeedbackData::joint_angles)
    .def_readwrite("motor_positions", &FeedbackData::motor_positions);

  // ==================== HandState ====================

  py::class_<HandState>(m, "HandState", "Hand State Structure / 机械手状态结构")
    .def(py::init<>())
    .def_readwrite("hand_id", &HandState::hand_id)
    .def_readwrite("connected", &HandState::connected)
    .def_readwrite("positions", &HandState::positions)
    .def_readwrite("motor_positions", &HandState::motor_positions)
    .def_readwrite("speeds", &HandState::speeds)
    .def_readwrite("forces", &HandState::forces)
    .def_readwrite("joint_angles", &HandState::joint_angles)
    .def_readwrite("force_feedback", &HandState::force_feedback)
    .def_readwrite("force_feedback_valid", &HandState::force_feedback_valid)
    .def_readwrite("palm_forces", &HandState::palm_forces)
    .def_readwrite("palm_forces_valid", &HandState::palm_forces_valid);

  // ==================== ModbusError ====================

  py::register_exception<ModbusError>(m, "ModbusError");

  // ==================== LZHandModbusDriver ====================

  py::class_<LZHandModbusDriver>(m, "LZHandModbusDriver",
    "LZ Hand Modbus-RTU Driver / 灵巧手Modbus-RTU驱动类")
    .def(py::init<const std::string &, int, int, bool>(),
         py::arg("port"),
         py::arg("hand_id") = 1,
         py::arg("baudrate") = HandConstants::DEFAULT_BAUDRATE,
         py::arg("auto_connect") = true,
         "Constructor / 构造函数")

    // Connection
    .def("connect", &LZHandModbusDriver::connect,
         "Connect to the hand / 连接到机械手")
    .def("disconnect", &LZHandModbusDriver::disconnect,
         "Disconnect from the hand / 断开连接")
    .def("is_connected", &LZHandModbusDriver::is_connected,
         "Check connection status / 检查连接状态")

    // Position Control
    .def("set_joint_position", &LZHandModbusDriver::set_joint_position,
         py::arg("joint_index"), py::arg("position"),
         "Set single joint position / 设置单个关节位置")
    .def("set_all_positions", &LZHandModbusDriver::set_all_positions,
         py::arg("positions"),
         "Set all joint positions / 设置所有关节位置")

    // Speed Control
    .def("set_joint_speed", &LZHandModbusDriver::set_joint_speed,
         py::arg("joint_index"), py::arg("speed"), py::arg("gradual") = true,
         "Set single joint speed / 设置单个关节速度")
    .def("set_all_speeds", &LZHandModbusDriver::set_all_speeds,
         py::arg("speeds"), py::arg("gradual") = true,
         "Set all joint speeds / 设置所有关节速度")

    // Force Control
    .def("set_joint_force", &LZHandModbusDriver::set_joint_force,
         py::arg("joint_index"), py::arg("force"), py::arg("gradual") = true,
         "Set single joint force / 设置单个关节力")
    .def("set_all_forces", &LZHandModbusDriver::set_all_forces,
         py::arg("forces"), py::arg("gradual") = true,
         "Set all joint forces / 设置所有关节力")

    // Feedback Reading
    .def("read_motor_positions", &LZHandModbusDriver::read_motor_positions,
         "Read motor positions / 读取电机位置")
    .def("read_joint_angles", &LZHandModbusDriver::read_joint_angles,
         "Read joint angles / 读取关节角度")
    .def("read_force_feedback", &LZHandModbusDriver::read_force_feedback,
         "Read force feedback / 读取力反馈")
    .def("read_all_feedback", &LZHandModbusDriver::read_all_feedback,
         "Read all feedback data / 读取所有反馈数据")
    .def("read_control_state", &LZHandModbusDriver::read_control_state,
         "Read control state (positions, speeds, forces) / 读取控制状态")

    // High-Level Control
    .def("set_hand_pose", [](LZHandModbusDriver &self,
                             const std::array<int, 6> &positions,
                             const py::object &speeds,
                             const py::object &forces) {
           const std::array<int, 6> *speeds_ptr = nullptr;
           const std::array<int, 6> *forces_ptr = nullptr;
           std::array<int, 6> speeds_arr, forces_arr;

           if (!speeds.is_none()) {
             speeds_arr = speeds.cast<std::array<int, 6>>();
             speeds_ptr = &speeds_arr;
           }
           if (!forces.is_none()) {
             forces_arr = forces.cast<std::array<int, 6>>();
             forces_ptr = &forces_arr;
           }
           return self.set_hand_pose(positions, speeds_ptr, forces_ptr);
         },
         py::arg("positions"),
         py::arg("speeds") = py::none(),
         py::arg("forces") = py::none(),
         "Set complete hand pose / 设置完整手部姿态")

    .def("open_hand", &LZHandModbusDriver::open_hand,
         py::arg("speed") = 500,
         "Open the hand / 张开手")
    .def("close_hand", &LZHandModbusDriver::close_hand,
         py::arg("speed") = 500, py::arg("force") = 500,
         "Close the hand / 握拳")
    .def("pinch_grip", &LZHandModbusDriver::pinch_grip,
         py::arg("strength") = 500,
         "Pinch grip pose / 捏取姿态")
    .def("point_gesture", &LZHandModbusDriver::point_gesture,
         "Pointing gesture / 指向姿态")
    .def("ok_gesture", &LZHandModbusDriver::ok_gesture,
         "OK gesture / OK手势")
    .def("thumbs_up", &LZHandModbusDriver::thumbs_up,
         "Thumbs up gesture / 竖大拇指")

    // Configuration
    .def("set_gradual_step_size", &LZHandModbusDriver::set_gradual_step_size,
         py::arg("step_size"),
         "Set gradual adjustment step size / 设置渐进调节步长")
    .def("get_hand_id", &LZHandModbusDriver::get_hand_id,
         "Get hand ID / 获取机械手ID")
    
    // Debug
    .def("set_debug", &LZHandModbusDriver::set_debug,
         py::arg("enable"),
         "Enable/disable debug output / 启用/禁用调试输出")
    .def("set_raw_debug", &LZHandModbusDriver::set_raw_debug,
         py::arg("enable"),
         "Enable/disable libmodbus raw debug (shows all bytes) / 启用libmodbus原始调试");

  // Module version
  m.attr("__version__") = "1.0.0";
}
