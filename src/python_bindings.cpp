/**
 * @file python_bindings.cpp
 * @brief 灵巧手驱动Python绑定（LZ Hand Driver Python Bindings using pybind11）
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

#include "lz_hand_rs485_driver/modbus_driver.hpp"
#include "lz_hand_rs485_driver/hand_constants.hpp"

namespace py = pybind11;
using namespace lz_hand;

PYBIND11_MODULE(lz_hand_driver_cpp, m) {
  m.doc() = "灵巧手Modbus驱动C++绑定（LZ Hand Modbus Driver C++ Bindings）";

  // ==================== 枚举（Enums） ====================

  py::enum_<HandID>(m, "HandID", "手ID（Hand ID）")
    .value("RIGHT_HAND", HandID::RIGHT_HAND, "右手（Right hand）")
    .value("LEFT_HAND", HandID::LEFT_HAND, "左手（Left hand）")
    .export_values();

  py::enum_<JointIndex>(m, "JointIndex", "关节索引（Joint Index）")
    .value("THUMB_ROTATION", JointIndex::THUMB_ROTATION, "大拇指翻转（Thumb rotation）")
    .value("THUMB_BEND", JointIndex::THUMB_BEND, "大拇指弯曲（Thumb bend）")
    .value("INDEX_BEND", JointIndex::INDEX_BEND, "食指弯曲（Index bend）")
    .value("MIDDLE_BEND", JointIndex::MIDDLE_BEND, "中指弯曲（Middle bend）")
    .value("RING_BEND", JointIndex::RING_BEND, "无名指弯曲（Ring bend）")
    .value("PINKY_BEND", JointIndex::PINKY_BEND, "小拇指弯曲（Pinky bend）")
    .export_values();

  // ==================== 寄存器映射（Register Map） ====================

  py::class_<RegisterMap>(m, "RegisterMap", "寄存器映射（Register Map）")
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

  // ==================== 常量（Constants） ====================

  py::class_<HandConstants>(m, "HandConstants", "机械手常量（Hand Constants）")
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
                "检查力反馈值是否有效（Check if force value is valid）")
    .def_static("clamp_position", &HandConstants::clamp_position, py::arg("value"),
                "限制位置值范围（Clamp position value）")
    .def_static("clamp_speed", &HandConstants::clamp_speed, py::arg("value"),
                "限制速度值范围（Clamp speed value）")
    .def_static("clamp_force", &HandConstants::clamp_force, py::arg("value"),
                "限制力值范围（Clamp force value）")
    .def_static("angle_to_degrees", &HandConstants::angle_to_degrees, py::arg("raw_value"),
                "转换角度值为度（Convert angle to degrees）");

  // ==================== 数据结构（Data Structures） ====================

  py::class_<FeedbackData>(m, "FeedbackData", "反馈数据结构（Feedback Data Structure）")
    .def(py::init<>())
    .def_readwrite("force_values", &FeedbackData::force_values)
    .def_readwrite("force_valid", &FeedbackData::force_valid)
    .def_readwrite("joint_angles", &FeedbackData::joint_angles)
    .def_readwrite("motor_positions", &FeedbackData::motor_positions);

  py::class_<HandState>(m, "HandState", "机械手状态结构（Hand State Structure）")
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

  // ==================== 异常（Exception） ====================

  py::register_exception<ModbusError>(m, "ModbusError");

  // ==================== 驱动类（Driver Class） ====================

  py::class_<LZHandModbusDriver>(m, "LZHandModbusDriver",
    "灵巧手Modbus-RTU驱动类（LZ Hand Modbus-RTU Driver Class）")
    .def(py::init<const std::string &, int, int, bool>(),
         py::arg("port"),
         py::arg("hand_id") = 1,
         py::arg("baudrate") = HandConstants::DEFAULT_BAUDRATE,
         py::arg("auto_connect") = true,
         "构造函数（Constructor）")

    // 连接（Connection）
    .def("connect", &LZHandModbusDriver::connect, "连接（Connect）")
    .def("disconnect", &LZHandModbusDriver::disconnect, "断开连接（Disconnect）")
    .def("is_connected", &LZHandModbusDriver::is_connected, "检查连接状态（Check connection）")

    // 位置控制（Position control）
    .def("set_joint_position", &LZHandModbusDriver::set_joint_position,
         py::arg("joint_index"), py::arg("position"),
         "设置单关节位置（Set single joint position）")
    .def("set_all_positions", &LZHandModbusDriver::set_all_positions,
         py::arg("positions"),
         "设置所有关节位置（Set all joint positions）")

    // 速度控制（Speed control）
    .def("set_joint_speed", &LZHandModbusDriver::set_joint_speed,
         py::arg("joint_index"), py::arg("speed"), py::arg("gradual") = true,
         "设置单关节速度（Set single joint speed）")
    .def("set_all_speeds", &LZHandModbusDriver::set_all_speeds,
         py::arg("speeds"), py::arg("gradual") = true,
         "设置所有关节速度（Set all joint speeds）")

    // 力控制（Force control）
    .def("set_joint_force", &LZHandModbusDriver::set_joint_force,
         py::arg("joint_index"), py::arg("force"), py::arg("gradual") = true,
         "设置单关节力（Set single joint force）")
    .def("set_all_forces", &LZHandModbusDriver::set_all_forces,
         py::arg("forces"), py::arg("gradual") = true,
         "设置所有关节力（Set all joint forces）")

    // 反馈读取（Feedback reading）
    .def("read_motor_positions", &LZHandModbusDriver::read_motor_positions,
         "读取电机位置（Read motor positions）")
    .def("read_joint_angles", &LZHandModbusDriver::read_joint_angles,
         "读取关节角度（Read joint angles）")
    .def("read_force_feedback", &LZHandModbusDriver::read_force_feedback,
         "读取力反馈（Read force feedback）")
    .def("read_all_feedback", &LZHandModbusDriver::read_all_feedback,
         "读取所有反馈数据（Read all feedback）")
    .def("read_control_state", &LZHandModbusDriver::read_control_state,
         "读取控制状态（Read control state）")

    // 高级控制（High-level control）
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
         "设置完整手部姿态（Set complete hand pose）")

    .def("open_hand", &LZHandModbusDriver::open_hand, py::arg("speed") = 500, "张开手（Open hand）")
    .def("close_hand", &LZHandModbusDriver::close_hand, py::arg("speed") = 500, py::arg("force") = 500, "握拳（Close hand）")
    .def("pinch_grip", &LZHandModbusDriver::pinch_grip, py::arg("strength") = 500, "捏取姿态（Pinch grip）")
    .def("point_gesture", &LZHandModbusDriver::point_gesture, "指向姿态（Point gesture）")
    .def("ok_gesture", &LZHandModbusDriver::ok_gesture, "OK手势（OK gesture）")
    .def("thumbs_up", &LZHandModbusDriver::thumbs_up, "竖大拇指（Thumbs up）")

    // 配置（Configuration）
    .def("set_gradual_step_size", &LZHandModbusDriver::set_gradual_step_size, py::arg("step_size"), "设置渐进步长（Set gradual step size）")
    .def("get_hand_id", &LZHandModbusDriver::get_hand_id, "获取手ID（Get hand ID）")
    .def("set_debug", &LZHandModbusDriver::set_debug, py::arg("enable"), "启用调试（Enable debug）")
    .def("set_raw_debug", &LZHandModbusDriver::set_raw_debug, py::arg("enable"), "启用原始调试（Enable raw debug）");

  m.attr("__version__") = "1.0.0";
}
