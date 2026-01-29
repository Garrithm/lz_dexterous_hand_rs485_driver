cmake_minimum_required(VERSION 3.8)
project(lz_hand_rs485_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Find dependencies
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find libmodbus for C++ driver
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

# Find pybind11 for Python bindings
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# ============================================================
# Generate ROS2 messages
# ============================================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandControl.msg"
  "msg/JointControl.msg"
  "msg/HandFeedback.msg"
  "msg/JointFeedback.msg"
  "msg/ForceFeedback.msg"
  "msg/MotorFeedback.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# ============================================================
# C++ Library (底层Modbus驱动)
# ============================================================

# Build C++ modbus driver library
add_library(lz_hand_modbus_driver SHARED
  src/modbus_driver.cpp
)

target_include_directories(lz_hand_modbus_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MODBUS_INCLUDE_DIRS}
)

target_link_libraries(lz_hand_modbus_driver
  ${MODBUS_LIBRARIES}
)

# Install C++ headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install C++ library
install(
  TARGETS lz_hand_modbus_driver
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ============================================================
# Python Bindings (C++底层的Python绑定)
# ============================================================

# Build Python bindings using pybind11
pybind11_add_module(lz_hand_driver_cpp
  src/python_bindings.cpp
)

target_include_directories(lz_hand_driver_cpp PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MODBUS_INCLUDE_DIRS}
)

target_link_libraries(lz_hand_driver_cpp PRIVATE
  lz_hand_modbus_driver
  ${MODBUS_LIBRARIES}
)

# Set RPATH so Python module can find liblz_hand_modbus_driver.so at runtime
# Python module is in lib/${PROJECT_NAME}, dependency is in lib/
set_target_properties(lz_hand_driver_cpp PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Install Python bindings module to the package directory
# Install to lib/${PROJECT_NAME} so it can be found by ROS2 Python scripts
install(TARGETS lz_hand_driver_cpp
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Python Scripts (ROS2节点)
# ============================================================

# ROS2节点脚本
install(PROGRAMS
  scripts/hand_driver_node.py
  scripts/hand_test_node.py
  scripts/detect_hand_ports.py
  DESTINATION lib/${PROJECT_NAME}
)

# 工具脚本（Tools）
install(PROGRAMS
  tool/detect_hand_ports.sh
  DESTINATION lib/${PROJECT_NAME}
)

# 独立测试脚本（不依赖ROS，用于硬件调试）
install(PROGRAMS
  demo/simple_driver_test.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Launch and Config files
# ============================================================

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)


# ============================================================
# Export and Testing
# ============================================================

# Export C++ library for other packages
ament_export_include_directories(include)
ament_export_libraries(lz_hand_modbus_driver)
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
