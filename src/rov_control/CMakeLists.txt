cmake_minimum_required(VERSION 3.14)
project(rov_control)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Eigen3 REQUIRED)

# ========= OSQP VENDOR SETUP =========
# Build OSQP from vendor source
add_subdirectory(external/osqp)

# ========= BUILD THRUSTER HARDWARE INTERFACE =========
add_library(${PROJECT_NAME} SHARED
  src/pwm_interface.cpp
  src/libdriver_pca9685/interface/driver_pca9685_interface.c
  src/libdriver_pca9685/src/driver_pca9685.c
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(gamepad_parser_node src/gamepad_parser_node.cpp)
target_include_directories(gamepad_parser_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(gamepad_parser_node
  rclcpp
  sensor_msgs
  geometry_msgs
)


ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  controller_interface
  sensor_msgs
  Eigen3
)

# ========= BUILD AXIS-TO-COMMAND CONTROLLER =========
add_library(axis_to_command_controller SHARED
  src/axis_to_command_controller.cpp
)

target_include_directories(axis_to_command_controller PRIVATE
  include
  external/osqp/include
  external/osqp/algebra/common
  external/osqp/build/include
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(axis_to_command_controller
  osqpstatic
)

ament_target_dependencies(axis_to_command_controller
  rclcpp
  controller_interface
  hardware_interface
  Eigen3
)

pluginlib_export_plugin_description_file(hardware_interface plugins/pwm_interface_plugin.xml)
pluginlib_export_plugin_description_file(controller_interface plugins/axis_to_command_controller_plugin.xml)

# ========= INSTALL =========
install(
  TARGETS rov_control axis_to_command_controller gamepad_parser_node
  EXPORT export_rov_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_libraries(
  rov_control
  axis_to_command_controller
)
ament_export_targets(
  export_rov_control
)

ament_package()
