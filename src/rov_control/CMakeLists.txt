cmake_minimum_required(VERSION 3.14)
project(rov_control)

# ========= DEPENDENCIES =========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ========= EXTERNAL DEPENDENCIES =========
# Build OSQP from vendor source
add_subdirectory(external/osqp)

# ========= SHARED LIBRARY: THRUSTER HARDWARE INTERFACE =========
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

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  controller_interface
  sensor_msgs
  Eigen3
)

# ========= EXECUTABLE: GAMEPAD PARSER NODE =========
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

# ========= SHARED LIBRARY: AXIS-TO-COMMAND CONTROLLER =========
add_library(axis_to_command_controller SHARED
  src/axis_to_command_controller.cpp
)

target_include_directories(axis_to_command_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
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
  pluginlib
  Eigen3
)

# ========= SHARED LIBRARY: THRUSTER PID CONTROLLER =========
add_library(thruster_pid_controller SHARED
  src/thruster_pid_controller.cpp
)

target_include_directories(thruster_pid_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(thruster_pid_controller
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  control_toolbox
  geometry_msgs
)

# ========= PLUGIN EXPORTS =========
pluginlib_export_plugin_description_file(hardware_interface plugins/pwm_interface_plugin.xml)
pluginlib_export_plugin_description_file(controller_interface plugins/axis_to_command_controller_plugin.xml)
pluginlib_export_plugin_description_file(controller_interface plugins/thruster_pid_controller_plugin.xml)

# ========= INSTALL TARGETS =========
install(
  TARGETS 
    ${PROJECT_NAME}
    axis_to_command_controller
    thruster_pid_controller
    gamepad_parser_node
  EXPORT export_rov_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ========= INSTALL DIRECTORIES =========
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ========= INSTALL PYTHON SCRIPTS =========
install(PROGRAMS
  src/bar100_sensor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bar100_sensor
)

# ========= AMENT EXPORTS =========
ament_export_libraries(
  ${PROJECT_NAME}
  axis_to_command_controller
  thruster_pid_controller
)

ament_export_targets(
  export_rov_control
)

ament_package()
