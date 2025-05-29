cmake_minimum_required(VERSION 3.14)
project(rov_control)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)

# ========= OSQP VENDOR SETUP =========
# Build OSQP from vendor source
add_subdirectory(external/osqp)

add_library(${PROJECT_NAME} SHARED
  src/pwm_interface.cpp
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
)

# ========= BUILD AXIS-TO-COMMAND CONTROLLER =========
add_library(axis_to_command_controller SHARED
  src/axis_to_command_controller.cpp
)

target_include_directories(axis_to_command_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/osqp/include
)

target_link_libraries(axis_to_command_controller
  osqpstatic
)

ament_target_dependencies(axis_to_command_controller
  rclcpp
  controller_interface
  hardware_interface
)

pluginlib_export_plugin_description_file(
  controller_interface
  plugins/axis_to_command_controller_plugin.xml
)

# ========= INSTALL =========
install(
  TARGETS rov_control axis_to_command_controller
  EXPORT export_rov_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_libraries(
  rov_control
  axis_to_command_controller
)
ament_export_targets(
  export_rov_control
)
ament_package()
