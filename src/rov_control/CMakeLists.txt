cmake_minimum_required(VERSION 3.8)
project(rov_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)

include_directories(
  include
  ${hardware_interface_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/thruster_interface.cpp
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

pluginlib_export_plugin_description_file(hardware_interface plugins/thruster_hardware_plugin.xml)


install(
  TARGETS rov_control
  EXPORT export_rov_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  rov_control
)
ament_export_targets(
  export_rov_control
)



ament_package()