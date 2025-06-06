#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rov_control::thruster_pid_controller" for configuration ""
set_property(TARGET rov_control::thruster_pid_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rov_control::thruster_pid_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libthruster_pid_controller.so"
  IMPORTED_SONAME_NOCONFIG "libthruster_pid_controller.so"
  )

list(APPEND _cmake_import_check_targets rov_control::thruster_pid_controller )
list(APPEND _cmake_import_check_files_for_rov_control::thruster_pid_controller "${_IMPORT_PREFIX}/lib/libthruster_pid_controller.so" )

# Import target "rov_control::rov_control" for configuration ""
set_property(TARGET rov_control::rov_control APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rov_control::rov_control PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librov_control.so"
  IMPORTED_SONAME_NOCONFIG "librov_control.so"
  )

list(APPEND _cmake_import_check_targets rov_control::rov_control )
list(APPEND _cmake_import_check_files_for_rov_control::rov_control "${_IMPORT_PREFIX}/lib/librov_control.so" )

# Import target "rov_control::axis_to_command_controller" for configuration ""
set_property(TARGET rov_control::axis_to_command_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rov_control::axis_to_command_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libaxis_to_command_controller.so"
  IMPORTED_SONAME_NOCONFIG "libaxis_to_command_controller.so"
  )

list(APPEND _cmake_import_check_targets rov_control::axis_to_command_controller )
list(APPEND _cmake_import_check_files_for_rov_control::axis_to_command_controller "${_IMPORT_PREFIX}/lib/libaxis_to_command_controller.so" )

# Import target "rov_control::gamepad_parser_node" for configuration ""
set_property(TARGET rov_control::gamepad_parser_node APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rov_control::gamepad_parser_node PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/rov_control/gamepad_parser_node"
  )

list(APPEND _cmake_import_check_targets rov_control::gamepad_parser_node )
list(APPEND _cmake_import_check_files_for_rov_control::gamepad_parser_node "${_IMPORT_PREFIX}/lib/rov_control/gamepad_parser_node" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
