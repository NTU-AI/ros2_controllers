cmake_minimum_required(VERSION 3.5)
project(state_machine_command_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

add_library(state_machine_command_controller
  SHARED
  src/state_machine_command_controller.cpp
)
target_include_directories(state_machine_command_controller PRIVATE include)
ament_target_dependencies(state_machine_command_controller
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  std_msgs
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(state_machine_command_controller PRIVATE "STATE_MACHINE_COMMAND_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(state_machine_command_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface state_machine_command_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  state_machine_command_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gmock REQUIRED)
#   find_package(controller_manager REQUIRED)
#   find_package(hardware_interface REQUIRED)
#   find_package(ros2_control_test_assets REQUIRED)

#   ament_add_gmock(
#     test_load_state_machine_command_controller
#     test/test_load_state_machine_command_controller.cpp
#   )
#   target_include_directories(test_load_state_machine_command_controller PRIVATE include)
#   ament_target_dependencies(test_load_state_machine_command_controller
#     controller_manager
#     hardware_interface
#     ros2_control_test_assets
#   )
#   # prevent pluginlib from using boost
#   target_compile_definitions(test_load_state_machine_command_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

#   ament_add_gmock(
#     test_state_machine_command_controller
#     test/test_state_machine_command_controller.cpp
#   )
#   target_include_directories(test_state_machine_command_controller PRIVATE include)
#   target_link_libraries(test_state_machine_command_controller
#     state_machine_command_controller
#   )
# endif()

ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  state_machine_command_controller
)
ament_package()
