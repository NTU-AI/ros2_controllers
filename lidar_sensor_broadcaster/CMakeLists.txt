cmake_minimum_required(VERSION 3.16)
project(lidar_sensor_broadcaster LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  std_msgs
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
)
find_package(generate_parameter_library REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(lidar_sensor_broadcaster_parameters
  src/lidar_sensor_broadcaster_parameters.yaml
)

add_library(lidar_sensor_broadcaster SHARED
  src/lidar_sensor_broadcaster.cpp
)
target_compile_features(lidar_sensor_broadcaster PUBLIC cxx_std_17)
target_include_directories(lidar_sensor_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/lidar_sensor_broadcaster>
)
target_link_libraries(lidar_sensor_broadcaster PUBLIC
  lidar_sensor_broadcaster_parameters
)
ament_target_dependencies(lidar_sensor_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(lidar_sensor_broadcaster PRIVATE "LIDAR_SENSOR_BROADCASTER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface lidar_sensor_broadcaster.xml)



install(
  DIRECTORY include/
  DESTINATION include/lidar_sensor_broadcaster
)
install(
  TARGETS
    lidar_sensor_broadcaster
    lidar_sensor_broadcaster_parameters
  EXPORT export_lidar_sensor_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_lidar_sensor_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
