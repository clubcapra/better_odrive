cmake_minimum_required(VERSION 3.8)
project(odrive_controller_interface)

find_package(ros2_control_cmake REQUIRED)

set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  builtin_interfaces
  rclcpp
  rclcpp_lifecycle
  controller_interface
  control_toolbox
  realtime_tools

  std_msgs
  std_srvs
  odrive_types

  generate_parameter_library
  pluginlib
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# generate parameters
generate_parameter_library(odrive_controller_interface_parameters
  src/odrive_controller_interface_parameters.yaml
)

# add library and configure
add_library(odrive_controller_interface SHARED
  src/odrive_controller_interface.cpp
)
target_compile_features(odrive_controller_interface PUBLIC cxx_std_17)
target_include_directories(odrive_controller_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/odrive_controller_interface>
)
target_link_libraries(odrive_controller_interface PUBLIC
  odrive_controller_interface_parameters
  controller_interface::controller_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  control_toolbox::control_toolbox
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${odrive_types_TARGETS}
  ${builtin_interfaces_TARGETS}
)
pluginlib_export_plugin_description_file(controller_interface odrive_controller_interface.xml)

# install directories
install(
  TARGETS 
    odrive_controller_interface
    odrive_controller_interface_parameters
  EXPORT export_odrive_controller_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# export library
ament_export_targets(export_odrive_controller_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
