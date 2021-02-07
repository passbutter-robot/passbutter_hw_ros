cmake_minimum_required(VERSION 3.5)
project(passbutter_controller)

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

add_library(passbutter_controller
  SHARED
  src/passbutter_controller.cpp
)
target_include_directories(passbutter_controller PRIVATE include)
ament_target_dependencies(passbutter_controller
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
target_compile_definitions(passbutter_controller PRIVATE "PASSBUTTER_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(passbutter_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface passbutter_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  passbutter_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

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
  passbutter_controller
)
ament_package()
