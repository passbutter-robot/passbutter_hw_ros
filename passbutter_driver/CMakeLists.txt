cmake_minimum_required(VERSION 3.5)
project(passbutter_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(stepper_driver 
  src/stepper_driver_node.cpp
  src/ThunderBorg/src/ThunderBorg.cpp)
ament_target_dependencies(stepper_driver rclcpp ros2_control example_interfaces)

add_executable(stepper_command 
  src/stepper_command_node.cpp)
ament_target_dependencies(stepper_command rclcpp ros2_control example_interfaces)

add_executable(wheel_driver 
  src/wheel_driver_node.cpp
  src/ThunderBorg/src/ThunderBorg.cpp)
ament_target_dependencies(wheel_driver rclcpp ros2_control example_interfaces)

add_executable(wheel_command 
  src/wheel_command_node.cpp)
ament_target_dependencies(wheel_command rclcpp ros2_control example_interfaces)

install(TARGETS
  stepper_driver
  stepper_command
  wheel_driver
  wheel_command
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()