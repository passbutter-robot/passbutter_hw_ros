cmake_minimum_required(VERSION 3.5)
project(passbutter_driver_demo_robot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

## INSTALL
install(
  DIRECTORY controllers description launch/
  DESTINATION share/${PROJECT_NAME}
)

## EXPORTS
ament_package()
