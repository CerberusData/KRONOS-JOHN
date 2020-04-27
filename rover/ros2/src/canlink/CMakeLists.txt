cmake_minimum_required(VERSION 3.5)
project(canlink)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(usr_msgs REQUIRED)
find_package(utils REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_lib 
  lib/socket_can.cpp
)

add_executable(chassis_node 
  src/can_chassis.cpp
  src/modules/battery.cpp
  src/modules/chassis.cpp
  src/modules/lights.cpp
  lib/socket_can.cpp
)

add_executable(cabin_node 
  src/can_cabin.cpp
  src/modules/lock_system.cpp
  lib/socket_can.cpp
)

set(DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  usr_msgs
  utils
)

target_link_libraries(chassis_node utils::utils)
ament_target_dependencies(chassis_node ${DEPENDENCIES})

target_link_libraries(cabin_node utils::utils)
ament_target_dependencies(cabin_node ${DEPENDENCIES})

install(TARGETS
  chassis_node
  cabin_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()