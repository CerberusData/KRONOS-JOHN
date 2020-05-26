cmake_minimum_required(VERSION 3.5)
project(can_test)

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
find_package(std_srvs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} 
  lib/socket_can.cpp
) 

set(DEPENDENCIES
  rclcpp
  std_msgs
  std_srvs
)

add_executable(test_node
  src/main.cpp
  src/can_test.cpp
  src/modules/chassis.cpp
  lib/socket_can.cpp
)

ament_target_dependencies(test_node ${DEPENDENCIES})

install(TARGETS
  test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
