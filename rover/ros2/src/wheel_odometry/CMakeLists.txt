cmake_minimum_required(VERSION 3.5)
project(wheel_odometry)

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

include_directories(include)

set(DEPENDENCIES
  rclcpp
  std_msgs
)

add_executable(wheel_odometry
  src/main.cpp
)

ament_target_dependencies(wheel_odometry ${DEPENDENCIES})

install(TARGETS
  wheel_odometry
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
