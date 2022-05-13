cmake_minimum_required(VERSION 3.5)
project(phoenix_bridge)

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

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# C++ stuff
include_directories(include)

add_executable(phoenix_bridge_node src/phoenix_bridge_node.cpp
                              src/phoenix_bridge.cpp)
ament_target_dependencies(phoenix_bridge_node rclcpp)

install(TARGETS
  phoenix_bridge_node
  DESTINATION lib/${PROJECT_NAME})

# Python stuff
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
