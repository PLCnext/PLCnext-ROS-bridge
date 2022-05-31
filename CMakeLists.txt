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

add_custom_target(
 cog ALL
 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
 COMMAND echo "Generating bridge types based on config file"
 COMMAND cog -r ${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/include_types.h
 COMMAND cog -r ${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/conversions.hpp
 COMMAND cog -r ${CMAKE_CURRENT_SOURCE_DIR}/src/phoenix_bridge_node.cpp
 COMMENT "COGGING"
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SetIO.msg"
  "srv/BatchGetIO.srv"
  "srv/BatchSetIO.srv"
  "srv/SingleGetIO.srv"
  "srv/SingleSetIO.srv"
)

# C++ stuff
include_directories(include)

add_executable(phoenix_bridge_node src/phoenix_bridge_node.cpp)
add_dependencies(phoenix_bridge_node cog)
ament_target_dependencies(phoenix_bridge_node
                            rclcpp
                            std_msgs
                            nav_msgs
                            geometry_msgs)

add_executable(phoenix_io_services_node src/phoenix_io_services_node.cpp
                                        src/phoenix_io_services.cpp)
add_dependencies(phoenix_io_services_node ${PROJECT_NAME})
rosidl_target_interfaces(phoenix_io_services_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(phoenix_io_services_node
                            rclcpp)

install(TARGETS
  phoenix_bridge_node
  phoenix_io_services_node
  DESTINATION lib/${PROJECT_NAME})

# Python stuff
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY 
  msg
  srv
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
