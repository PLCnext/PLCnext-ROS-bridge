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

set(TOOLCHAIN "/opt/pxc/sdks/AXCF3152/2022.0")
set(ARPVERSION "2022.0.5")
set(TARGETNAME "AXCF3152")

set(CMAKE_TOOLCHAIN_FILE ${TOOLCHAIN}/toolchain.cmake CACHE STRING "PLCnext toolchain file")
set(ARP_TOOLCHAIN_ROOT ${TOOLCHAIN} CACHE STRING "PLCnext toolchain root directory")
set(ARP_DEVICE ${TARGETNAME} CACHE STRING "PLCnext target device name")

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
find_package(gRPC CONFIG REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SetIO.msg"
  "srv/BatchGetIO.srv"
  "srv/BatchSetIO.srv"
  "srv/SingleGetIO.srv"
  "srv/SingleSetIO.srv"
  "srv/AnalogIO.srv"
)

# C++ stuff
include_directories(include)

set(WILDCARD_SOURCE *.cpp *.cc)
set(WILDCARD_HEADER *.h *.hpp *.hxx)
file(GLOB_RECURSE Sources
    include/${PROJECT_NAME}/${WILDCARD_SOURCE}
)
file(GLOB_RECURSE Headers
    include/${PROJECT_NAME}/${WILDCARD_HEADER}
)

add_executable(phoenix_bridge_node src/phoenix_bridge_node.cpp
                                    ${Headers}
                                    ${Sources})
target_link_libraries(phoenix_bridge_node  gRPC::gpr gRPC::grpc gRPC::grpc++)
add_dependencies(phoenix_bridge_node cog)
ament_target_dependencies(phoenix_bridge_node
                            rclcpp
                            std_msgs
                            nav_msgs
                            geometry_msgs)

add_executable(phoenix_io_services_node src/phoenix_io_services_node.cpp
                                        src/phoenix_io_services.cpp
                                        ${Headers}
                                        ${Sources})
add_dependencies(phoenix_io_services_node ${PROJECT_NAME})
target_link_libraries(phoenix_io_services_node  gRPC::gpr gRPC::grpc gRPC::grpc++)
rosidl_target_interfaces(phoenix_io_services_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(phoenix_io_services_node
                            rclcpp)

if(rclcpp_FOUND)
  target_include_directories(phoenix_bridge_node
      PUBLIC
      $<BUILD_INTERFACE:${rclcpp_INCLUDE_DIRS}>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs/Includes>
  )
  target_include_directories(phoenix_io_services_node
      PUBLIC
      $<BUILD_INTERFACE:${rclcpp_INCLUDE_DIRS}>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs/Includes>
  )
endif()

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
  # Linting
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(files_to_lint
    include/${PROJECT_NAME}/bridge_type.hpp
    include/${PROJECT_NAME}/phoenix_comm.h
    include/${PROJECT_NAME}/phoenix_io_services.hpp
    src/phoenix_bridge_node.cpp
    src/phoenix_io_services_node.cpp
    src/phoenix_io_services.cpp
  )

  ament_clang_format(${files_to_lint})
  ament_cpplint(${files_to_lint} FILTERS "-legal/copyright,-build/include_order")
  ament_cppcheck(${files_to_lint})
  ament_lint_cmake()
  ament_xmllint()

  # Unit testing
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_conversions test/test_conversions.cpp
                                    ${Headers}
                                    ${Sources})
  target_link_libraries(test_conversions  gRPC::gpr gRPC::grpc gRPC::grpc++)
  target_include_directories(test_conversions
      PUBLIC
      $<BUILD_INTERFACE:${rclcpp_INCLUDE_DIRS}>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs/Includes>
  )
  ament_target_dependencies(test_conversions
                            rclcpp
                            std_msgs
                            nav_msgs
                            geometry_msgs)

  # Integration testing
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/launch_test_bridge.test.py)

endif()

ament_package()
