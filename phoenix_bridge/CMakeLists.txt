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
  COMMAND cog -r ${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/read_conversions.hpp
  COMMAND cog -r ${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/write_conversions.hpp
  COMMAND cog -r ${CMAKE_CURRENT_SOURCE_DIR}/src/phoenix_bridge_node.cpp
  COMMENT "COGGING"
)

list(APPEND MSG_TYPES_TO_LINK
    geometry_msgs
    nav_msgs
    std_msgs
)

# Find all required messages
foreach(msg ${MSG_TYPES_TO_LINK})
  find_package(${msg} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(phoenix_interfaces REQUIRED)

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
ament_target_dependencies(phoenix_bridge_node rclcpp ${MSG_TYPES_TO_LINK})
target_include_directories(phoenix_bridge_node
    PUBLIC
    $<BUILD_INTERFACE:${rclcpp_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs/Includes>
)

add_executable(phoenix_io_services_node src/phoenix_io_services_node.cpp
                                        src/phoenix_io_services.cpp
                                        ${Headers}
                                        ${Sources})
target_link_libraries(phoenix_io_services_node  gRPC::gpr gRPC::grpc gRPC::grpc++)
ament_target_dependencies(phoenix_io_services_node rclcpp phoenix_interfaces ${MSG_TYPES_TO_LINK})
target_include_directories(phoenix_io_services_node
    PUBLIC
    $<BUILD_INTERFACE:${rclcpp_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs/Includes>
)

add_executable(liveliness_check src/liveliness_check.cpp
                                        ${Headers}
                                        ${Sources})
target_link_libraries(liveliness_check  gRPC::gpr gRPC::grpc gRPC::grpc++)
ament_target_dependencies(liveliness_check rclcpp ${MSG_TYPES_TO_LINK})
target_include_directories(liveliness_check
    PUBLIC
    $<BUILD_INTERFACE:${rclcpp_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs/Includes>
)

install(TARGETS
  phoenix_bridge_node
  phoenix_io_services_node
  liveliness_check
  DESTINATION lib/${PROJECT_NAME})

# Python stuff
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Linting
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(files_to_lint
    include/${PROJECT_NAME}/bridge_type.hpp
    include/${PROJECT_NAME}/phoenix_comm.hpp
    include/${PROJECT_NAME}/phoenix_io_services.hpp
    src/phoenix_bridge_node.cpp
    src/phoenix_io_services_node.cpp
    src/phoenix_io_services.cpp
    src/liveliness_check.cpp
  )

  ament_clang_format(${files_to_lint})
  ament_cpplint(${files_to_lint} FILTERS "-build/include_order")
  ament_cppcheck(${files_to_lint})
  ament_xmllint()

  # Unit testing
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_write_conversions test/test_write_conversions.cpp ${Headers} ${Sources})
  target_link_libraries(test_write_conversions  gRPC::gpr gRPC::grpc gRPC::grpc++)
  target_include_directories(test_write_conversions PUBLIC
                              $<BUILD_INTERFACE:${rclcpp_INCLUDE_DIRS}>
                              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs>
                              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs/Includes>)
  ament_target_dependencies(test_write_conversions rclcpp ${MSG_TYPES_TO_LINK})

  ament_add_gtest(test_read_conversions test/test_read_conversions.cpp ${Headers} ${Sources})
  target_link_libraries(test_read_conversions  gRPC::gpr gRPC::grpc gRPC::grpc++)
  target_include_directories(test_read_conversions PUBLIC
                              $<BUILD_INTERFACE:${rclcpp_INCLUDE_DIRS}>
                              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs>
                              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ServiceStubs/Includes>)
  ament_target_dependencies(test_read_conversions rclcpp ${MSG_TYPES_TO_LINK})

  # Component testing
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/launch_test_bridge.test.py)

endif()

ament_package()
