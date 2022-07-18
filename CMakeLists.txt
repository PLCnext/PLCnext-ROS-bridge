cmake_minimum_required(VERSION 3.0.2)
project(phoenix_bridge)

add_compile_options(-std=c++11)

add_custom_target(
 cog ALL
 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
 COMMAND echo "Generating bridge types based on config file"
 COMMAND cog -r ${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/include_types.h
 COMMAND cog -r ${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/phoenix_bridge.hpp
 COMMAND cog -r ${CMAKE_CURRENT_SOURCE_DIR}/src/phoenix_bridge.cpp
 COMMENT "COGGING"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)
find_package(gRPC CONFIG REQUIRED)

add_message_files(
  FILES
  SetIO.msg
)

add_service_files(
  FILES
  BatchGetIO.srv
  BatchSetIO.srv
  SingleGetIO.srv
  SingleSetIO.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(WILDCARD_SOURCE *.cpp *.cc)
set(WILDCARD_HEADER *.h *.hpp *.hxx)
file(GLOB_RECURSE Sources
    include/${PROJECT_NAME}/${WILDCARD_SOURCE}
)
file(GLOB_RECURSE Headers
    include/${PROJECT_NAME}/${WILDCARD_HEADER}
)

add_executable(${PROJECT_NAME}_node
                    src/phoenix_bridge_node.cpp
                    src/phoenix_bridge.cpp
                    ${Headers} ${Sources})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} cog)
target_link_libraries(${PROJECT_NAME}_node
  gRPC::gpr gRPC::grpc gRPC::grpc++
  ${catkin_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    $<BUILD_INTERFACE:${rclcpp_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/ServiceStubs>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/ServiceStubs/Includes>
)

add_executable(phoenix_io_services
                    src/phoenix_io_services_node.cpp
                    src/phoenix_io_services.cpp
                    ${Headers} ${Sources})
add_dependencies(phoenix_io_services ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} cog)
target_link_libraries(phoenix_io_services
  gRPC::gpr gRPC::grpc gRPC::grpc++
  ${catkin_LIBRARIES}
)
target_include_directories(phoenix_io_services
    PUBLIC
    $<BUILD_INTERFACE:${rclcpp_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/ServiceStubs>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/ServiceStubs/Includes>
)

set(COG_INCLUDE "cog -r ")
