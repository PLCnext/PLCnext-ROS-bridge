cmake_minimum_required(VERSION 3.0.2)
project(phoenix_bridge)

add_compile_options(-std=c++11)

add_custom_target(
 cog ALL
 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
 COMMAND echo "Generating bridge types based on config file"
 COMMAND cog -r ${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/include_types.h
 COMMAND cog -r ${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/phoenix_bridge.h
 COMMAND cog -r ${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/write_conversions.h
 COMMAND cog -r ${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/read_conversions.h
 COMMAND cog -r ${CMAKE_CURRENT_SOURCE_DIR}/src/phoenix_bridge.cpp
 COMMENT "COGGING"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  roslint
  rostest
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
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nav_msgs
    roscpp
    roslint
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(WILDCARD_SOURCE *.cpp *.cc)
set(WILDCARD_HEADER *.h *.h *.hxx)
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

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY src/parsers/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src/parsers/)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS phoenix_io_services
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(write_conversion
        tests/unit_tests/write_conversion.test
        tests/unit_tests/write_conversion.cpp
        ${Headers} ${Sources})
    add_dependencies(write_conversion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(write_conversion gRPC::gpr gRPC::grpc gRPC::grpc++ ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    target_include_directories(write_conversion
        PUBLIC
        $<BUILD_INTERFACE:${rclcpp_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/ServiceStubs>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/ServiceStubs/Includes>
    )

    add_rostest_gtest(read_conversion
        tests/unit_tests/read_conversion.test
        tests/unit_tests/read_conversion.cpp
        ${Headers} ${Sources})
    add_dependencies(read_conversion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(read_conversion gRPC::gpr gRPC::grpc gRPC::grpc++ ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    target_include_directories(read_conversion
        PUBLIC
        $<BUILD_INTERFACE:${rclcpp_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/ServiceStubs>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/phoenix_bridge/ServiceStubs/Includes>
    )

endif()

set(files_to_lint
    include/${PROJECT_NAME}/bridge_type.h
    include/${PROJECT_NAME}/params_struct.h
    include/${PROJECT_NAME}/phoenix_bridge.h
    include/${PROJECT_NAME}/phoenix_comm.h
    include/${PROJECT_NAME}/phoenix_io_services.h
    src/phoenix_bridge_node.cpp
    src/phoenix_bridge.cpp
    src/phoenix_io_services_node.cpp
    src/phoenix_io_services.cpp
  )
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp(${files_to_lint})
