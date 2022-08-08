cmake_minimum_required(VERSION 3.8)
project(phoenix_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SetIO.msg"
  "srv/BatchGetIO.srv"
  "srv/BatchSetIO.srv"
  "srv/SingleGetIO.srv"
  "srv/SingleSetIO.srv"
  "srv/AnalogIO.srv"
)

install(DIRECTORY
  msg
  srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
