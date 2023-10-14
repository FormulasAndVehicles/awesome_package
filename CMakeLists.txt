cmake_minimum_required(VERSION 3.5)

project(awesome_package)

find_package(ament_cmake REQUIRED)

find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)


install(PROGRAMS
  nodes/setpoint_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
