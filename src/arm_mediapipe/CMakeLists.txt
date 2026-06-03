cmake_minimum_required(VERSION 3.8)
project(arm_mediapipe)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/roboarm_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
