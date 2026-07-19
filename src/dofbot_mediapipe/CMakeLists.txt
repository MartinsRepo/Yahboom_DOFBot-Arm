cmake_minimum_required(VERSION 3.8)
project(dofbot_mediapipe)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/camera_driver.py
    scripts/02_PoseDetector.py
    scripts/04_FaceMesh.py
    scripts/07_FaceDetection.py
    scripts/08_GestureDetection.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
