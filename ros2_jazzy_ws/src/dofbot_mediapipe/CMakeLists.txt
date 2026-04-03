cmake_minimum_required(VERSION 3.8)
project(dofbot_mediapipe)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/01_HandDetector.py
    scripts/02_PoseDetector.py
    scripts/03_Holistic.py
    scripts/04_FaceMesh.py
    scripts/05_FaceEyeDetection.py
    scripts/06_FaceLandmarks.py
    scripts/07_FaceDetection.py
    scripts/08_Objectron.py
    scripts/09_VirtualPaint.py
    scripts/10_HandCtrl.py
    scripts/11_GestureRecognition.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    scripts/file
    scripts/img
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
