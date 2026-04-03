cmake_minimum_required(VERSION 3.8)
project(arm_mediapipe)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/ArmCtrl.py
    scripts/Find_Hand.py
    scripts/FingerCtrl.py
    scripts/HandCtrl.py
    scripts/PoseArm.py
    scripts/RobotCtrl.py
    scripts/arm_driver.py
    scripts/media_library.py
    scripts/msgToimg.py
    scripts/roboarm_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    scripts/img
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
