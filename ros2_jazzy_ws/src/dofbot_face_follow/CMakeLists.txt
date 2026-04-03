cmake_minimum_required(VERSION 3.8)
project(dofbot_face_follow)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    dofbot_config.py
    face_follow.py
    face_pose.py
    PID.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    haarcascade_frontalface_default.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
    .
  DESTINATION share/${PROJECT_NAME}/notebooks
  FILES_MATCHING PATTERN "*.ipynb"
)

ament_package()
