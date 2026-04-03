cmake_minimum_required(VERSION 3.8)
project(dofbot_color_follow)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    color_follow.py
    dofbot_config.py
    face_follow.py
    positioning.py
    PID.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    HSV_config.txt
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
