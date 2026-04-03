cmake_minimum_required(VERSION 3.8)
project(dofbot_snake_follow)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/dofbot_config.py
    scripts/snake_ctrl.py
    scripts/snake_move.py
    scripts/snake_target.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    scripts/HSV_config.txt
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
    scripts
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.ipynb"
)

ament_package()
