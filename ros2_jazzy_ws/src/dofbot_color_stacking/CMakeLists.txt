cmake_minimum_required(VERSION 3.8)
project(dofbot_color_stacking)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/dofbot_config.py
    scripts/stacking_grap.py
    scripts/stacking_target.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    scripts/HSV_config.txt
    scripts/XYT_config.txt
    scripts/color_stacking.txt
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
    scripts
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.ipynb"
)

ament_package()
