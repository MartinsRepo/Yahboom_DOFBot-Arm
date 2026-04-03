cmake_minimum_required(VERSION 3.8)
project(dofbot_color_identify)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/dofbot_config.py
    scripts/identify_grap.py
    scripts/identify_target.py
    scripts/speech_identify_target.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    scripts/HSV_config.txt
    scripts/XYT_config.txt
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
    scripts
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.ipynb"
)

ament_package()
