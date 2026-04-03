cmake_minimum_required(VERSION 3.8)
project(dofbot_garbage_yolov5)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    garbage_identify.py
    garbage_grap.py
    single_garbage_identify.py
    speech_garbage.py
    speech_garbage_identify.py
    dofbot_config.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    models
    utils
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    model.pt
    model0.pt
    XYT_config.txt
  DESTINATION share/${PROJECT_NAME}
)

# Keep notebooks available for migration/teaching workflows.
install(
  DIRECTORY
    .
  DESTINATION share/${PROJECT_NAME}/notebooks
  FILES_MATCHING PATTERN "*.ipynb"
)

ament_package()
