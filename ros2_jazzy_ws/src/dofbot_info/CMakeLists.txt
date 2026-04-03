cmake_minimum_required(VERSION 3.8)
project(dofbot_info)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Kinemarics.srv"
)

ament_export_dependencies(rosidl_default_runtime)

# Install Python scripts
install(PROGRAMS
  scripts/kinemarics_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
