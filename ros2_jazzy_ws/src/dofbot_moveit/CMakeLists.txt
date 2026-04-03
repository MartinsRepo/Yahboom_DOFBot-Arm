cmake_minimum_required(VERSION 3.8)
project(dofbot_moveit)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/00_dofbot_move.py
    scripts/01_set_move.py
    scripts/02_motion_plan.py
    scripts/03_attached_object.py
    scripts/04_Set_Scene.py
    scripts/04_cartesian.py
    scripts/1.py
    scripts/2.py
    scripts/3.py
    scripts/4.py
    scripts/5.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    meshes
    rviz
    urdf
  DESTINATION share/${PROJECT_NAME}
)

# ROS1 MoveIt C++ nodes in src/ remain deferred until dedicated MoveIt2/rclcpp porting.
ament_package()
