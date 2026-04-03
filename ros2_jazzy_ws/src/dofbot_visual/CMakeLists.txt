cmake_minimum_required(VERSION 3.8)
project(dofbot_visual)

find_package(ament_cmake REQUIRED)

option(DOFBOT_VISUAL_BUILD_CPP_NODES "Build migrated rclcpp demo nodes" OFF)

if(DOFBOT_VISUAL_BUILD_CPP_NODES)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(OpenCV REQUIRED)

  add_executable(get_rgb_image src/get_rgb_image.cpp)
  ament_target_dependencies(get_rgb_image rclcpp sensor_msgs cv_bridge)
  target_link_libraries(get_rgb_image ${OpenCV_LIBS})

  install(TARGETS
    get_rgb_image
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  PROGRAMS
    AR/simple_AR.py
    detection/target_detection.py
    scripts/astra_color_point.py
    scripts/astra_depth_image.py
    scripts/astra_image_flip.py
    scripts/astra_rgb_image.py
    scripts/laser_to_image.py
    scripts/pub_image.py
    scripts/sub_point.py
    simple_qrcode/QRcode_Create.py
    simple_qrcode/QRcode_Parsing.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    AR
    detection
    font
    Marker
    rviz
    launch
    simple_qrcode
  DESTINATION share/${PROJECT_NAME}
)

# C++ demo node is now ported to rclcpp and can be enabled with -DDOFBOT_VISUAL_BUILD_CPP_NODES=ON.
ament_package()
