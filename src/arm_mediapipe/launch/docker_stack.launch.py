#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_device = LaunchConfiguration('serial_device')
    arm_lib_dir = LaunchConfiguration('arm_lib_dir')
    camera_device = LaunchConfiguration('camera_device')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    camera_fps = LaunchConfiguration('camera_fps')
    show_preview = LaunchConfiguration('show_preview')
    face_detection_min_conf = LaunchConfiguration('face_detection_min_conf')
    enable_face_detection = LaunchConfiguration('enable_face_detection')
    enable_face_mesh = LaunchConfiguration('enable_face_mesh')
    enable_pose = LaunchConfiguration('enable_pose')

    return LaunchDescription([
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('arm_lib_dir', default_value=''),
        DeclareLaunchArgument('camera_device', default_value='0'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='15'),
        DeclareLaunchArgument('show_preview', default_value='0'),
        DeclareLaunchArgument('face_detection_min_conf', default_value='0.60'),
        DeclareLaunchArgument('enable_face_detection', default_value='1'),
        DeclareLaunchArgument('enable_face_mesh', default_value='0'),
        DeclareLaunchArgument('enable_pose', default_value='0'),

        Node(
            package='dofbot_mediapipe',
            executable='camera_driver.py',
            name='dofbot_camera_driver',
            output='screen',
            additional_env={
                'DOFBOT_CAMERA_DEVICE': camera_device,
                'DOFBOT_CAMERA_WIDTH': camera_width,
                'DOFBOT_CAMERA_HEIGHT': camera_height,
                'DOFBOT_CAMERA_FPS': camera_fps,
            },
        ),

        Node(
            package='arm_mediapipe',
            executable='roboarm_bridge.py',
            name='roboarm_bridge',
            output='screen',
            arguments=['--device', serial_device, '--arm-lib-dir', arm_lib_dir],
        ),

        Node(
            package='dofbot_mediapipe',
            executable='07_FaceDetection.py',
            name='face_detection',
            output='screen',
            additional_env={
                'DOFBOT_SHOW_PREVIEW': show_preview,
                'DOFBOT_FACE_DETECTION_MIN_CONF': face_detection_min_conf,
            },
            condition=IfCondition(enable_face_detection),
        ),

        Node(
            package='dofbot_mediapipe',
            executable='04_FaceMesh.py',
            name='face_mesh',
            output='screen',
            condition=IfCondition(enable_face_mesh),
        ),

        Node(
            package='dofbot_mediapipe',
            executable='02_PoseDetector.py',
            name='pose_detector',
            output='screen',
            condition=IfCondition(enable_pose),
        ),
    ])
