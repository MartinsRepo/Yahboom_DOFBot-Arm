#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_device = LaunchConfiguration('serial_device')
    arm_lib_dir = LaunchConfiguration('arm_lib_dir')
    camera_device = LaunchConfiguration('camera_device')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    camera_fps = LaunchConfiguration('camera_fps')
    camera_rotate_180 = LaunchConfiguration('camera_rotate_180')
    show_preview = LaunchConfiguration('show_preview')
    face_detection_min_conf = LaunchConfiguration('face_detection_min_conf')
    enable_face_detection = LaunchConfiguration('enable_face_detection')
    enable_face_mesh = LaunchConfiguration('enable_face_mesh')
    enable_pose = LaunchConfiguration('enable_pose')
    enable_speech_controller = LaunchConfiguration('enable_speech_controller')
    enable_llm_controller = LaunchConfiguration('enable_llm_controller')
    enable_agent_tools = LaunchConfiguration('enable_agent_tools')
    control_mode = LaunchConfiguration('control_mode')
    strict_safety = LaunchConfiguration('strict_safety')
    command_rate_limit_hz = LaunchConfiguration('command_rate_limit_hz')
    llm_stale_timeout_s = LaunchConfiguration('llm_stale_timeout_s')
    manual_override_window_s = LaunchConfiguration('manual_override_window_s')
    vosk_model_dir = LaunchConfiguration('vosk_model_dir')
    speech_device = LaunchConfiguration('speech_device')
    speech_sample_rate = LaunchConfiguration('speech_sample_rate')
    speech_blocksize = LaunchConfiguration('speech_blocksize')
    speech_language = LaunchConfiguration('speech_language')
    speech_topic = LaunchConfiguration('speech_topic')
    speech_flush_silence_s = LaunchConfiguration('speech_flush_silence_s')
    wake_word = LaunchConfiguration('wake_word')
    wake_word_timeout_s = LaunchConfiguration('wake_word_timeout_s')

    return LaunchDescription([
        DeclareLaunchArgument('serial_device', default_value='auto'),
        DeclareLaunchArgument('arm_lib_dir', default_value=''),
        DeclareLaunchArgument('camera_device', default_value='0'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='15'),
        DeclareLaunchArgument('camera_rotate_180', default_value='1'),
        DeclareLaunchArgument('show_preview', default_value='0'),
        DeclareLaunchArgument('face_detection_min_conf', default_value='0.60'),
        DeclareLaunchArgument('enable_face_detection', default_value='1'),
        DeclareLaunchArgument('enable_face_mesh', default_value='0'),
        DeclareLaunchArgument('enable_pose', default_value='0'),
        DeclareLaunchArgument('enable_speech_controller', default_value='0'),
        DeclareLaunchArgument('enable_llm_controller', default_value='0'),
        DeclareLaunchArgument('enable_agent_tools', default_value='1'),
        DeclareLaunchArgument('control_mode', default_value='GUI'),
        DeclareLaunchArgument('strict_safety', default_value='1'),
        DeclareLaunchArgument('command_rate_limit_hz', default_value='8.0'),
        DeclareLaunchArgument('llm_stale_timeout_s', default_value='2.0'),
        DeclareLaunchArgument('manual_override_window_s', default_value='1.5'),
        DeclareLaunchArgument('vosk_model_dir', default_value='/opt/ros/overlay_ws/models/vosk-model-small-de-zamia-0.3'),
        DeclareLaunchArgument('speech_device', default_value='pulse'),
        DeclareLaunchArgument('speech_sample_rate', default_value='16000'),
        DeclareLaunchArgument('speech_blocksize', default_value='2000'),
        DeclareLaunchArgument('speech_language', default_value='de'),
        DeclareLaunchArgument('speech_topic', default_value='roboarm/speech_input'),
        DeclareLaunchArgument('speech_flush_silence_s', default_value='0.4'),
        DeclareLaunchArgument('wake_word', default_value='hallo'),
        DeclareLaunchArgument('wake_word_timeout_s', default_value='4.0'),

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
                'DOFBOT_CAMERA_ROTATE_180': camera_rotate_180,
            },
        ),

        Node(
            package='arm_mediapipe',
            executable='roboarm_bridge.py',
            name='roboarm_bridge',
            output='screen',
            arguments=['--device', serial_device, '--arm-lib-dir', arm_lib_dir],
            additional_env={
                'DOFBOT_CONTROL_MODE': control_mode,
                'DOFBOT_STRICT_SAFETY': strict_safety,
                'DOFBOT_COMMAND_RATE_LIMIT_HZ': command_rate_limit_hz,
                'DOFBOT_LLM_STALE_TIMEOUT_S': llm_stale_timeout_s,
                'DOFBOT_MANUAL_OVERRIDE_WINDOW_S': manual_override_window_s,
            },
        ),

        Node(
            package='arm_mediapipe',
            executable='llm_controller.py',
            name='llm_controller',
            output='screen',
            condition=IfCondition(enable_llm_controller),
            additional_env={
                'DOFBOT_AGENT_TOOLS_ENABLED': enable_agent_tools,
            },
        ),

        Node(
            package='arm_mediapipe',
            executable='tool_executor.py',
            name='tool_executor',
            output='screen',
            condition=IfCondition(enable_agent_tools),
            additional_env={
                'ENABLE_ARM_AGENT_TOOLS': enable_agent_tools,
                'DOFBOT_TOOL_MAX_DURATION_MS': EnvironmentVariable('DOFBOT_TOOL_MAX_DURATION_MS', default_value='1200'),
                'DOFBOT_TOOL_ALLOWED_ACTIONS': EnvironmentVariable('DOFBOT_TOOL_ALLOWED_ACTIONS', default_value=''),
            },
        ),

        Node(
            package='arm_mediapipe',
            executable='speech_input.py',
            name='speech_input',
            output='screen',
            condition=IfCondition(enable_speech_controller),
            additional_env={
                'DOFBOT_VOSK_MODEL_DIR': vosk_model_dir,
                'DOFBOT_SPEECH_DEVICE': speech_device,
                'DOFBOT_SPEECH_SAMPLE_RATE': speech_sample_rate,
                'DOFBOT_SPEECH_BLOCKSIZE': speech_blocksize,
                'DOFBOT_SPEECH_LANGUAGE': speech_language,
                'DOFBOT_SPEECH_TOPIC': speech_topic,
                'DOFBOT_SPEECH_FLUSH_SILENCE_S': speech_flush_silence_s,
                'DOFBOT_WAKE_WORD': wake_word,
                'DOFBOT_WAKE_WORD_TIMEOUT_S': wake_word_timeout_s,
            },
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
