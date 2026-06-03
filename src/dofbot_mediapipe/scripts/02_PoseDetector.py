#!/usr/bin/env python3
# encoding: utf-8
import os
import sys
import json


PY312_ENV_MARKER = 'DOFBOT_PY312_REEXEC'


def ensure_py312_runtime():
    if sys.version_info[:2] == (3, 12):
        return
    if os.environ.get(PY312_ENV_MARKER) == '1':
        return

    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    py312_python = os.path.join(workspace_root, '.venv312', 'bin', 'python')
    if not os.path.exists(py312_python):
        raise RuntimeError(
            f'Python 3.12 runtime not found at {py312_python}. '
            'Create it before running ROS Jazzy detector scripts.'
        )

    new_env = dict(os.environ)
    new_env[PY312_ENV_MARKER] = '1'
    host_lib_dir = os.path.join(workspace_root, '.host-libs')
    if os.path.isdir(host_lib_dir):
        existing_ld_path = new_env.get('LD_LIBRARY_PATH', '')
        new_env['LD_LIBRARY_PATH'] = host_lib_dir if not existing_ld_path else f'{host_lib_dir}:{existing_ld_path}'
    os.execvpe(py312_python, [py312_python, os.path.abspath(__file__), *sys.argv[1:]], new_env)


ensure_py312_runtime()

import time
import cv2 as cv
import numpy as np
import mediapipe as mp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


POSE_TOPIC = '/mediapipe/pose_summary'
CAMERA_TOPIC = '/mediapipe/camera/image/compressed'
POSE_NOSE_INDEX = 0
POSE_LEFT_SHOULDER_INDEX = 11
POSE_RIGHT_SHOULDER_INDEX = 12
SHOW_PREVIEW = bool(os.environ.get('DISPLAY'))

class PoseDetector(Node):
    def __init__(self, mode=False, smooth=True, detectionCon=0.5, trackCon=0.5):
        super().__init__('pose_detector')
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils
        self.pose = self.mpPose.Pose(
            static_image_mode=mode,
            smooth_landmarks=smooth,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackCon)
        self.pub_summary = self.create_publisher(String, POSE_TOPIC, 10)
        self.subscription = self.create_subscription(CompressedImage, CAMERA_TOPIC, self._handle_camera_frame, 10)
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)
        self.prev_frame_ts = 0.0

    def pubPosePoint(self, frame, draw=True):
        img = np.zeros(frame.shape, np.uint8)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.pose.process(img_RGB)
        summary_payload = {
            'points_detected': 0,
            'shoulder_x': None,
            'shoulder_y': None,
            'nose_x': None,
            'nose_y': None,
            'nose_z': None,
            'status': 'no pose',
        }
        if self.results.pose_landmarks:
            if draw: self.mpDraw.draw_landmarks(frame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            self.mpDraw.draw_landmarks(img, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            pose_points = list(self.results.pose_landmarks.landmark)
            summary_payload['points_detected'] = len(pose_points)
            summary_payload['status'] = 'tracking'
            nose_point = pose_points[min(POSE_NOSE_INDEX, len(pose_points) - 1)]
            summary_payload['nose_x'] = nose_point.x
            summary_payload['nose_y'] = nose_point.y
            summary_payload['nose_z'] = nose_point.z

            shoulder_points = []
            for index in (POSE_LEFT_SHOULDER_INDEX, POSE_RIGHT_SHOULDER_INDEX):
                if index < len(pose_points):
                    shoulder_points.append(pose_points[index])
            if shoulder_points:
                summary_payload['shoulder_x'] = sum(point.x for point in shoulder_points) / len(shoulder_points)
                summary_payload['shoulder_y'] = sum(point.y for point in shoulder_points) / len(shoulder_points)

        message = String()
        message.data = json.dumps(summary_payload)
        self.pub_summary.publish(message)
        return frame, img

    def _handle_camera_frame(self, message: CompressedImage):
        data = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv.imdecode(data, cv.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('Failed decoding compressed camera frame')
            return

        frame, img = self.pubPosePoint(frame, draw=False)
        now = time.time()
        fps = 1.0 / max(now - self.prev_frame_ts, 1e-6) if self.prev_frame_ts else 0.0
        self.prev_frame_ts = now
        if SHOW_PREVIEW:
            cv.putText(frame, f'FPS : {int(fps)}', (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
            dist = self.frame_combine(frame, img)
            cv.imshow('dist', dist)
            cv.waitKey(1)

    def frame_combine(slef,frame, src):
        if len(frame.shape) == 3:
            frameH, frameW = frame.shape[:2]
            srcH, srcW = src.shape[:2]
            dst = np.zeros((max(frameH, srcH), frameW + srcW, 3), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        else:
            src = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
            frameH, frameW = frame.shape[:2]
            imgH, imgW = src.shape[:2]
            dst = np.zeros((frameH, frameW + imgW), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        return dst

if __name__ == '__main__':
    rclpy.init(args=None)
    pose_detector = PoseDetector()
    pose_detector.get_logger().info(f'Subscribing to {CAMERA_TOPIC} and publishing {POSE_TOPIC}')
    try:
        rclpy.spin(pose_detector)
    finally:
        cv.destroyAllWindows()
        pose_detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
