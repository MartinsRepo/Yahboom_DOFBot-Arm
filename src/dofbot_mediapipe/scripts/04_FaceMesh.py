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


FACE_MESH_TOPIC = '/mediapipe/face_mesh_summary'
CAMERA_TOPIC = '/mediapipe/camera/image/compressed'
FACE_MESH_NOSE_INDEX = 1
SHOW_PREVIEW = bool(os.environ.get('DISPLAY'))


class FaceMesh(Node):
    def __init__(self, staticMode=False, maxFaces=2, minDetectionCon=0.5, minTrackingCon=0.5):
        super().__init__('face_mesh_detector')
        self.mpDraw = mp.solutions.drawing_utils
        self.mpFaceMesh = mp.solutions.face_mesh
        self.faceMesh = self.mpFaceMesh.FaceMesh(
            static_image_mode=staticMode,
            max_num_faces=maxFaces,
            min_detection_confidence=minDetectionCon,
            min_tracking_confidence=minTrackingCon)
        self.pub_summary = self.create_publisher(String, FACE_MESH_TOPIC, 10)
        self.subscription = self.create_subscription(CompressedImage, CAMERA_TOPIC, self._handle_camera_frame, 10)
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=3)
        self.drawSpec = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1)
        self.prev_frame_ts = 0.0

    def pubFaceMeshPoint(self, frame, draw=True):
        img = np.zeros(frame.shape, np.uint8)
        imgRGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.faceMesh.process(imgRGB)
        summary_payload = {
            'faces_detected': 0,
            'center_x': None,
            'center_y': None,
            'nose_x': None,
            'nose_y': None,
            'nose_z': None,
            'status': 'no face',
        }
        if self.results.multi_face_landmarks:
            primary_face_points = []
            summary_payload['faces_detected'] = len(self.results.multi_face_landmarks)
            summary_payload['status'] = 'tracking'
            for i in range(len(self.results.multi_face_landmarks)):
                if draw: self.mpDraw.draw_landmarks(frame, self.results.multi_face_landmarks[i], self.mpFaceMesh.FACEMESH_CONTOURS, self.lmDrawSpec, self.drawSpec)
                self.mpDraw.draw_landmarks(img, self.results.multi_face_landmarks[i], self.mpFaceMesh.FACEMESH_CONTOURS, self.lmDrawSpec, self.drawSpec)
                if i == 0:
                    primary_face_points = list(self.results.multi_face_landmarks[i].landmark)

            if primary_face_points:
                summary_payload['center_x'] = sum(point.x for point in primary_face_points) / len(primary_face_points)
                summary_payload['center_y'] = sum(point.y for point in primary_face_points) / len(primary_face_points)
                nose_point = primary_face_points[min(FACE_MESH_NOSE_INDEX, len(primary_face_points) - 1)]
                summary_payload['nose_x'] = nose_point.x
                summary_payload['nose_y'] = nose_point.y
                summary_payload['nose_z'] = nose_point.z

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

        frame, img = self.pubFaceMeshPoint(frame, draw=False)
        now = time.time()
        fps = 1.0 / max(now - self.prev_frame_ts, 1e-6) if self.prev_frame_ts else 0.0
        self.prev_frame_ts = now
        if SHOW_PREVIEW:
            cv.putText(frame, f'FPS : {int(fps)}', (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
            dst = self.frame_combine(frame, img)
            cv.imshow('dst', dst)
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
    face_mesh = FaceMesh(maxFaces=2)
    face_mesh.get_logger().info(f'Subscribing to {CAMERA_TOPIC} and publishing {FACE_MESH_TOPIC}')
    try:
        rclpy.spin(face_mesh)
    finally:
        cv.destroyAllWindows()
        face_mesh.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
