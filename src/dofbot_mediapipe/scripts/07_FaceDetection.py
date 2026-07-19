#!/usr/bin/env python3
# encoding: utf-8
import json
import os
import sys


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
from std_msgs.msg import Bool
from std_msgs.msg import String


FACE_DETECTION_TOPIC = '/mediapipe/face_detection_summary'
CAMERA_TOPIC = '/mediapipe/camera/image/compressed'
FACE_DETECTION_ENABLED_TOPIC = '/robocontrol/face_detection_enabled'
SHOW_PREVIEW = os.environ.get('DOFBOT_SHOW_PREVIEW', '0').strip().lower() in ('1', 'true', 'yes', 'on')
RAW_CAMERA_WINDOW = 'camera_stream'
FACE_DETECTION_WINDOW = 'face_detection'
PREVIEW_WINDOW_MARGIN = 24


class FaceDetector(Node):
    def __init__(self, minDetectionCon=0.75):
        super().__init__('face_detector')
        self.mpFaceDetection = mp.solutions.face_detection
        self.facedetection = self.mpFaceDetection.FaceDetection(min_detection_confidence=minDetectionCon)
        self.pub_summary = self.create_publisher(String, FACE_DETECTION_TOPIC, 10)
        self.subscription = self.create_subscription(CompressedImage, CAMERA_TOPIC, self._handle_camera_frame, 10)
        self.enable_subscription = self.create_subscription(Bool, FACE_DETECTION_ENABLED_TOPIC, self._handle_detection_enabled, 10)
        self.overlay_enabled = False
        self.prev_frame_ts = 0.0
        self.preview_windows_ready = False

    def _handle_detection_enabled(self, message: Bool):
        self.overlay_enabled = bool(message.data)

    def pubFaceDetection(self, frame, draw=True):
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.facedetection.process(img_RGB)
        summary_payload = {
            'faces_detected': 0,
            'center_x': None,
            'center_y': None,
            'bbox_width': None,
            'bbox_height': None,
            'score': None,
            'keypoints': {},
            'status': 'no face',
        }

        detections = []
        if results.detections:
            summary_payload['faces_detected'] = len(results.detections)
            summary_payload['status'] = 'tracking'
            for detection in results.detections:
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, _ = frame.shape
                bbox = (
                    int(bboxC.xmin * iw),
                    int(bboxC.ymin * ih),
                    int(bboxC.width * iw),
                    int(bboxC.height * ih),
                )
                score = float(detection.score[0]) if detection.score else 0.0
                keypoints = self._extract_face_keypoints(detection)
                detections.append((bbox, score, keypoints))
                if draw:
                    self._fancyDraw(frame, bbox)
                    self._draw_face_keypoints(frame, detection)
                    cv.putText(
                        frame,
                        f'{int(score * 100)}%',
                        (bbox[0], bbox[1] - 20),
                        cv.FONT_HERSHEY_PLAIN,
                        3,
                        (255, 0, 255),
                        2,
                    )

            primary_bbox, primary_score, primary_keypoints = max(detections, key=lambda item: item[1])
            center_x = primary_bbox[0] + primary_bbox[2] / 2.0
            center_y = primary_bbox[1] + primary_bbox[3] / 2.0
            summary_payload['center_x'] = center_x / max(frame.shape[1], 1)
            summary_payload['center_y'] = center_y / max(frame.shape[0], 1)
            summary_payload['bbox_width'] = primary_bbox[2] / max(frame.shape[1], 1)
            summary_payload['bbox_height'] = primary_bbox[3] / max(frame.shape[0], 1)
            summary_payload['score'] = primary_score
            summary_payload['keypoints'] = primary_keypoints

        message = String()
        message.data = json.dumps(summary_payload)
        self.pub_summary.publish(message)
        return frame

    def _extract_face_keypoints(self, detection):
        keypoint_map = {
            'left_eye': self.mpFaceDetection.FaceKeyPoint.LEFT_EYE,
            'right_eye': self.mpFaceDetection.FaceKeyPoint.RIGHT_EYE,
            'nose': self.mpFaceDetection.FaceKeyPoint.NOSE_TIP,
            'mouth': self.mpFaceDetection.FaceKeyPoint.MOUTH_CENTER,
        }
        points = {}
        for key, enum_value in keypoint_map.items():
            kp = self.mpFaceDetection.get_key_point(detection, enum_value)
            points[key] = {'x': float(kp.x), 'y': float(kp.y)}
        return points

    def _draw_face_keypoints(self, frame, detection):
        ih, iw, _ = frame.shape
        keypoint_map = [
            (self.mpFaceDetection.FaceKeyPoint.LEFT_EYE, 'L eye', (0, 255, 255)),
            (self.mpFaceDetection.FaceKeyPoint.RIGHT_EYE, 'R eye', (0, 255, 255)),
            (self.mpFaceDetection.FaceKeyPoint.NOSE_TIP, 'Nose', (255, 255, 0)),
            (self.mpFaceDetection.FaceKeyPoint.MOUTH_CENTER, 'Mouth', (0, 200, 255)),
        ]
        for keypoint_enum, label, color in keypoint_map:
            kp = self.mpFaceDetection.get_key_point(detection, keypoint_enum)
            x = int(kp.x * iw)
            y = int(kp.y * ih)
            cv.circle(frame, (x, y), 4, color, -1)
            cv.putText(frame, label, (x + 6, y - 6), cv.FONT_HERSHEY_PLAIN, 1.0, color, 1)

    def _handle_camera_frame(self, message: CompressedImage):
        data = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv.imdecode(data, cv.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('Failed decoding compressed camera frame')
            return

        if SHOW_PREVIEW:
            self._ensure_preview_windows(frame)
            cv.imshow(RAW_CAMERA_WINDOW, frame)

        frame = self.pubFaceDetection(frame, draw=(SHOW_PREVIEW and self.overlay_enabled))
        now = time.time()
        fps = 1.0 / max(now - self.prev_frame_ts, 1e-6) if self.prev_frame_ts else 0.0
        self.prev_frame_ts = now
        if SHOW_PREVIEW:
            cv.putText(frame, f'FPS : {int(fps)}', (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
            cv.imshow(FACE_DETECTION_WINDOW, frame)
            cv.waitKey(1)

    def _ensure_preview_windows(self, frame):
        if self.preview_windows_ready:
            return

        frame_height, frame_width = frame.shape[:2]
        window_width = max(320, frame_width)
        window_height = max(240, frame_height)
        top_left_x = 40
        top_left_y = 40

        cv.namedWindow(RAW_CAMERA_WINDOW, cv.WINDOW_NORMAL)
        cv.namedWindow(FACE_DETECTION_WINDOW, cv.WINDOW_NORMAL)
        cv.resizeWindow(RAW_CAMERA_WINDOW, window_width, window_height)
        cv.resizeWindow(FACE_DETECTION_WINDOW, window_width, window_height)
        
        # Position the windows horizontally!
        cv.moveWindow(RAW_CAMERA_WINDOW, top_left_x, top_left_y)
        cv.moveWindow(FACE_DETECTION_WINDOW, top_left_x + window_width + PREVIEW_WINDOW_MARGIN, top_left_y)
        self.preview_windows_ready = True

    @staticmethod
    def _fancyDraw(frame, bbox, line_length=30, thickness=10):
        x, y, w, h = bbox
        x1, y1 = x + w, y + h
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv.line(frame, (x, y), (x + line_length, y), (255, 0, 255), thickness)
        cv.line(frame, (x, y), (x, y + line_length), (255, 0, 255), thickness)
        cv.line(frame, (x1, y), (x1 - line_length, y), (255, 0, 255), thickness)
        cv.line(frame, (x1, y), (x1, y + line_length), (255, 0, 255), thickness)
        cv.line(frame, (x, y1), (x + line_length, y1), (255, 0, 255), thickness)
        cv.line(frame, (x, y1), (x, y1 - line_length), (255, 0, 255), thickness)
        cv.line(frame, (x1, y1), (x1 - line_length, y1), (255, 0, 255), thickness)
        cv.line(frame, (x1, y1), (x1, y1 - line_length), (255, 0, 255), thickness)
        return frame


if __name__ == '__main__':
    rclpy.init(args=None)
    min_conf = float(os.environ.get('DOFBOT_FACE_DETECTION_MIN_CONF', '0.60'))
    face_detector = FaceDetector(min_conf)
    face_detector.get_logger().info(
        f'Subscribing to {CAMERA_TOPIC} and publishing {FACE_DETECTION_TOPIC} '
        f"(preview={'on' if SHOW_PREVIEW else 'off'})"
    )
    try:
        rclpy.spin(face_detector)
    finally:
        cv.destroyAllWindows()
        face_detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
