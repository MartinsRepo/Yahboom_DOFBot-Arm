#!/usr/bin/env python3
# encoding: utf-8
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

import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


CAMERA_TOPIC = '/mediapipe/camera/image/compressed'


class CameraDriver(Node):
    def __init__(self, camera_device: str, width: int, height: int, fps: int, node_name: str, topic: str):
        super().__init__(node_name)
        self.topic = topic
        self.pub = self.create_publisher(CompressedImage, self.topic, 10)
        self.rotate_180 = os.environ.get('DOFBOT_CAMERA_ROTATE_180', '1').strip().lower() in (
            '1', 'true', 'yes', 'on'
        )
        self.capture = cv.VideoCapture(int(camera_device) if camera_device.isdigit() else camera_device)
        self.capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.capture.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, height)
        interval = 1.0 / max(float(fps), 1.0)
        self.timer = self.create_timer(interval, self._publish_frame)

        if not self.capture.isOpened():
            self.get_logger().error(f'Failed to open camera device: {camera_device}')
        else:
            transform = 'enabled' if self.rotate_180 else 'disabled'
            self.get_logger().info(
                f'Publishing camera frames on {self.topic} from {camera_device} (180deg rotation {transform})'
            )

    def _publish_frame(self):
        if not self.capture.isOpened():
            return
        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warning('Failed to read camera frame')
            return
        if self.rotate_180:
            frame = cv.rotate(frame, cv.ROTATE_180)
        ok, encoded = cv.imencode('.jpg', frame, [int(cv.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            self.get_logger().warning('Failed to encode camera frame')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            if self.capture is not None:
                self.capture.release()
        finally:
            super().destroy_node()


def main():
    rclpy.init(args=None)
    camera_device = os.environ.get('DOFBOT_CAMERA_DEVICE', '0')
    width = int(os.environ.get('DOFBOT_CAMERA_WIDTH', '640'))
    height = int(os.environ.get('DOFBOT_CAMERA_HEIGHT', '480'))
    fps = int(os.environ.get('DOFBOT_CAMERA_FPS', '15'))
    node_name = os.environ.get('DOFBOT_CAMERA_NODE_NAME', 'dofbot_camera_driver')
    topic = os.environ.get('DOFBOT_CAMERA_TOPIC', CAMERA_TOPIC)
    node = CameraDriver(
        camera_device=camera_device,
        width=width,
        height=height,
        fps=fps,
        node_name=node_name,
        topic=topic,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()