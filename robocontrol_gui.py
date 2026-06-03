"""Qt front-end for steering the DOFBot arm from inside the container."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Dict

import rclpy
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QCheckBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String


FACE_DETECTION_TOPIC = '/mediapipe/face_detection_summary'


@dataclass
class ArmStatus:
    connected: bool = False
    active: bool = False
    device: str = ""
    status_text: str = "Starting"
    last_error: str = ""


@dataclass
class FaceSummary:
    status: str = 'no face'
    center_x: float | None = None
    center_y: float | None = None
    bbox_width: float | None = None
    bbox_height: float | None = None
    keypoints: Dict[str, Dict[str, float]] = None


class RoboArmController(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        if not rclpy.ok():
            rclpy.init(args=None)

        self.setWindowTitle("RoboControl")
        self.resize(1280, 860)

        self.node = rclpy.create_node("robocontrol_gui")
        self.command_publisher = self.node.create_publisher(String, "roboarm/command", 10)
        self.face_detection_publisher = self.node.create_publisher(Bool, "/robocontrol/face_detection_enabled", 10)
        self.node.create_subscription(String, "roboarm/status", self._handle_status, 10)
        self.node.create_subscription(CompressedImage, "/mediapipe/camera/image/compressed", self._handle_camera, 10)
        self.node.create_subscription(String, FACE_DETECTION_TOPIC, self._handle_face_summary, 10)

        self.status = ArmStatus()
        self.face_summary = FaceSummary(keypoints={})
        self.last_frame = None

        self.preview_label = QLabel("Waiting for camera stream")
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setMinimumSize(960, 540)
        self.preview_label.setStyleSheet("background: #101014; color: #d8d8d8; border: 1px solid #303030;")

        self.status_label = QLabel("Starting")
        self.status_label.setWordWrap(True)

        self.error_label = QLabel("")
        self.error_label.setWordWrap(True)
        self.error_label.setStyleSheet("color: #d9534f;")

        self.face_detection_checkbox = QCheckBox("Face detection overlays")
        self.face_detection_checkbox.setChecked(False)
        self.face_detection_checkbox.toggled.connect(self._publish_face_detection_state)

        self._build_ui()

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin_ros_once)
        self.spin_timer.start(20)

        self._publish_face_detection_state(False)

    def _build_ui(self) -> None:
        root = QWidget(self)
        self.setCentralWidget(root)

        main_layout = QVBoxLayout(root)
        main_layout.addWidget(self.preview_label, stretch=1)

        status_group = QGroupBox("Arm status")
        status_layout = QVBoxLayout(status_group)
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.error_label)
        status_layout.addWidget(self.face_detection_checkbox)
        main_layout.addWidget(status_group)

        controls_group = QGroupBox("Controls")
        controls_layout = QGridLayout(controls_group)

        button_specs = [
            ("Power on", "power_on", 0, 0),
            ("Power off", "power_off", 0, 1),
            ("Home", "home", 0, 2),
            ("Refresh", "refresh", 0, 3),
            ("Base -", "move_left", 1, 0),
            ("Base +", "move_right", 1, 1),
            ("Shoulder +", "move_up", 1, 2),
            ("Shoulder -", "move_down", 1, 3),
            ("Rotate -", "turn_left", 2, 0),
            ("Rotate +", "turn_right", 2, 1),
            ("Stretch", "arm_stretch", 2, 2),
            ("Shrink", "arm_shrink", 2, 3),
            ("Grip open", "grip_open", 3, 0),
            ("Grip close", "grip_close", 3, 1),
        ]

        for label, action, row, column in button_specs:
            button = QPushButton(label)
            button.clicked.connect(lambda checked=False, action=action: self._send_action(action))
            controls_layout.addWidget(button, row, column)

        main_layout.addWidget(controls_group)

    def _spin_ros_once(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def _send_action(self, action: str) -> None:
        if action not in {"power_on", "power_off", "home", "refresh"} and not self.status.active:
            self._publish_command("power_on")
        self._publish_command(action)

    def _publish_command(self, action: str, **payload: object) -> None:
        message = String()
        command_payload = {"action": action}
        command_payload.update(payload)
        message.data = json.dumps(command_payload)
        self.command_publisher.publish(message)

    def _publish_face_detection_state(self, enabled: bool) -> None:
        message = Bool()
        message.data = bool(enabled)
        self.face_detection_publisher.publish(message)

    def _handle_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            payload = {"status_text": message.data, "last_error": ""}

        self.status = ArmStatus(
            connected=bool(payload.get("connected", False)),
            active=bool(payload.get("active", False)),
            device=str(payload.get("device", "")),
            status_text=str(payload.get("status_text", "")),
            last_error=str(payload.get("last_error", "")),
        )

        status_parts = [
            f"Connected: {'yes' if self.status.connected else 'no'}",
            f"Active: {'yes' if self.status.active else 'no'}",
            f"Device: {self.status.device or 'n/a'}",
            f"State: {self.status.status_text or 'n/a'}",
        ]
        self.status_label.setText(" | ".join(status_parts))
        self.error_label.setText(self.status.last_error)

    def _handle_camera(self, message: CompressedImage) -> None:
        image = QImage.fromData(message.data, 'JPG')
        if image.isNull():
            return

        self.last_frame = image.copy()
        pixmap = QPixmap.fromImage(self.last_frame)

        if self.face_detection_checkbox.isChecked():
            self._draw_face_overlay(pixmap)

        pixmap = pixmap.scaled(
            self.preview_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.preview_label.setPixmap(pixmap)

    def _handle_face_summary(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return

        keypoints = payload.get('keypoints', {})
        if not isinstance(keypoints, dict):
            keypoints = {}

        self.face_summary = FaceSummary(
            status=str(payload.get('status', 'no face')),
            center_x=payload.get('center_x'),
            center_y=payload.get('center_y'),
            bbox_width=payload.get('bbox_width'),
            bbox_height=payload.get('bbox_height'),
            keypoints=keypoints,
        )

    def _draw_face_overlay(self, pixmap: QPixmap) -> None:
        if self.face_summary.status != 'tracking':
            return
        if self.face_summary.center_x is None or self.face_summary.center_y is None:
            return
        if self.face_summary.bbox_width is None or self.face_summary.bbox_height is None:
            return

        width = pixmap.width()
        height = pixmap.height()
        center_x = int(float(self.face_summary.center_x) * width)
        center_y = int(float(self.face_summary.center_y) * height)
        box_w = int(float(self.face_summary.bbox_width) * width)
        box_h = int(float(self.face_summary.bbox_height) * height)
        x = center_x - box_w // 2
        y = center_y - box_h // 2

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        box_pen = QPen(QColor(255, 0, 255), 2)
        painter.setPen(box_pen)
        painter.drawRect(x, y, box_w, box_h)

        marker_colors = {
            'left_eye': QColor(0, 255, 255),
            'right_eye': QColor(0, 255, 255),
            'nose': QColor(255, 255, 0),
            'mouth': QColor(0, 200, 255),
        }
        for key, color in marker_colors.items():
            point = (self.face_summary.keypoints or {}).get(key)
            if not isinstance(point, dict):
                continue
            px = point.get('x')
            py = point.get('y')
            if px is None or py is None:
                continue
            point_x = int(float(px) * width)
            point_y = int(float(py) * height)
            painter.setPen(QPen(color, 1))
            painter.setBrush(color)
            painter.drawEllipse(point_x - 4, point_y - 4, 8, 8)

        painter.end()

    def closeEvent(self, event) -> None:
        self.spin_timer.stop()
        if hasattr(self, "node") and self.node is not None:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        super().closeEvent(event)
