"""Qt front-end for steering the DOFBot arm from inside the container."""

from __future__ import annotations

import json
import html
import os
from dataclasses import dataclass
from typing import Dict

import rclpy
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String


FACE_DETECTION_TOPIC = '/mediapipe/face_detection_summary'
SPEECH_INPUT_TOPIC = 'roboarm/speech_input'
SPEECH_STATUS_TOPIC = 'roboarm/speech_status'
CONTROL_MODES = ('GUI', 'LLM', 'AUTO')
MOVE_DURATION_MIN_MS = 100
MOVE_DURATION_MAX_MS = 1200
SPEED_SLIDER_MIN = 1
SPEED_SLIDER_MAX = 100


def _env_int(name: str, default: int, minimum: int, maximum: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        value = int(raw)
    except ValueError:
        return default
    return max(minimum, min(value, maximum))


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
        self.setGeometry(100, 100, 1080, 680)
        self.setMinimumSize(980, 600)
        self.setStyleSheet("background-color: #f0f0f0;")

        self.node = rclpy.create_node("robocontrol_gui")
        self.command_publisher = self.node.create_publisher(String, "roboarm/command", 10)
        self.face_detection_publisher = self.node.create_publisher(Bool, "/robocontrol/face_detection_enabled", 10)
        self.mode_publisher = self.node.create_publisher(String, "/robocontrol/mode", 10)
        self.node.create_subscription(String, "roboarm/status", self._handle_status, 10)
        self.node.create_subscription(CompressedImage, "/mediapipe/camera/image/compressed", self._handle_camera, 10)
        self.node.create_subscription(String, FACE_DETECTION_TOPIC, self._handle_face_summary, 10)
        self.node.create_subscription(String, SPEECH_INPUT_TOPIC, self._handle_speech_input, 10)
        self.node.create_subscription(String, SPEECH_STATUS_TOPIC, self._handle_speech_status, 10)

        self.status = ArmStatus()
        self.latest_status_payload: dict = {}
        self.face_summary = FaceSummary(keypoints={})
        self.face_detection_enabled = False
        self.control_mode = 'GUI'
        self.last_spoken_command = ''
        self.last_heard_speech = ''
        self.speech_state = 'n/a'
        self.speech_message = 'n/a'
        self.last_frame = None
        self._held_action: str | None = None
        self._hold_repeat_initial_delay_ms = _env_int(
            'DOFBOT_HOLD_REPEAT_INITIAL_DELAY_MS',
            default=260,
            minimum=50,
            maximum=2000,
        )
        self._hold_repeat_interval_ms = _env_int(
            'DOFBOT_HOLD_REPEAT_INTERVAL_MS',
            default=140,
            minimum=40,
            maximum=1000,
        )
        self._hold_repeat_timer = QTimer(self)
        self._hold_repeat_timer.setSingleShot(True)
        self._hold_repeat_timer.timeout.connect(self._repeat_held_action)
        self.move_duration_ms = _env_int(
            'DOFBOT_MOVE_DURATION_MS',
            default=120,
            minimum=MOVE_DURATION_MIN_MS,
            maximum=MOVE_DURATION_MAX_MS,
        )

        self._build_ui()

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin_ros_once)
        self.spin_timer.start(20)

        self._publish_face_detection_state(self.face_detection_enabled)

    def _build_ui(self) -> None:
        root = QWidget(self)
        self.setCentralWidget(root)

        main_layout = QHBoxLayout(root)
        left_layout = QVBoxLayout()

        top_layout = QHBoxLayout()
        text_style = "background-color: white; border: 2px solid black; font-family: monospace;"

        self.pos_display = QTextEdit()
        self.pos_display.setReadOnly(True)
        self.pos_display.setStyleSheet(text_style)
        self.pos_display.setPlaceholderText("SERVO1 ANGLE / POS\\nSERVO2 ANGLE / POS\\n...")

        self.speed_display = QTextEdit()
        self.speed_display.setReadOnly(True)
        self.speed_display.setStyleSheet(text_style)
        self.speed_display.setPlaceholderText("SERVO1 SPEED / ACC\\nSERVO2 SPEED / ACC\\n...")

        top_layout.addWidget(self.pos_display)
        top_layout.addWidget(self.speed_display)
        left_layout.addLayout(top_layout)

        mid_layout = QGridLayout()

        button_3d_style = """
            QPushButton {
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ffffff, stop:1 #d1d1d1);
                border: 2px solid #7a7a7a;
                border-radius: 8px;
                padding: 10px;
                font-weight: bold;
                min-width: 64px;
                min-height: 40px;
            }
            QPushButton:pressed {
                background-color: #b5b5b5;
                border-style: inset;
                padding-top: 12px;
            }
        """

        circle_button_style = button_3d_style + "QPushButton { border-radius: 32px; min-width: 64px; min-height: 64px; }"

        grip_open_layout = QVBoxLayout()
        label_grip = QLabel("GRIP OPEN/CLOSE")
        label_grip.setAlignment(Qt.AlignCenter)
        self.btn_open = QPushButton("Open")
        self.btn_close = QPushButton("Close")
        self.btn_open.setStyleSheet(button_3d_style)
        self.btn_close.setStyleSheet(button_3d_style)
        grip_open_layout.addWidget(self.btn_open)
        grip_open_layout.addWidget(label_grip)
        grip_open_layout.addWidget(self.btn_close)
        mid_layout.addLayout(grip_open_layout, 1, 0)

        self.btn_up = QPushButton("Up")
        self.btn_down = QPushButton("Down")
        self.btn_left = QPushButton("Left")
        self.btn_right = QPushButton("Right")
        for btn in [self.btn_up, self.btn_down, self.btn_left, self.btn_right]:
            btn.setStyleSheet(circle_button_style)

        mid_layout.addWidget(self.btn_up, 0, 2, Qt.AlignCenter)
        mid_layout.addWidget(self.btn_left, 1, 1, Qt.AlignCenter)
        mid_layout.addWidget(self.btn_right, 1, 3, Qt.AlignCenter)
        mid_layout.addWidget(self.btn_down, 2, 2, Qt.AlignCenter)

        grip_turn_layout = QVBoxLayout()
        label_turn = QLabel("GRIP TURN LEFT/\\nRIGHT")
        label_turn.setAlignment(Qt.AlignCenter)
        self.btn_t_right = QPushButton("Right")
        self.btn_t_left = QPushButton("Left")
        self.btn_t_right.setStyleSheet(button_3d_style)
        self.btn_t_left.setStyleSheet(button_3d_style)
        grip_turn_layout.addWidget(self.btn_t_right)
        grip_turn_layout.addWidget(label_turn)
        grip_turn_layout.addWidget(self.btn_t_left)
        mid_layout.addLayout(grip_turn_layout, 1, 4)

        arm_size_layout = QVBoxLayout()
        label_size = QLabel("ARM SIZE")
        label_size.setAlignment(Qt.AlignCenter)
        self.btn_stretch = QPushButton("Stretch")
        self.btn_shrink = QPushButton("Shrink")
        self.btn_stretch.setStyleSheet(button_3d_style)
        self.btn_shrink.setStyleSheet(button_3d_style)
        arm_size_layout.addWidget(self.btn_stretch)
        arm_size_layout.addWidget(label_size)
        arm_size_layout.addWidget(self.btn_shrink)
        mid_layout.addLayout(arm_size_layout, 1, 2)

        left_layout.addLayout(mid_layout)

        speed_layout = QHBoxLayout()
        speed_layout.addStretch()
        self.speed_label = QLabel('Speed')
        self.speed_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.speed_value_label = QLabel('')
        self.speed_value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(SPEED_SLIDER_MIN)
        self.speed_slider.setMaximum(SPEED_SLIDER_MAX)
        self.speed_slider.setFixedWidth(260)
        self.speed_slider.setValue(self._duration_to_speed_value(self.move_duration_ms))
        self._refresh_speed_labels()
        speed_layout.addWidget(self.speed_label)
        speed_layout.addSpacing(8)
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addSpacing(8)
        speed_layout.addWidget(self.speed_value_label)
        speed_layout.addStretch()
        left_layout.addLayout(speed_layout)

        bottom_layout = QHBoxLayout()
        self.btn_onoff = QPushButton("ON/OFF")
        self.btn_home = QPushButton("HOME")
        self.btn_refresh = QPushButton("REFRESH")
        self.btn_face_detection = QPushButton("FACE DETECTION OFF")
        self.btn_mode = QPushButton("MODE: GUI")

        self.btn_onoff.setStyleSheet(button_3d_style + "QPushButton { background-color: #ffcccc; }")
        self.btn_home.setStyleSheet(button_3d_style)
        self.btn_refresh.setStyleSheet(button_3d_style)
        self.btn_face_detection.setStyleSheet(button_3d_style)
        self.btn_mode.setStyleSheet(button_3d_style)

        bottom_layout.addStretch()
        bottom_layout.addWidget(self.btn_onoff)
        bottom_layout.addWidget(self.btn_home)
        bottom_layout.addWidget(self.btn_refresh)
        bottom_layout.addWidget(self.btn_face_detection)
        bottom_layout.addWidget(self.btn_mode)
        bottom_layout.addStretch()
        left_layout.addLayout(bottom_layout)

        right_layout = QVBoxLayout()
        right_layout.setAlignment(Qt.AlignTop)

        camera_title = QLabel('Camera Preview')
        camera_title.setAlignment(Qt.AlignCenter)
        camera_title.setStyleSheet('font-weight: bold; padding-bottom: 4px;')

        self.preview_label = QLabel("Waiting for camera stream")
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setFixedSize(360, 270)
        self.preview_label.setStyleSheet(
            'background-color: #101010; color: #f0f0f0; border: 2px solid #7a7a7a;'
        )

        self.status_label = QLabel("Bridge status: waiting for data")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setWordWrap(True)
        self.status_label.setFixedWidth(360)

        self.error_label = QLabel("")
        self.error_label.setAlignment(Qt.AlignCenter)
        self.error_label.setWordWrap(True)
        self.error_label.setFixedWidth(360)
        self.error_label.setStyleSheet("color: #d9534f;")

        text_output_title = QLabel('Text Output')
        text_output_title.setAlignment(Qt.AlignCenter)
        text_output_title.setStyleSheet('font-weight: bold; padding-top: 10px; padding-bottom: 4px;')

        self.text_output_display = QTextEdit()
        self.text_output_display.setReadOnly(True)
        self.text_output_display.setFixedWidth(360)
        self.text_output_display.setMinimumHeight(220)
        self.text_output_display.setStyleSheet(text_style)

        right_layout.addWidget(camera_title)
        right_layout.addWidget(self.preview_label, alignment=Qt.AlignCenter)
        right_layout.addWidget(self.status_label)
        right_layout.addWidget(self.error_label)
        right_layout.addWidget(text_output_title)
        right_layout.addWidget(self.text_output_display)
        right_layout.addStretch()

        main_layout.addLayout(left_layout, 1)
        main_layout.addSpacing(12)
        main_layout.addLayout(right_layout, 0)

        self._connect_signals()
        self._refresh_displays()

    def _connect_signals(self) -> None:
        self.btn_open.clicked.connect(lambda: self._send_action('grip_open'))
        self.btn_close.clicked.connect(lambda: self._send_action('grip_close'))
        self._bind_hold_repeat(self.btn_up, 'move_up')
        self._bind_hold_repeat(self.btn_down, 'move_down')
        self._bind_hold_repeat(self.btn_left, 'move_left')
        self._bind_hold_repeat(self.btn_right, 'move_right')
        self._bind_hold_repeat(self.btn_t_left, 'turn_left')
        self._bind_hold_repeat(self.btn_t_right, 'turn_right')
        self._bind_hold_repeat(self.btn_stretch, 'arm_stretch')
        self._bind_hold_repeat(self.btn_shrink, 'arm_shrink')
        self.btn_home.clicked.connect(lambda: self._send_action('home'))
        self.btn_refresh.clicked.connect(lambda: self._send_action('refresh'))
        self.btn_onoff.clicked.connect(self._toggle_power)
        self.btn_face_detection.clicked.connect(self._toggle_face_detection)
        self.btn_mode.clicked.connect(self._cycle_control_mode)
        self.speed_slider.valueChanged.connect(self._handle_speed_slider_changed)

    def _bind_hold_repeat(self, button: QPushButton, action: str) -> None:
        button.pressed.connect(lambda act=action: self._start_hold_action(act))
        button.released.connect(self._stop_hold_action)

    def _start_hold_action(self, action: str) -> None:
        self._held_action = action
        self._send_action(action)
        self._hold_repeat_timer.start(self._hold_repeat_initial_delay_ms)

    def _repeat_held_action(self) -> None:
        if not self._held_action:
            return
        self._send_action(self._held_action)
        self._hold_repeat_timer.start(self._hold_repeat_interval_ms)

    def _stop_hold_action(self) -> None:
        self._hold_repeat_timer.stop()
        self._held_action = None

    def _duration_to_speed_value(self, duration_ms: int) -> int:
        clamped = max(MOVE_DURATION_MIN_MS, min(int(duration_ms), MOVE_DURATION_MAX_MS))
        ratio = (clamped - MOVE_DURATION_MIN_MS) / float(MOVE_DURATION_MAX_MS - MOVE_DURATION_MIN_MS)
        speed = SPEED_SLIDER_MAX - int(round(ratio * (SPEED_SLIDER_MAX - SPEED_SLIDER_MIN)))
        return max(SPEED_SLIDER_MIN, min(speed, SPEED_SLIDER_MAX))

    def _speed_value_to_duration(self, speed_value: int) -> int:
        clamped = max(SPEED_SLIDER_MIN, min(int(speed_value), SPEED_SLIDER_MAX))
        ratio = (SPEED_SLIDER_MAX - clamped) / float(SPEED_SLIDER_MAX - SPEED_SLIDER_MIN)
        duration = MOVE_DURATION_MIN_MS + int(round(ratio * (MOVE_DURATION_MAX_MS - MOVE_DURATION_MIN_MS)))
        return max(MOVE_DURATION_MIN_MS, min(duration, MOVE_DURATION_MAX_MS))

    def _refresh_speed_labels(self) -> None:
        speed_value = self.speed_slider.value()
        self.speed_value_label.setText(f"{speed_value}% ({self.move_duration_ms} ms)")

    def _handle_speed_slider_changed(self, speed_value: int) -> None:
        duration_ms = self._speed_value_to_duration(speed_value)
        if duration_ms == self.move_duration_ms:
            self._refresh_speed_labels()
            return

        self.move_duration_ms = duration_ms
        self._refresh_speed_labels()
        # Use refresh to apply a new default motion duration without triggering movement.
        self._publish_command('refresh', duration_ms=self.move_duration_ms)

    def _cycle_control_mode(self) -> None:
        current_index = CONTROL_MODES.index(self.control_mode) if self.control_mode in CONTROL_MODES else 0
        next_mode = CONTROL_MODES[(current_index + 1) % len(CONTROL_MODES)]
        self._publish_control_mode(next_mode)
        self.control_mode = next_mode
        self._refresh_displays()

    def _toggle_power(self) -> None:
        if self.status.active:
            self._send_action('power_off')
            return
        self._send_action('power_on')

    def _toggle_face_detection(self) -> None:
        self.face_detection_enabled = not self.face_detection_enabled
        self._publish_face_detection_state(self.face_detection_enabled)
        self._refresh_displays()

    def _refresh_displays(self) -> None:
        payload = self.latest_status_payload if isinstance(self.latest_status_payload, dict) else {}
        servos = payload.get('servos', []) if isinstance(payload.get('servos', []), list) else []

        pos_lines = []
        speed_lines = []
        for index in range(1, 7):
            servo = servos[index - 1] if len(servos) >= index and isinstance(servos[index - 1], dict) else {}
            angle = servo.get('angle_deg', 0)
            raw_position = servo.get('raw_position', 0)
            speed = servo.get('estimated_speed_dps', 0)
            accel = servo.get('estimated_accel_dps2', 0)

            pos_lines.append(f"SERVO{index} {angle:>6} deg | pos {raw_position}")
            speed_lines.append(f"SERVO{index} {speed:>6} d/s | acc {accel}")

        self.pos_display.setText("\n".join(pos_lines) if pos_lines else "No servo telemetry yet")
        self.speed_display.setText("\n".join(speed_lines) if speed_lines else "No speed telemetry yet")

        status_parts = [
            f"Connected: {'yes' if self.status.connected else 'no'}",
            f"Active: {'yes' if self.status.active else 'no'}",
            f"Device: {self.status.device or 'n/a'}",
            f"State: {self.status.status_text or 'n/a'}",
        ]
        self.status_label.setText(" | ".join(status_parts))
        self.error_label.setText(self.status.last_error)

        self.btn_onoff.setText("OFF" if self.status.active else "ON")
        self.btn_face_detection.setText(
            "FACE DETECTION ON" if self.face_detection_enabled else "FACE DETECTION OFF"
        )
        self.btn_mode.setText(f"MODE: {self.control_mode}")

        face_status = self.face_summary.status
        center_x = self.face_summary.center_x
        center_y = self.face_summary.center_y
        bbox_w = self.face_summary.bbox_width
        bbox_h = self.face_summary.bbox_height

        gui_active = self.control_mode in ('GUI', 'AUTO')
        llm_active = self.control_mode in ('LLM', 'AUTO')
        last_command_source = str(payload.get('last_command_source', 'n/a'))
        last_command_action = str(payload.get('last_command_action', 'n/a'))
        bridge_duration_ms = payload.get('move_duration_ms')
        if bridge_duration_ms is not None:
            try:
                bridge_duration_int = max(MOVE_DURATION_MIN_MS, min(int(bridge_duration_ms), MOVE_DURATION_MAX_MS))
                if bridge_duration_int != self.move_duration_ms:
                    self.move_duration_ms = bridge_duration_int
                    self.speed_slider.blockSignals(True)
                    self.speed_slider.setValue(self._duration_to_speed_value(self.move_duration_ms))
                    self.speed_slider.blockSignals(False)
            except (TypeError, ValueError):
                pass
        self._refresh_speed_labels()
        last_spoken = self.last_spoken_command or 'n/a'
        last_heard = self.last_heard_speech or 'n/a'

        gui_active_label = '<span style="color:#2e7d32; font-weight:700;">ACTIVE</span>' if gui_active else '<span style="color:#c62828; font-weight:700;">INACTIVE</span>'
        llm_active_label = '<span style="color:#2e7d32; font-weight:700;">ACTIVE</span>' if llm_active else '<span style="color:#c62828; font-weight:700;">INACTIVE</span>'

        text_lines = [
            f"Control mode: {self.control_mode}",
            f"GUI active: {gui_active_label}",
            f"LLM active: {llm_active_label}",
            f"Last control source: {last_command_source}",
            f"Last control action: {last_command_action}",
            f"Last heard speech: {last_heard}",
            f"Last spoken command: {last_spoken}",
            f"Speech status: {self.speech_state}",
            f"Speech info: {self.speech_message}",
            f"GUI speed: {self.speed_slider.value()}% ({self.move_duration_ms} ms)",
            f"Face detection: {'enabled' if self.face_detection_enabled else 'disabled'}",
            f"Face status: {face_status}",
            f"Center: x={center_x if center_x is not None else 'n/a'}, y={center_y if center_y is not None else 'n/a'}",
            f"BBox: w={bbox_w if bbox_w is not None else 'n/a'}, h={bbox_h if bbox_h is not None else 'n/a'}",
            f"Last error: {self.status.last_error or 'none'}",
        ]
        text_html = '<br/>'.join(html.escape(line) if '<span' not in line else line for line in text_lines)
        self.text_output_display.setHtml(text_html)

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

    def _publish_control_mode(self, mode: str) -> None:
        message = String()
        message.data = mode
        self.mode_publisher.publish(message)

    def _handle_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            payload = {"status_text": message.data, "last_error": ""}
        self.latest_status_payload = payload if isinstance(payload, dict) else {}

        self.status = ArmStatus(
            connected=bool(payload.get("connected", False)),
            active=bool(payload.get("active", False)),
            device=str(payload.get("device", "")),
            status_text=str(payload.get("status_text", "")),
            last_error=str(payload.get("last_error", "")),
        )
        mode = str(payload.get('control_mode', self.control_mode)).upper()
        if mode in CONTROL_MODES:
            self.control_mode = mode
        self._refresh_displays()

    def _handle_camera(self, message: CompressedImage) -> None:
        image = QImage.fromData(message.data, 'JPG')
        if image.isNull():
            return

        self.last_frame = image.copy()
        pixmap = QPixmap.fromImage(self.last_frame)

        if self.face_detection_enabled:
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
        self._refresh_displays()

    def _handle_speech_input(self, message: String) -> None:
        transcript = ''
        try:
            payload = json.loads(message.data)
            if isinstance(payload, dict):
                transcript = str(payload.get('transcript', '')).strip()
        except json.JSONDecodeError:
            transcript = message.data.strip()

        if transcript:
            self.last_heard_speech = transcript
            self.last_spoken_command = transcript
            self._refresh_displays()

    def _handle_speech_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            payload = {'state': 'unknown', 'message': message.data}

        self.speech_state = str(payload.get('state', 'unknown'))
        self.speech_message = str(payload.get('message', 'n/a'))

        lowered = self.speech_message.lower()
        marker = ''
        if lowered.startswith('ignored speech without wake word: '):
            marker = self.speech_message.split(': ', 1)[1].strip()
            self.last_heard_speech = marker
        elif lowered.startswith('prompt captured: '):
            marker = self.speech_message.split(': ', 1)[1].strip()
            self.last_heard_speech = marker
            self.last_spoken_command = marker
        elif lowered.startswith('wake word heard; prompt='):
            marker = self.speech_message.split('prompt=', 1)[1].strip()
            if marker:
                self.last_heard_speech = marker
                self.last_spoken_command = marker

        self._refresh_displays()

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
        self._stop_hold_action()
        self.spin_timer.stop()
        if hasattr(self, "node") and self.node is not None:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        super().closeEvent(event)
