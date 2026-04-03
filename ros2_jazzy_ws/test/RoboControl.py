import json
import os
import sys

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS_AVAILABLE = True
    ROS_IMPORT_ERROR = ""
except Exception as exc:
    rclpy = None
    Node = None
    String = None
    ROS_AVAILABLE = False
    ROS_IMPORT_ERROR = str(exc)

from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, QTextEdit,
                             QLabel, QGridLayout, QVBoxLayout, QHBoxLayout)
from PyQt5.QtCore import Qt, QTimer


DEFAULT_ARM_LIB_DIR = "/home/osboxes/Vorlagen/yahboom/Dofbot/0.py_install/Arm_Lib"
DEFAULT_DEVICE = "/dev/ttyUSB0"
DEFAULT_HOME_JOINTS = [90, 130, 0, 0, 90, 30]
DEFAULT_MOVE_DURATION_MS = 500


class LocalArmBackend:
    def __init__(self, arm_lib_dir=DEFAULT_ARM_LIB_DIR, device=DEFAULT_DEVICE, move_duration_ms=DEFAULT_MOVE_DURATION_MS):
        self.arm_lib_dir = arm_lib_dir
        self.device = device
        self.move_duration_ms = move_duration_ms
        self.joint_names = [
            'base',
            'shoulder',
            'elbow',
            'wrist_pitch',
            'wrist_rotate',
            'gripper',
        ]
        self.joint_limits = [180, 180, 180, 180, 270, 180]
        self.current_joints = [float(value) for value in DEFAULT_HOME_JOINTS]
        self.last_deltas = [0.0] * 6
        self.last_speeds = [0.0] * 6
        self.last_accels = [0.0] * 6
        self.last_error = ''
        self.status_text = 'Direct mode starting'
        self.connected = False
        self.active = False
        self.arm = None
        self._load_and_connect()

    def _load_and_connect(self):
        if not os.path.isdir(self.arm_lib_dir):
            self.last_error = f'Arm_Lib directory not found: {self.arm_lib_dir}'
            self.status_text = 'Arm_Lib missing'
            return

        if self.arm_lib_dir not in sys.path:
            sys.path.insert(0, self.arm_lib_dir)

        try:
            from Arm_Lib import Arm_Device
        except Exception as exc:
            self.last_error = f'Failed importing Arm_Lib: {exc}'
            self.status_text = 'Arm_Lib import failed'
            return

        try:
            self.arm = Arm_Device(self.device)
            self.connected = True
            self.active = True
            self.status_text = 'Direct arm connected'
            self.last_error = ''
            self._enable_servo_bus()
            self.refresh()
        except Exception as exc:
            self.arm = None
            self.connected = False
            self.active = False
            self.last_error = f'Failed opening {self.device}: {exc}'
            self.status_text = 'Direct mode disconnected'

    def _enable_servo_bus(self):
        if self.arm is None:
            return
        try:
            self.arm.Arm_serial_set_torque(1)
        except Exception:
            pass

    def refresh(self):
        if self.arm is None:
            return self.get_status()

        updated_joints = list(self.current_joints)
        for index in range(6):
            try:
                value = self.arm.Arm_serial_servo_read(index + 1)
            except Exception:
                value = None
            if value is not None:
                updated_joints[index] = float(value)
        self.current_joints = updated_joints
        if self.connected:
            self.status_text = 'Direct mode active' if self.active else 'Direct mode inactive'
        return self.get_status()

    def send_action(self, action):
        if action == 'toggle_active':
            if not self.connected and self.arm is None:
                self._load_and_connect()
            self.active = self.connected and not self.active
            self.status_text = 'Direct mode active' if self.active else 'Direct mode inactive'
            return self.get_status()

        if action == 'refresh':
            return self.refresh()

        if action == 'home':
            return self._move_home()
        if action == 'move_left':
            return self._step_joint(1, 5)
        if action == 'move_right':
            return self._step_joint(1, -5)
        if action == 'move_up':
            return self._step_joint(2, 5)
        if action == 'move_down':
            return self._step_joint(2, -5)
        if action == 'turn_left':
            return self._step_joint(5, -8)
        if action == 'turn_right':
            return self._step_joint(5, 8)
        if action == 'arm_stretch':
            return self._arm_stretch()
        if action == 'arm_shrink':
            return self._arm_shrink()
        if action == 'grip_open':
            return self._move_joint_to(6, 30)
        if action == 'grip_close':
            return self._move_joint_to(6, 140)

        self.last_error = f'Unsupported action: {action}'
        self.status_text = 'Unsupported action'
        return self.get_status()

    def _move_home(self):
        if not self._ensure_motion_allowed('home'):
            return self.get_status()

        target_joints = [float(value) for value in DEFAULT_HOME_JOINTS]
        deltas = [target - current for target, current in zip(target_joints, self.current_joints)]
        try:
            self.arm.Arm_serial_servo_write6_array([int(round(value)) for value in target_joints], self.move_duration_ms)
            self.current_joints = target_joints
            self._record_motion(deltas)
            self.last_error = ''
            self.status_text = 'Home command sent'
        except Exception as exc:
            self.last_error = f'Home command failed: {exc}'
            self.status_text = 'Home failed'
        return self.get_status()

    def _step_joint(self, servo_id, delta):
        if not self._ensure_motion_allowed(f'step joint {servo_id}'):
            return self.get_status()
        current = self.current_joints[servo_id - 1]
        target = self._clamp(current + delta, 0, self.joint_limits[servo_id - 1])
        return self._move_joint_to(servo_id, target)

    def _move_joint_to(self, servo_id, target_angle):
        if not self._ensure_motion_allowed(f'move joint {servo_id}'):
            return self.get_status()
        bounded = self._clamp(target_angle, 0, self.joint_limits[servo_id - 1])
        delta = bounded - self.current_joints[servo_id - 1]
        try:
            self.arm.Arm_serial_servo_write(servo_id, int(round(bounded)), self.move_duration_ms)
            self.current_joints[servo_id - 1] = float(bounded)
            deltas = [0.0] * 6
            deltas[servo_id - 1] = delta
            self._record_motion(deltas)
            self.last_error = ''
            self.status_text = f'Moved {self.joint_names[servo_id - 1]}'
        except Exception as exc:
            self.last_error = f'Move failed for servo {servo_id}: {exc}'
            self.status_text = 'Move failed'
        return self.get_status()

    def _arm_stretch(self):
        if not self._ensure_motion_allowed('stretch arm'):
            return self.get_status()
        # Stretch forward by opening shoulder/elbow geometry incrementally.
        targets = {
            2: self._clamp(self.current_joints[1] - 5, 0, self.joint_limits[1]),
            3: self._clamp(self.current_joints[2] + 6, 0, self.joint_limits[2]),
            4: self._clamp(self.current_joints[3] + 6, 0, self.joint_limits[3]),
        }
        return self._apply_multi_joint_targets(targets, 'Arm stretched')

    def _arm_shrink(self):
        if not self._ensure_motion_allowed('shrink arm'):
            return self.get_status()
        # Shrink back by folding shoulder/elbow geometry incrementally.
        targets = {
            2: self._clamp(self.current_joints[1] + 5, 0, self.joint_limits[1]),
            3: self._clamp(self.current_joints[2] - 6, 0, self.joint_limits[2]),
            4: self._clamp(self.current_joints[3] - 6, 0, self.joint_limits[3]),
        }
        return self._apply_multi_joint_targets(targets, 'Arm shrunk')

    def _apply_multi_joint_targets(self, targets, success_status):
        deltas = [0.0] * 6
        try:
            for servo_id, target in targets.items():
                current = self.current_joints[servo_id - 1]
                deltas[servo_id - 1] = target - current
                self.arm.Arm_serial_servo_write(servo_id, int(round(target)), self.move_duration_ms)
                self.current_joints[servo_id - 1] = float(target)
            self._record_motion(deltas)
            self.last_error = ''
            self.status_text = success_status
        except Exception as exc:
            self.last_error = f'Multi-joint move failed: {exc}'
            self.status_text = 'Move failed'
        return self.get_status()

    def _record_motion(self, deltas):
        duration_s = max(self.move_duration_ms / 1000.0, 0.001)
        self.last_deltas = [float(delta) for delta in deltas]
        self.last_speeds = [abs(delta) / duration_s for delta in deltas]
        self.last_accels = [speed / duration_s for speed in self.last_speeds]

    def _ensure_motion_allowed(self, reason):
        if not self.connected or self.arm is None:
            self.last_error = f'Cannot {reason}: arm is not connected'
            self.status_text = 'Direct mode disconnected'
            return False
        if not self.active:
            self.last_error = f'Cannot {reason}: controller is inactive'
            self.status_text = 'Direct mode inactive'
            return False
        return True

    def _angle_to_raw_position(self, servo_id, angle):
        bounded = self._clamp(angle, 0, self.joint_limits[servo_id - 1])
        if servo_id == 5:
            return int((3700 - 380) * bounded / 270 + 380)
        if servo_id in (2, 3, 4):
            bounded = 180 - bounded
        return int((3100 - 900) * bounded / 180 + 900)

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(value, upper))

    def get_status(self):
        return {
            'connected': self.connected,
            'active': self.active,
            'device': self.device,
            'move_duration_ms': self.move_duration_ms,
            'status_text': self.status_text,
            'last_error': self.last_error,
            'transport': 'direct',
            'servos': [
                {
                    'id': servo_id,
                    'name': self.joint_names[servo_id - 1],
                    'angle_deg': round(self.current_joints[servo_id - 1], 1),
                    'raw_position': self._angle_to_raw_position(servo_id, self.current_joints[servo_id - 1]),
                    'estimated_speed_dps': round(self.last_speeds[servo_id - 1], 2),
                    'estimated_accel_dps2': round(self.last_accels[servo_id - 1], 2),
                    'delta_deg': round(self.last_deltas[servo_id - 1], 1),
                }
                for servo_id in range(1, 7)
            ],
        }

class RoboArmController(QWidget):
    def __init__(self):
        super().__init__()
        self.using_ros = ROS_AVAILABLE
        self.node = None
        self.command_publisher = None
        self.status_subscription = None
        self.local_backend = None

        if self.using_ros:
            if not rclpy.ok():
                rclpy.init(args=None)

            self.node = Node('roboarm_controller_ui')
            self.command_publisher = self.node.create_publisher(String, 'roboarm/command', 10)
            self.status_subscription = self.node.create_subscription(
                String,
                'roboarm/status',
                self._handle_status_message,
                10,
            )
        else:
            self.local_backend = LocalArmBackend()

        self.latest_status = self._empty_status()
        self.initUI()
        self.ros_timer = QTimer(self)
        if self.using_ros:
            self.ros_timer.timeout.connect(self._spin_ros_once)
            self.ros_timer.start(50)
        else:
            self.ros_timer.timeout.connect(self._poll_local_backend)
            self.ros_timer.start(250)

        self._send_command('refresh')

    @staticmethod
    def _empty_status():
        return {
            'connected': False,
            'active': False,
            'device': '/dev/ttyUSB0',
            'move_duration_ms': 500,
            'status_text': 'Waiting for bridge',
            'last_error': '',
            'transport': 'ros2' if ROS_AVAILABLE else 'direct',
            'servos': [
                {
                    'id': index,
                    'name': f'servo_{index}',
                    'angle_deg': 0.0,
                    'raw_position': 0,
                    'estimated_speed_dps': 0.0,
                    'estimated_accel_dps2': 0.0,
                    'delta_deg': 0.0,
                }
                for index in range(1, 7)
            ],
        }

    def initUI(self):
        self.setWindowTitle('Roboarm Controller')
        self.setGeometry(100, 100, 600, 600)
        self.setFixedWidth(600)
        self.setStyleSheet("background-color: #f0f0f0;")

        main_layout = QVBoxLayout()

        top_layout = QHBoxLayout()

        text_style = "background-color: white; border: 2px solid black; font-family: monospace;"

        self.pos_display = QTextEdit()
        self.pos_display.setReadOnly(True)
        self.pos_display.setMinimumWidth(0)
        self.pos_display.setStyleSheet(text_style)
        self.pos_display.setPlaceholderText("SERVO1 ANGLE / POS\nSERVO2 ANGLE / POS\n...")

        self.speed_display = QTextEdit()
        self.speed_display.setReadOnly(True)
        self.speed_display.setMinimumWidth(0)
        self.speed_display.setStyleSheet(text_style)
        self.speed_display.setPlaceholderText("SERVO1 SPEED / ACC\nSERVO2 SPEED / ACC\n...")

        top_layout.addWidget(self.pos_display)
        top_layout.addWidget(self.speed_display)
        main_layout.addLayout(top_layout)

        self.status_label = QLabel('Bridge status: waiting for data')
        self.status_label.setStyleSheet('font-weight: bold; padding: 4px 0;')
        main_layout.addWidget(self.status_label)

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
        
        circle_button_style = button_3d_style.replace("border-radius: 8px;", "border-radius: 32px; min-width: 64px; min-height: 64px;")

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
        label_turn = QLabel("GRIP TURN LEFT/\nRIGHT")
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

        main_layout.addLayout(mid_layout)

        bottom_layout = QHBoxLayout()
        self.btn_onoff = QPushButton("ON/OFF")
        self.btn_home = QPushButton("HOME")

        self.btn_onoff.setStyleSheet(button_3d_style + "background-color: #ffcccc;")
        self.btn_home.setStyleSheet(button_3d_style)

        bottom_layout.addStretch()
        bottom_layout.addWidget(self.btn_onoff)
        bottom_layout.addWidget(self.btn_home)
        bottom_layout.addStretch()

        main_layout.addLayout(bottom_layout)

        self.setLayout(main_layout)
        self._connect_signals()
        self._refresh_displays()

    def _connect_signals(self):
        self.btn_open.clicked.connect(lambda: self._send_command('grip_open'))
        self.btn_close.clicked.connect(lambda: self._send_command('grip_close'))
        self.btn_up.clicked.connect(lambda: self._send_command('move_up'))
        self.btn_down.clicked.connect(lambda: self._send_command('move_down'))
        self.btn_left.clicked.connect(lambda: self._send_command('move_left'))
        self.btn_right.clicked.connect(lambda: self._send_command('move_right'))
        self.btn_t_left.clicked.connect(lambda: self._send_command('turn_left'))
        self.btn_t_right.clicked.connect(lambda: self._send_command('turn_right'))
        self.btn_stretch.clicked.connect(lambda: self._send_command('arm_stretch'))
        self.btn_shrink.clicked.connect(lambda: self._send_command('arm_shrink'))
        self.btn_home.clicked.connect(lambda: self._send_command('home'))
        self.btn_onoff.clicked.connect(lambda: self._send_command('toggle_active'))

    def _spin_ros_once(self):
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except RuntimeError:
            pass

    def _poll_local_backend(self):
        if self.local_backend is None:
            return
        self.latest_status = self.local_backend.refresh()
        self._refresh_displays()

    def _send_command(self, action, **extra_fields):
        if self.using_ros:
            message = String()
            payload = {'action': action}
            payload.update(extra_fields)
            message.data = json.dumps(payload)
            self.command_publisher.publish(message)
            return

        if self.local_backend is not None:
            self.latest_status = self.local_backend.send_action(action)
            self._refresh_displays()

    def _handle_status_message(self, message):
        try:
            self.latest_status = json.loads(message.data)
        except json.JSONDecodeError:
            self.latest_status = self._empty_status()
            self.latest_status['last_error'] = f'Invalid status payload: {message.data}'
        self._refresh_displays()

    def _refresh_displays(self):
        servos = self.latest_status.get('servos', [])
        if not servos:
            servos = self._empty_status()['servos']

        pos_lines = []
        speed_lines = []
        for servo in servos:
            pos_lines.append(
                f"SERVO{servo['id']} {servo['name']:<12} angle={servo['angle_deg']:>6.1f} deg   pos={servo['raw_position']:>4}"
            )
            speed_lines.append(
                f"SERVO{servo['id']} speed={servo['estimated_speed_dps']:>6.2f} deg/s   acc={servo['estimated_accel_dps2']:>7.2f} deg/s^2"
            )

        self.pos_display.setPlainText('\n'.join(pos_lines))
        self.speed_display.setPlainText('\n'.join(speed_lines))

        status_text = self.latest_status.get('status_text', 'Unknown')
        device = self.latest_status.get('device', '/dev/ttyUSB0')
        move_duration_ms = self.latest_status.get('move_duration_ms', 500)
        connected = self.latest_status.get('connected', False)
        active = self.latest_status.get('active', False)
        last_error = self.latest_status.get('last_error', '')
        transport = self.latest_status.get('transport', 'ros2' if self.using_ros else 'direct')
        state_text = 'active' if active else 'inactive'
        connection_text = 'connected' if connected else 'disconnected'
        summary = f"Mode {transport} | status: {status_text} | {connection_text} | {state_text} | device {device} | duration {move_duration_ms} ms"
        if not self.using_ros and ROS_IMPORT_ERROR:
            summary = summary + " | ros bridge unavailable"
        if last_error:
            summary = summary + f" | error: {last_error}"
        self.status_label.setText(summary)

        if connected and active:
            self.btn_onoff.setText('TURN OFF')
        else:
            self.btn_onoff.setText('TURN ON')

    def closeEvent(self, event):
        if hasattr(self, 'ros_timer'):
            self.ros_timer.stop()
        if self.using_ros and hasattr(self, 'node') and self.node is not None:
            self.node.destroy_node()
        if self.using_ros and rclpy.ok():
            rclpy.shutdown()
        super().closeEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RoboArmController()
    ex.show()
    sys.exit(app.exec_())