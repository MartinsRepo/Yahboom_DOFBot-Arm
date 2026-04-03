#!/usr/bin/env python3
"""ROS 2 bridge between RoboControl UI and the vendor arm controller."""

import argparse
import json
import math
import os
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


DEFAULT_ARM_LIB_DIR = "/home/osboxes/Vorlagen/yahboom/Dofbot/0.py_install/Arm_Lib"
DEFAULT_DEVICE = "/dev/ttyUSB0"
DEFAULT_HOME_JOINTS = [90, 130, 0, 0, 90, 30]
DEFAULT_MOVE_DURATION_MS = 500
DEFAULT_POLL_PERIOD = 0.25


class RoboArmBridge(Node):
    def __init__(self, arm_lib_dir: str, device: str, move_duration_ms: int, poll_period: float):
        super().__init__("roboarm_bridge")

        self.arm_lib_dir = arm_lib_dir
        self.device = device
        self.move_duration_ms = max(100, int(move_duration_ms))
        self.poll_period = max(0.05, float(poll_period))

        self.joint_names = [
            "base",
            "shoulder",
            "elbow",
            "wrist_pitch",
            "wrist_rotate",
            "gripper",
        ]
        self.joint_limits = [180, 180, 180, 180, 270, 180]
        self.current_joints = list(DEFAULT_HOME_JOINTS)
        self.last_deltas = [0.0] * 6
        self.last_speeds = [0.0] * 6
        self.last_accels = [0.0] * 6

        self.arm_device_cls = None
        self.arm = None
        self.connected = False
        self.active = False
        self.last_error = ""
        self.status_text = "Starting"

        self.status_publisher = self.create_publisher(String, "roboarm/status", 10)
        self.joint_state_publisher = self.create_publisher(JointState, "roboarm/joint_states", 10)
        self.command_subscription = self.create_subscription(
            String,
            "roboarm/command",
            self.handle_command,
            10,
        )

        self._load_arm_library()
        self._connect_arm()
        self.publish_state()
        self.create_timer(self.poll_period, self.poll_and_publish)

    def _load_arm_library(self) -> None:
        if not os.path.isdir(self.arm_lib_dir):
            self.last_error = f"Arm_Lib directory not found: {self.arm_lib_dir}"
            self.status_text = "Arm_Lib missing"
            self.get_logger().error(self.last_error)
            return

        if self.arm_lib_dir not in sys.path:
            sys.path.insert(0, self.arm_lib_dir)

        try:
            from Arm_Lib import Arm_Device  # pylint: disable=import-error
        except Exception as exc:  # pragma: no cover - import failure depends on host setup
            self.last_error = f"Failed importing Arm_Lib: {exc}"
            self.status_text = "Arm_Lib import failed"
            self.get_logger().error(self.last_error)
            return

        self.arm_device_cls = Arm_Device

    def _connect_arm(self) -> bool:
        if self.arm_device_cls is None:
            return False

        try:
            self.arm = self.arm_device_cls(self.device)
            self.connected = True
            self.active = True
            self.last_error = ""
            self.status_text = "Connected"
            self._enable_servo_bus()
            self._read_current_joints()
            self.get_logger().info(f"Connected to arm on {self.device}")
            return True
        except Exception as exc:  # pragma: no cover - hardware access depends on host setup
            self.arm = None
            self.connected = False
            self.active = False
            self.last_error = f"Failed opening {self.device}: {exc}"
            self.status_text = "Disconnected"
            self.get_logger().error(self.last_error)
            return False

    def _enable_servo_bus(self) -> None:
        if self.arm is None:
            return

        try:
            self.arm.Arm_serial_set_torque(1)
        except Exception:
            pass

    def _read_current_joints(self) -> None:
        if self.arm is None:
            return

        updated_joints = list(self.current_joints)
        for index in range(6):
            try:
                value = self.arm.Arm_serial_servo_read(index + 1)
            except Exception:
                value = None

            if value is not None:
                updated_joints[index] = float(value)

        self.current_joints = updated_joints

    def poll_and_publish(self) -> None:
        if self.connected:
            self._read_current_joints()
        self.publish_state()

    def publish_state(self) -> None:
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self.joint_names)
        joint_state.position = [math.radians(angle) for angle in self.current_joints]
        joint_state.velocity = [math.radians(speed) for speed in self.last_speeds]
        joint_state.effort = []
        self.joint_state_publisher.publish(joint_state)

        status_message = String()
        status_message.data = json.dumps(self._build_status_payload())
        self.status_publisher.publish(status_message)

    def _build_status_payload(self) -> dict:
        return {
            "connected": self.connected,
            "active": self.active,
            "device": self.device,
            "move_duration_ms": self.move_duration_ms,
            "status_text": self.status_text,
            "last_error": self.last_error,
            "servos": [
                {
                    "id": servo_id,
                    "name": self.joint_names[servo_id - 1],
                    "angle_deg": round(self.current_joints[servo_id - 1], 1),
                    "raw_position": self._angle_to_raw_position(servo_id, self.current_joints[servo_id - 1]),
                    "estimated_speed_dps": round(self.last_speeds[servo_id - 1], 2),
                    "estimated_accel_dps2": round(self.last_accels[servo_id - 1], 2),
                    "delta_deg": round(self.last_deltas[servo_id - 1], 1),
                }
                for servo_id in range(1, 7)
            ],
        }

    def _angle_to_raw_position(self, servo_id: int, angle: float) -> int:
        if servo_id == 5:
            bounded = self._clamp(angle, 0, 270)
            return int((3700 - 380) * bounded / 270 + 380)

        bounded = self._clamp(angle, 0, 180)
        if servo_id in (2, 3, 4):
            bounded = 180 - bounded
        return int((3100 - 900) * bounded / 180 + 900)

    def handle_command(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self.last_error = f"Invalid command payload: {message.data}"
            self.status_text = "Invalid command"
            self.publish_state()
            return

        duration_ms = payload.get("duration_ms")
        if duration_ms is not None:
            self.move_duration_ms = max(100, min(int(duration_ms), 5000))

        action = payload.get("action", "")

        if action == "toggle_active":
            self.toggle_active()
        elif action == "power_on":
            self.set_active(True)
        elif action == "power_off":
            self.set_active(False)
        elif action == "home":
            self.move_home()
        elif action == "move_left":
            self.step_joint(1, 5)
        elif action == "move_right":
            self.step_joint(1, -5)
        elif action == "move_up":
            self.step_joint(2, 5)
        elif action == "move_down":
            self.step_joint(2, -5)
        elif action == "turn_left":
            self.step_joint(5, -8)
        elif action == "turn_right":
            self.step_joint(5, 8)
        elif action == "arm_stretch":
            self.arm_stretch()
        elif action == "arm_shrink":
            self.arm_shrink()
        elif action == "grip_open":
            self.move_joint_to(6, 30)
        elif action == "grip_close":
            self.move_joint_to(6, 140)
        elif action == "refresh":
            self._read_current_joints()
            self.status_text = "Refreshed"
        else:
            self.last_error = f"Unsupported action: {action}"
            self.status_text = "Unsupported action"

        self.publish_state()

    def toggle_active(self) -> None:
        if not self.connected and not self._connect_arm():
            return
        self.active = not self.active
        self.status_text = "Active" if self.active else "Inactive"

    def set_active(self, active: bool) -> None:
        if active and not self.connected and not self._connect_arm():
            return

        self.active = active and self.connected
        self.status_text = "Active" if self.active else "Inactive"

    def move_home(self) -> None:
        if not self._ensure_motion_allowed("home"):
            return

        target_joints = list(DEFAULT_HOME_JOINTS)
        deltas = [target - current for target, current in zip(target_joints, self.current_joints)]
        try:
            self.arm.Arm_serial_servo_write6_array(target_joints, self.move_duration_ms)
            self.current_joints = [float(value) for value in target_joints]
            self._record_motion(deltas)
            self.last_error = ""
            self.status_text = "Home command sent"
        except Exception as exc:  # pragma: no cover - hardware access depends on host setup
            self.last_error = f"Home command failed: {exc}"
            self.status_text = "Home failed"

    def step_joint(self, servo_id: int, delta: float) -> None:
        if not self._ensure_motion_allowed(f"step joint {servo_id}"):
            return

        current = self.current_joints[servo_id - 1]
        target = self._clamp(current + delta, 0, self.joint_limits[servo_id - 1])
        self.move_joint_to(servo_id, target)

    def move_joint_to(self, servo_id: int, target_angle: float) -> None:
        if not self._ensure_motion_allowed(f"move joint {servo_id}"):
            return

        bounded = self._clamp(target_angle, 0, self.joint_limits[servo_id - 1])
        current = self.current_joints[servo_id - 1]
        delta = bounded - current
        try:
            self.arm.Arm_serial_servo_write(servo_id, int(round(bounded)), self.move_duration_ms)
            self.current_joints[servo_id - 1] = float(bounded)
            deltas = [0.0] * 6
            deltas[servo_id - 1] = delta
            self._record_motion(deltas)
            self.last_error = ""
            self.status_text = f"Moved {self.joint_names[servo_id - 1]}"
        except Exception as exc:  # pragma: no cover - hardware access depends on host setup
            self.last_error = f"Move failed for servo {servo_id}: {exc}"
            self.status_text = "Move failed"

    def arm_stretch(self) -> None:
        if not self._ensure_motion_allowed("stretch arm"):
            return

        targets = {
            2: self._clamp(self.current_joints[1] - 5, 0, self.joint_limits[1]),
            3: self._clamp(self.current_joints[2] + 6, 0, self.joint_limits[2]),
            4: self._clamp(self.current_joints[3] + 6, 0, self.joint_limits[3]),
        }
        self._apply_multi_joint_targets(targets, "Arm stretched")

    def arm_shrink(self) -> None:
        if not self._ensure_motion_allowed("shrink arm"):
            return

        targets = {
            2: self._clamp(self.current_joints[1] + 5, 0, self.joint_limits[1]),
            3: self._clamp(self.current_joints[2] - 6, 0, self.joint_limits[2]),
            4: self._clamp(self.current_joints[3] - 6, 0, self.joint_limits[3]),
        }
        self._apply_multi_joint_targets(targets, "Arm shrunk")

    def _apply_multi_joint_targets(self, targets: dict[int, float], success_status: str) -> None:
        if self.arm is None:
            self.last_error = "Arm handle unavailable"
            self.status_text = "Unavailable"
            return

        deltas = [0.0] * 6
        try:
            for servo_id, target in targets.items():
                current = self.current_joints[servo_id - 1]
                deltas[servo_id - 1] = target - current
                self.arm.Arm_serial_servo_write(servo_id, int(round(target)), self.move_duration_ms)
                self.current_joints[servo_id - 1] = float(target)
            self._record_motion(deltas)
            self.last_error = ""
            self.status_text = success_status
        except Exception as exc:  # pragma: no cover - hardware access depends on host setup
            self.last_error = f"Multi-joint move failed: {exc}"
            self.status_text = "Move failed"

    def _record_motion(self, deltas: list[float]) -> None:
        duration_s = max(self.move_duration_ms / 1000.0, 0.001)
        self.last_deltas = [float(delta) for delta in deltas]
        self.last_speeds = [abs(delta) / duration_s for delta in deltas]
        self.last_accels = [speed / duration_s for speed in self.last_speeds]

    def _ensure_motion_allowed(self, reason: str) -> bool:
        if not self.connected:
            self.last_error = f"Cannot {reason}: arm is not connected"
            self.status_text = "Disconnected"
            return False

        if not self.active:
            self.last_error = f"Cannot {reason}: controller is inactive"
            self.status_text = "Inactive"
            return False

        if self.arm is None:
            self.last_error = f"Cannot {reason}: arm handle unavailable"
            self.status_text = "Unavailable"
            return False

        return True

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(value, upper))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RoboControl ROS 2 bridge")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="Serial device path")
    parser.add_argument(
        "--arm-lib-dir",
        default=os.environ.get("ARM_LIB_DIR", DEFAULT_ARM_LIB_DIR),
        help="Directory containing Arm_Lib.py",
    )
    parser.add_argument(
        "--move-duration-ms",
        type=int,
        default=DEFAULT_MOVE_DURATION_MS,
        help="Default movement duration in milliseconds",
    )
    parser.add_argument(
        "--poll-period",
        type=float,
        default=DEFAULT_POLL_PERIOD,
        help="Servo polling interval in seconds",
    )
    return parser.parse_args()


def main(args: Optional[list[str]] = None) -> None:
    cli_args = parse_args() if args is None else parse_args()
    rclpy.init(args=args)
    node = RoboArmBridge(
        arm_lib_dir=cli_args.arm_lib_dir,
        device=cli_args.device,
        move_duration_ms=cli_args.move_duration_ms,
        poll_period=cli_args.poll_period,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()