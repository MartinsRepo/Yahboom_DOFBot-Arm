#!/usr/bin/env python3
"""Minimal helper to nudge the Dofbot arm upward.

Uses the vendor Arm_Lib API directly (no ROS dependency).
"""

import argparse
import os
import sys
import time


DEFAULT_ARM_LIB_DIR = "/home/osboxes/Vorlagen/yahboom/Dofbot/0.py_install/Arm_Lib"
DEFAULT_DEVICE = "/dev/ttyUSB0"
DEFAULT_HOME_JOINTS = [90, 130, 0, 0, 90, 30]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move arm to a conservative up pose")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="Serial device path")
    parser.add_argument(
        "--arm-lib-dir",
        default=os.environ.get("ARM_LIB_DIR", DEFAULT_ARM_LIB_DIR),
        help="Directory containing Arm_Lib.py",
    )
    parser.add_argument(
        "--duration-ms",
        type=int,
        default=900,
        help="Servo movement duration in milliseconds",
    )
    parser.add_argument(
        "--mode",
        choices=["up", "home", "pulse", "sweep", "pwm_sweep", "buzzer", "read"],
        default="up",
        help="Test mode: up, home, pulse, sweep, pwm_sweep, buzzer, read",
    )
    parser.add_argument(
        "--no-torque-on",
        action="store_true",
        help="Do not send torque-on before bus-servo movement commands",
    )
    parser.add_argument(
        "--joints",
        nargs=6,
        type=int,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        default=[90, 80, 35, 40, 90, 30],
        help="Target joint angles (default is known workspace up pose)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print action only, do not open serial port",
    )
    return parser.parse_args()


def validate_joints(joints: list[int]) -> None:
    limits = [180, 180, 180, 180, 270, 180]
    for idx, (angle, limit) in enumerate(zip(joints, limits), start=1):
        if angle < 0 or angle > limit:
            raise ValueError(f"Joint {idx} out of range: {angle} (0..{limit})")


def main() -> int:
    args = parse_args()
    validate_joints(args.joints)

    if args.dry_run:
        planned_joints = DEFAULT_HOME_JOINTS if args.mode == "home" else args.joints
        print("DRY_RUN")
        print(f"device={args.device}")
        print(f"arm_lib_dir={args.arm_lib_dir}")
        print(f"mode={args.mode}")
        print(f"torque_on={not args.no_torque_on}")
        print(f"joints={planned_joints}")
        print(f"duration_ms={args.duration_ms}")
        return 0

    if not os.path.isdir(args.arm_lib_dir):
        print(f"Arm_Lib directory not found: {args.arm_lib_dir}", file=sys.stderr)
        return 2

    sys.path.insert(0, args.arm_lib_dir)
    try:
        from Arm_Lib import Arm_Device
    except Exception as exc:
        print(f"Failed importing Arm_Lib from {args.arm_lib_dir}: {exc}", file=sys.stderr)
        return 3

    try:
        arm = Arm_Device(args.device)
    except Exception as exc:
        print(f"Failed opening serial device {args.device}: {exc}", file=sys.stderr)
        return 4

    if args.mode == "buzzer":
        print(f"Sending buzzer command to {args.device}")
        arm.Arm_Buzzer_On(1)
        time.sleep(0.3)
        print("Buzzer command sent.")
        return 0

    if args.mode == "read":
        values: list[int | None] = []
        for idx in range(1, 7):
            try:
                values.append(arm.Arm_serial_servo_read(idx))
            except Exception:
                values.append(None)
        print(f"Servo readback: {values}")
        return 0

    if args.mode == "pwm_sweep":
        print(f"Sending PWM sweep test to {args.device}")
        for angle in (45, 135, 90):
            print(f"pwm joint1={angle}")
            arm.Arm_PWM_servo_write(1, angle)
            time.sleep(max(args.duration_ms / 1000.0, 0.5) + 0.2)
        print("PWM sweep command sent.")
        return 0

    if not args.no_torque_on:
        arm.Arm_serial_set_torque(1)
        time.sleep(0.05)

    if args.mode == "home":
        print(f"Sending home pose to {args.device}: {DEFAULT_HOME_JOINTS}")
        arm.Arm_serial_servo_write6_array(DEFAULT_HOME_JOINTS, args.duration_ms)
        time.sleep(max(args.duration_ms / 1000.0, 0.5) + 0.2)
        print("Home command sent.")
        return 0

    if args.mode == "pulse":
        down_pose = [90, 130, 0, 0, 90, 30]
        print(f"Sending pulse test to {args.device}")
        print(f"step1 down={down_pose}")
        arm.Arm_serial_servo_write6_array(down_pose, args.duration_ms)
        time.sleep(max(args.duration_ms / 1000.0, 0.5) + 0.3)
        print(f"step2 up={args.joints}")
        arm.Arm_serial_servo_write6_array(args.joints, args.duration_ms)
        time.sleep(max(args.duration_ms / 1000.0, 0.5) + 0.3)
        print("Pulse command sent.")
        return 0

    if args.mode == "sweep":
        print(f"Sending sweep test to {args.device}")
        for angle in (45, 135, 90):
            print(f"joint1={angle}")
            arm.Arm_serial_servo_write(1, angle, args.duration_ms)
            time.sleep(max(args.duration_ms / 1000.0, 0.5) + 0.3)
        print("Sweep command sent.")
        return 0

    print(f"Sending up-pose command to {args.device}: {args.joints}")
    arm.Arm_serial_servo_write6_array(args.joints, args.duration_ms)
    time.sleep(max(args.duration_ms / 1000.0, 0.5) + 0.2)
    print("Command sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
