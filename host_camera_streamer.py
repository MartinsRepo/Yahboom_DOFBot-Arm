#!/usr/bin/env python3
"""
Host USB Camera Streamer for Yahboom DOFBot Arm on Windows.
Captures Windows USB Camera (Arm Camera) & PC Webcam frames and saves to shared volume for Podman container.
- Arm Camera  -> docker_data/log/latest_frame.jpg
- PC Webcam   -> docker_data/log/webcam_frame.jpg
"""

import os
import sys
import time
import cv2

LOG_DIR = os.path.join(os.path.dirname(__file__), "docker_data", "log")
os.makedirs(LOG_DIR, exist_ok=True)

ARM_FRAME_PATH = os.path.join(LOG_DIR, "latest_frame.jpg")
ARM_TMP_PATH = os.path.join(LOG_DIR, "tmp_latest_frame.jpg")

WEBCAM_FRAME_PATH = os.path.join(LOG_DIR, "webcam_frame.jpg")
WEBCAM_TMP_PATH = os.path.join(LOG_DIR, "tmp_webcam_frame.jpg")

# Configurable camera indices (Arm Camera = Index 0, Laptop Webcam = Index 1)
ARM_CAM_INDEX = int(os.environ.get("DOFBOT_ARM_CAM_INDEX", "0"))
WEBCAM_INDEX = int(os.environ.get("DOFBOT_WEBCAM_INDEX", "1"))


def open_camera_by_index(preferred_idx: int):
    for backend in [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]:
        try:
            cap = cv2.VideoCapture(preferred_idx, backend)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    return cap
                cap.release()
        except Exception:
            pass
    return None


def write_frame_atomically(frame, tmp_path, target_path):
    ok, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if ok:
        try:
            with open(tmp_path, 'wb') as f:
                f.write(jpeg.tobytes())
            os.replace(tmp_path, target_path)
        except Exception:
            pass


def main():
    print(f"[Host Camera Streamer] Initializing Arm Camera (Index {ARM_CAM_INDEX}) and Laptop Webcam (Index {WEBCAM_INDEX})...")
    arm_cap = open_camera_by_index(ARM_CAM_INDEX)
    webcam_cap = open_camera_by_index(WEBCAM_INDEX)

    if arm_cap is None:
        print(f"[Host Camera Streamer] Notice: Arm Camera (Index {ARM_CAM_INDEX}) not found; attempting index 1.")
        arm_cap = open_camera_by_index(1)

    if webcam_cap is None:
        print(f"[Host Camera Streamer] Notice: Webcam (Index {WEBCAM_INDEX}) not found; falling back to Arm Camera.")
        webcam_cap = arm_cap

    if arm_cap is not None:
        print(f"[Host Camera Streamer] Successfully connected Arm Camera (Index {ARM_CAM_INDEX}).")
    if webcam_cap is not None:
        print(f"[Host Camera Streamer] Successfully connected Laptop Webcam (Index {WEBCAM_INDEX}).")

    count = 0
    try:
        while True:
            # Capture Arm Camera -> latest_frame.jpg
            if arm_cap is not None:
                ret_arm, frame_arm = arm_cap.read()
                if ret_arm and frame_arm is not None:
                    write_frame_atomically(frame_arm, ARM_TMP_PATH, ARM_FRAME_PATH)

            # Capture Laptop Webcam -> webcam_frame.jpg
            if webcam_cap is not None:
                ret_web, frame_web = webcam_cap.read()
                if ret_web and frame_web is not None:
                    write_frame_atomically(frame_web, WEBCAM_TMP_PATH, WEBCAM_FRAME_PATH)

            count += 1
            if count % 300 == 0:
                print(f"[Host Camera Streamer] Streamed {count} frames successfully.")

            time.sleep(0.033)  # ~30 FPS
    except KeyboardInterrupt:
        print("\n[Host Camera Streamer] Stopped.")
    finally:
        if arm_cap is not None:
            arm_cap.release()
        if webcam_cap is not None and webcam_cap != arm_cap:
            webcam_cap.release()


if __name__ == '__main__':
    main()
