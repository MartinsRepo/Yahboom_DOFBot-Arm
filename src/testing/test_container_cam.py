#!/usr/bin/env python3
"""
Test script to verify camera access and shared volume frame reading inside WSL/Podman container.
"""

import os
import cv2

def main():
    print("=== Testing Direct V4L2 Devices ===")
    for dev_path in ["/dev/video0", "/dev/video1", "/dev/video2"]:
        if os.path.exists(dev_path):
            cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
            opened = cap.isOpened()
            ret, frame = (False, None)
            if opened:
                ret, frame = cap.read()
            print(f"Device {dev_path}: opened={opened}, read={ret}, shape={frame.shape if ret else None}")
            cap.release()
        else:
            print(f"Device {dev_path}: not present")

    print("\n=== Testing Shared Volume Frame Stream ===")
    shared_path = "/opt/ros/overlay_ws/runtime_log/latest_frame.jpg"
    print(f"Shared frame path exists: {os.path.exists(shared_path)}")
    if os.path.exists(shared_path):
        img = cv2.imread(shared_path)
        print("Successfully read image inside container! Shape:", img.shape if img is not None else None)

if __name__ == "__main__":
    main()
