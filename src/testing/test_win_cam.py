#!/usr/bin/env python3
"""
Test script to probe available camera indices on Windows using OpenCV (DSHOW & MSMF backends).
"""

import cv2

def main():
    print("=== Probing Windows Cameras via OpenCV (DSHOW) ===")
    for idx in range(4):
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        opened = cap.isOpened()
        ret, frame = (False, None)
        if opened:
            ret, frame = cap.read()
        print(f"Index {idx} (DSHOW): opened={opened}, read={ret}, shape={frame.shape if ret else None}")
        cap.release()

    print("\n=== Probing Windows Cameras via OpenCV (MSMF) ===")
    for idx in range(4):
        cap = cv2.VideoCapture(idx, cv2.CAP_MSMF)
        opened = cap.isOpened()
        ret, frame = (False, None)
        if opened:
            ret, frame = cap.read()
        print(f"Index {idx} (MSMF): opened={opened}, read={ret}, shape={frame.shape if ret else None}")
        cap.release()

if __name__ == "__main__":
    main()
