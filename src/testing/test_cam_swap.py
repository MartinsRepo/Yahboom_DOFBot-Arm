#!/usr/bin/env python3
import cv2

def probe_camera(idx):
    for backend in [cv2.CAP_MSMF, cv2.CAP_DSHOW, cv2.CAP_ANY]:
        cap = cv2.VideoCapture(idx, backend)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                h, w, c = frame.shape
                cap.release()
                return f"Index {idx} OPENED with backend {backend} (res: {w}x{h})"
            cap.release()
    return f"Index {idx} COULD NOT BE OPENED"

print(probe_camera(0))
print(probe_camera(1))
