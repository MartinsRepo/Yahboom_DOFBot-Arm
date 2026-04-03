#!/usr/bin/env python3
"""
Quick camera test to verify ARM camera is working
"""
import cv2
import sys

def test_camera(device_id=0):
    """Test camera capture and display"""
    print(f"Attempting to open camera at /dev/video{device_id}...")
    cap = cv2.VideoCapture(device_id)
    
    if not cap.isOpened():
        print(f"Failed to open /dev/video{device_id}")
        return False
    
    # Get camera properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"✓ Camera opened successfully!")
    print(f"  Resolution: {width}x{height}")
    print(f"  FPS: {fps}")
    
    # Try to capture a few frames
    print("\nCapturing frames...")
    for i in range(5):
        ret, frame = cap.read()
        if ret:
            print(f"  Frame {i+1}: OK ({frame.shape})")
        else:
            print(f"  Frame {i+1}: FAILED")
            cap.release()
            return False
    
    cap.release()
    print("\n✓ Camera test passed!")
    return True

if __name__ == "__main__":
    device = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    test_camera(device)
