#!/usr/bin/env python3
import time
import sounddevice as sd
import numpy as np

print("=== Sounddevice Query Devices ===")
print(sd.query_devices())

print("\n=== Default Input Device ===")
try:
    print(sd.query_devices(kind='input'))
except Exception as e:
    print("Error querying input device:", e)

def audio_callback(indata, frames, time_info, status):
    if status:
        print("Status:", status)
    volume_norm = np.linalg.norm(indata) * 10
    print(f"Audio chunk received | RMS volume: {volume_norm:.2f}")

print("\n=== Testing Audio Capture (3 seconds at 16000Hz) ===")
try:
    with sd.InputStream(samplerate=16000, channels=1, callback=audio_callback):
        time.sleep(3)
    print("Capturing 16000Hz succeeded!")
except Exception as e:
    print("Failed capturing at 16000Hz:", e)

print("\n=== Testing Audio Capture (3 seconds at 44100Hz) ===")
try:
    with sd.InputStream(samplerate=44100, channels=1, callback=audio_callback):
        time.sleep(3)
    print("Capturing 44100Hz succeeded!")
except Exception as e:
    print("Failed capturing at 44100Hz:", e)
