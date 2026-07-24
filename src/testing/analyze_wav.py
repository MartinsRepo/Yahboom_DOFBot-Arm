#!/usr/bin/env python3
import os
import wave
import struct

wav_path = os.path.expandvars(r"%TEMP%\mic_test.wav")
if not os.path.exists(wav_path):
    print("WAV file not found:", wav_path)
    exit(1)

w = wave.open(wav_path, "rb")
nframes = w.getnframes()
framerate = w.getframerate()
nchannels = w.getnchannels()
sampwidth = w.getsampwidth()
frames = w.readframes(nframes)
w.close()

print(f"WAV Info: {nframes} frames | {framerate} Hz | {nchannels} ch | {sampwidth} bytes/sample")

if sampwidth == 1:
    samples = struct.unpack(f"<{len(frames)}B", frames)
    max_val = max(abs(s - 128) for s in samples)
    print(f"Max Sample Amplitude (8-bit): {max_val} (Out of 127)")
    if max_val == 0:
        print("RESULT: TOTAL SILENCE (0). The microphone hardware / Windows privacy / default device is muted or silent.")
    elif max_val < 5:
        print("RESULT: VERY LOW VOLUME (< 5). Microphone gain is too low or muted.")
    else:
        print(f"RESULT: AUDIO CAPTURED SUCCESSFULLY! Peak amplitude: {max_val}")
elif sampwidth == 2:
    samples = struct.unpack(f"<{len(frames)//2}h", frames)
    max_val = max(abs(s) for s in samples)
    print(f"Max Sample Amplitude (16-bit): {max_val} (Out of 32767)")
    if max_val == 0:
        print("RESULT: TOTAL SILENCE (0). The microphone hardware / Windows privacy / default device is muted or silent.")
    elif max_val < 500:
        print("RESULT: VERY LOW VOLUME (< 500). Microphone gain is too low or muted.")
    else:
        print(f"RESULT: AUDIO CAPTURED SUCCESSFULLY! Peak amplitude: {max_val}")
