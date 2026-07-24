#!/usr/bin/env python3
import time
import json
import sounddevice as sd
import numpy as np
from vosk import Model, KaldiRecognizer, SetLogLevel

SetLogLevel(0)

model_path = "/opt/ros/overlay_ws/models/vosk-model-small-de-zamia-0.3"
model = Model(model_path)

grammar = ["martin", "hoch", "runter", "links", "rechts", "vor", "zurück", "home", "nimm", "gib", "stop", "stopp", "halt", "[unk]"]
rec = KaldiRecognizer(model, 16000, json.dumps(grammar))

print("=== Starting 10-Second Speech Test with Vosk (Speak 'martin hoch' now) ===")

def callback(indata, frames, time_info, status):
    if status:
        print("Status:", status)
    data = bytes(indata)
    if rec.AcceptWaveform(data):
        res = json.loads(rec.Result())
        print("VOSK FINAL RESULT:", res)
    else:
        part = json.loads(rec.PartialResult())
        if part.get("partial"):
            print("VOSK PARTIAL:", part.get("partial"))

try:
    with sd.RawInputStream(samplerate=16000, blocksize=2000, dtype="int16", channels=1, callback=callback):
        time.sleep(10)
except Exception as e:
    print("Error:", e)

print("=== Test Complete ===")
