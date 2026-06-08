#!/usr/bin/env python3
"""Optimized Vosk microphone test for faster, lower-latency speech recognition."""

from __future__ import annotations

import argparse
import json
import os
import queue
import re
import sys
import time
from pathlib import Path

try:
    import sounddevice as sd
except Exception as exc:
    print(f"[ERROR] sounddevice import failed: {exc}", file=sys.stderr)
    raise SystemExit(1)

try:
    from vosk import KaldiRecognizer, Model, SetLogLevel
except Exception as exc:
    print(f"[ERROR] vosk import failed: {exc}", file=sys.stderr)
    raise

# OPTIMIZED DEFAULTS FOR ROBOTIC / REAL-TIME CONTROL
DEFAULT_MODEL_DIR = "/home/martin/workspace/Flowisetest/Yahboom_DOFBot-Arm/models/vosk-model-small-de-zamia-0.3"
DEFAULT_SAMPLE_RATE = 16000
DEFAULT_BLOCKSIZE = 2000          # Smaller block size = more frequent updates, lower latency
DEFAULT_FLUSH_SILENCE_S = 0.4     # Cut down from 0.8s for faster command finalization
DEFAULT_QUEUE_SIZE = 128          # Larger queue ceiling to prevent dropped frames


def resolve_model_dir(requested: str) -> str:
    requested = requested.strip()
    if requested and os.path.isdir(requested):
        return requested
    candidates = [
        "/home/martin/workspace/Flowisetest/Yahboom_DOFBot-Arm/models/vosk-model-small-en-us-0.15",
        "/home/martin/workspace/Flowisetest/Yahboom_DOFBot-Arm/models/vosk-model-small-de-zamia-0.3",
    ]
    for candidate in candidates:
        if candidate and os.path.isdir(candidate):
            return candidate
    return requested


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Optimized Vosk microphone test",
        epilog=(
            "Example:\n"
            "  python vosk_terminal_test.py --device pulse --sample-rate 16000"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--model", default=os.environ.get("DOFBOT_VOSK_MODEL_DIR", DEFAULT_MODEL_DIR))
    parser.add_argument("--device", default=os.environ.get("DOFBOT_SPEECH_DEVICE", "pulse")) # Default to pulse
    parser.add_argument("--sample-rate", type=int, default=DEFAULT_SAMPLE_RATE)
    parser.add_argument("--blocksize", type=int, default=DEFAULT_BLOCKSIZE)
    parser.add_argument("--flush-silence-s", type=float, default=DEFAULT_FLUSH_SILENCE_S)
    parser.add_argument("--show-partials", action="store_true", help="Print partials (increases CPU usage)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    SetLogLevel(-1) # Suppress Kaldi C++ spam

    model_dir = resolve_model_dir(args.model)
    if not Path(model_dir).is_dir():
        print(f"[ERROR] Vosk model directory not found: {model_dir}", file=sys.stderr)
        return 1

    print(f"[INFO] Fast-detect enabled. Silence timeout: {args.flush_silence_s}s | Blocksize: {args.blocksize}")
    print(f"[INFO] Model: {model_dir}")
    print(f"[INFO] Device: {args.device}")

    model = Model(model_dir)
    recognizer = KaldiRecognizer(model, args.sample_rate)
    recognizer.SetWords(False)  # Turn off per-word timestamps to save processing overhead

    audio_queue: queue.Queue[bytes] = queue.Queue(maxsize=DEFAULT_QUEUE_SIZE)
    
    def audio_callback(indata, frames, time_info, status):
        try:
            audio_queue.put_nowait(bytes(indata))
        except queue.Full:
            pass

    try:
        with sd.RawInputStream(
            samplerate=args.sample_rate,
            blocksize=args.blocksize,
            device=args.device if args.device != "default" else None,
            dtype="int16",
            channels=1,
            callback=audio_callback,
        ):
            silence_start = None
            
            while True:
                try:
                    # Snappy queue check to keep the loop spinning fast
                    data = audio_queue.get_nowait()
                    silence_start = None 
                except queue.Empty:
                    # Handle trailing silence to finalize speech quickly
                    if silence_start and (time.time() - silence_start > args.flush_silence_s):
                        res = json.loads(recognizer.FinalResult())
                        text = res.get("text", "").strip()
                        if text:
                            print(f"FINAL: {text}", flush=True)
                        silence_start = None
                    time.sleep(0.01)
                    continue

                if recognizer.AcceptWaveform(data):
                    res = json.loads(recognizer.Result())
                    text = res.get("text", "").strip()
                    if text:
                        print(f"FINAL: {text}", flush=True)
                    silence_start = None
                else:
                    if args.show_partials:
                        res = json.loads(recognizer.PartialResult())
                        part = res.get("partial", "").strip()
                        if part:
                            print(f"PARTIAL: {part}", flush=True)
                    
                    if silence_start is None:
                        silence_start = time.time()

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
        return 0
    except Exception as exc:
        print(f"[ERROR] Run failed: {exc}", file=sys.stderr)
        return 1

if __name__ == "__main__":
    raise SystemExit(main())