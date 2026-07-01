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
DEFAULT_MODEL_DIR = "/home/martin/workspace/AG-Projects/Yahboom_DOFBot-Arm/models/vosk-model-small-de-zamia-0.3"

# Import the grammar lists from the main speech_input module path so the
# terminal test can exercise the same restricted vocabulary.
_SCRIPTS_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, "arm_mediapipe", "scripts"
)
if os.path.isdir(_SCRIPTS_DIR) and _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

try:
    from speech_input import COMMAND_GRAMMAR_DE, COMMAND_GRAMMAR_EN
except ImportError:
    COMMAND_GRAMMAR_DE = None
    COMMAND_GRAMMAR_EN = None
DEFAULT_SAMPLE_RATE = 16000
DEFAULT_BLOCKSIZE = 2000          # Smaller block size = more frequent updates, lower latency
DEFAULT_FLUSH_SILENCE_S = 0.4     # Cut down from 0.8s for faster command finalization
DEFAULT_QUEUE_SIZE = 128          # Larger queue ceiling to prevent dropped frames


def normalize_device_name(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", "", text.lower())


def resolve_input_device(requested: str):
    requested = requested.strip()
    if requested == "":
        return None

    if requested.isdigit():
        return int(requested)

    requested_normalized = normalize_device_name(requested)
    if requested_normalized in {"", "default", "auto"}:
        return None

    try:
        devices = sd.query_devices()
    except Exception:
        return None

    capture_devices = [
        (index, dev)
        for index, dev in enumerate(devices)
        if int(dev.get("max_input_channels", 0)) > 0
    ]

    requested_lower = requested.lower()
    requested_tokens = [token for token in re.split(r"[^a-z0-9]+", requested_lower) if token]

    for index, dev in capture_devices:
        dev_name = str(dev.get("name", ""))
        dev_name_lower = dev_name.lower()
        dev_name_normalized = normalize_device_name(dev_name)
        dev_tokens = {token for token in re.split(r"[^a-z0-9]+", dev_name_lower) if token}

        if requested_lower in dev_name_lower:
            print(f"[INFO] Selected input device #{index}: {dev_name}")
            return index

        if requested_normalized and requested_normalized == dev_name_normalized:
            print(f"[INFO] Selected input device #{index}: {dev_name}")
            return index

        if requested_normalized and requested_normalized in dev_name_normalized:
            print(f"[INFO] Selected input device #{index}: {dev_name}")
            return index

        if requested_tokens and all(token in dev_tokens for token in requested_tokens):
            print(f"[INFO] Selected input device #{index}: {dev_name}")
            return index

    print(f"[WARN] Requested speech device '{requested}' not found by name; using default input device")
    return None


def resolve_model_dir(requested: str) -> str:
    requested = requested.strip()
    if requested and os.path.isdir(requested):
        return requested
    candidates = [
        "/home/martin/workspace/AG-Projects/Yahboom_DOFBot-Arm/models/vosk-model-small-en-us-0.15",
        "/home/martin/workspace/AG-Projects/Yahboom_DOFBot-Arm/models/vosk-model-small-de-zamia-0.3",
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
    parser.add_argument("--list-devices", action="store_true", help="List capture devices and exit")
    parser.add_argument(
        "--grammar",
        action="store_true",
        help="Restrict recognizer to known command vocabulary (grammar mode)",
    )
    parser.add_argument(
        "--language",
        default="de",
        choices=["de", "en"],
        help="Language for grammar mode (default: de)",
    )
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
    grammar_entries = None
    if args.grammar:
        if args.language.startswith("de") and COMMAND_GRAMMAR_DE:
            grammar_entries = COMMAND_GRAMMAR_DE
        elif COMMAND_GRAMMAR_EN:
            grammar_entries = COMMAND_GRAMMAR_EN
        else:
            print("[WARN] Grammar lists not available; running in open vocabulary mode")

    if grammar_entries:
        grammar_json = json.dumps(grammar_entries)
        recognizer = KaldiRecognizer(model, args.sample_rate, grammar_json)
        print(f"[INFO] Grammar mode: {len(grammar_entries)} entries for '{args.language}'")
    else:
        recognizer = KaldiRecognizer(model, args.sample_rate)
        print("[INFO] Open vocabulary mode (no grammar restriction)")
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
            device=resolve_input_device(args.device),
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