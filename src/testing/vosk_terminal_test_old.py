#!/usr/bin/env python3
"""Standalone Vosk microphone test that prints recognized speech to the terminal."""

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
except Exception as exc:  # pragma: no cover - depends on runtime
    print(f"[ERROR] sounddevice import failed: {exc}", file=sys.stderr)
    print(
        "[HINT] On Ubuntu/Pop!_OS, install PortAudio runtime: sudo apt update && sudo apt install -y libportaudio2",
        file=sys.stderr,
    )
    print(
        "[HINT] If you build audio Python wheels locally, also install: sudo apt install -y portaudio19-dev",
        file=sys.stderr,
    )
    raise SystemExit(1)

try:
    from vosk import KaldiRecognizer, Model, SetLogLevel
except Exception as exc:  # pragma: no cover - depends on runtime
    print(f"[ERROR] vosk import failed: {exc}", file=sys.stderr)
    raise


DEFAULT_MODEL_DIR = "/home/martin/workspace/Flowisetest/Yahboom_DOFBot-Arm/models/vosk-model-en-us-0.22"
DEFAULT_SAMPLE_RATE = 16000
DEFAULT_BLOCKSIZE = 8000
DEFAULT_FLUSH_SILENCE_S = 0.8
DEFAULT_QUEUE_SIZE = 64


def resolve_model_dir(requested: str) -> str:
    requested = requested.strip()
    if requested and os.path.isdir(requested):
        return requested

    candidates = [
        "/home/martin/workspace/Flowisetest/Yahboom_DOFBot-Arm/models/vosk-model-en-us-0.22",
        "/home/martin/workspace/Flowisetest/Yahboom_DOFBot-Arm/models/vosk-model-small-en-us-0.15",
    ]

    models_root = "/home/martin/workspace/Flowisetest/Yahboom_DOFBot-Arm/models"
    if os.path.isdir(models_root):
        try:
            dynamic_candidates = [
                os.path.join(models_root, entry)
                for entry in sorted(os.listdir(models_root))
                if entry.startswith("vosk-model") and "en-us" in entry
            ]
            candidates.extend(dynamic_candidates)
        except OSError:
            pass

    for candidate in candidates:
        if candidate and os.path.isdir(candidate):
            print(
                f"[WARN] Requested Vosk model '{requested}' not found; using '{candidate}'",
                file=sys.stderr,
            )
            return candidate

    return requested


def list_capture_devices() -> list[tuple[int, dict]]:
    devices = sd.query_devices()
    return [
        (index, dev)
        for index, dev in enumerate(devices)
        if int(dev.get("max_input_channels", 0)) > 0
    ]


def resolve_input_device(requested: str):
    requested = requested.strip()
    if requested == "" or requested.lower() in {"default", "auto"}:
        return None

    if requested.isdigit():
        return int(requested)

    capture_devices = list_capture_devices()

    alsa_match = re.search(r"(\d+)\s*,\s*(\d+)", requested)
    if alsa_match:
        card_id = alsa_match.group(1)
        sub_id = alsa_match.group(2)
        patterns = [f"hw:{card_id},{sub_id}", f"{card_id},{sub_id}"]
        for index, dev in capture_devices:
            dev_name = str(dev.get("name", "")).lower()
            if any(pattern in dev_name for pattern in patterns):
                return index

    requested_lower = requested.lower()
    for index, dev in capture_devices:
        dev_name = str(dev.get("name", ""))
        if requested_lower in dev_name.lower():
            return index

    return requested


def print_capture_devices() -> None:
    try:
        capture_devices = list_capture_devices()
    except Exception as exc:
        print(f"[WARN] Unable to enumerate capture devices: {exc}", file=sys.stderr)
        return

    if not capture_devices:
        print("[WARN] No audio capture devices detected by PortAudio", file=sys.stderr)
        return

    print("[INFO] Available audio capture devices:")
    for index, dev in capture_devices:
        name = str(dev.get("name", "unknown"))
        channels = int(dev.get("max_input_channels", 0))
        rate = int(float(dev.get("default_samplerate", 0.0)))
        print(f"  #{index}: channels={channels} rate={rate} name={name}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Vosk microphone test that prints speech transcripts")
    parser.add_argument(
        "--model",
        default=os.environ.get("DOFBOT_VOSK_MODEL_DIR", DEFAULT_MODEL_DIR),
        help="Path to Vosk model directory",
    )
    parser.add_argument(
        "--device",
        default=os.environ.get("DOFBOT_SPEECH_DEVICE", "default"),
        help="Input device selector: default, index, name substring, or ALSA hint (e.g. 4,0)",
    )
    parser.add_argument(
        "--sample-rate",
        type=int,
        default=int(float(os.environ.get("DOFBOT_SPEECH_SAMPLE_RATE", str(DEFAULT_SAMPLE_RATE)))),
        help="Input sample rate",
    )
    parser.add_argument(
        "--blocksize",
        type=int,
        default=int(float(os.environ.get("DOFBOT_SPEECH_BLOCKSIZE", str(DEFAULT_BLOCKSIZE)))),
        help="Audio blocksize in frames",
    )
    parser.add_argument(
        "--flush-silence-s",
        type=float,
        default=float(os.environ.get("DOFBOT_SPEECH_FLUSH_SILENCE_S", str(DEFAULT_FLUSH_SILENCE_S))),
        help="Seconds of silence before finalizing buffered partial speech",
    )
    parser.add_argument(
        "--show-partials",
        action="store_true",
        help="Print partial hypotheses while speaking",
    )
    parser.add_argument(
        "--list-devices",
        action="store_true",
        help="List capture devices and exit",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    SetLogLevel(-1)

    if args.list_devices:
        print_capture_devices()
        return 0

    model_dir = resolve_model_dir(args.model)
    if not Path(model_dir).is_dir():
        print(f"[ERROR] Vosk model directory not found: {model_dir}", file=sys.stderr)
        return 1

    sample_rate = max(8000, int(args.sample_rate))
    blocksize = max(400, int(args.blocksize))
    flush_silence_s = max(0.1, float(args.flush_silence_s))

    print_capture_devices()
    device = resolve_input_device(args.device)

    print(f"[INFO] Using model: {model_dir}")
    print(f"[INFO] Using device: {device if device is not None else 'default'}")
    print(f"[INFO] Sample rate: {sample_rate}")
    print(f"[INFO] Blocksize: {blocksize}")
    print("[INFO] Listening... Press Ctrl+C to stop.")

    model = Model(model_dir)
    recognizer = KaldiRecognizer(model, sample_rate)
    recognizer.SetWords(True)

    audio_queue: queue.Queue[bytes] = queue.Queue(maxsize=DEFAULT_QUEUE_SIZE)
    last_audio_time_s = 0.0

    def audio_callback(indata, frames, time_info, status):  # noqa: D401 - sounddevice callback signature
        nonlocal last_audio_time_s
        try:
            audio_queue.put_nowait(bytes(indata))
            last_audio_time_s = time.time()
        except queue.Full:
            pass

    try:
        with sd.RawInputStream(
            samplerate=sample_rate,
            blocksize=blocksize,
            device=device,
            dtype="int16",
            channels=1,
            callback=audio_callback,
        ):
            silence_deadline = 0.0
            while True:
                try:
                    data = audio_queue.get(timeout=0.2)
                except queue.Empty:
                    if last_audio_time_s and silence_deadline and time.time() > silence_deadline:
                        try:
                            final_result = json.loads(recognizer.FinalResult())
                            text = str(final_result.get("text", "")).strip()
                            if text:
                                print(f"FINAL: {text}")
                        except json.JSONDecodeError:
                            pass
                        silence_deadline = 0.0
                    continue

                if recognizer.AcceptWaveform(data):
                    try:
                        result = json.loads(recognizer.Result())
                        text = str(result.get("text", "")).strip()
                        if text:
                            print(f"FINAL: {text}")
                    except json.JSONDecodeError:
                        pass
                    silence_deadline = 0.0
                else:
                    if args.show_partials:
                        try:
                            partial = json.loads(recognizer.PartialResult())
                            text = str(partial.get("partial", "")).strip()
                            if text:
                                print(f"PARTIAL: {text}")
                        except json.JSONDecodeError:
                            pass
                    silence_deadline = time.time() + flush_silence_s
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
        return 0
    except Exception as exc:
        print(f"[ERROR] Audio capture failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
