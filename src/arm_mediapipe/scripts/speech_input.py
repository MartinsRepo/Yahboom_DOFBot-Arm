#!/usr/bin/env python3
"""Capture microphone speech with Vosk and publish transcripts for the LLM controller."""

from __future__ import annotations

import json
import os
import queue
import re
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import sounddevice as sd
except Exception as exc:  # pragma: no cover - depends on container runtime
    sd = None
    SOUNDDEVICE_IMPORT_ERROR = exc
else:
    SOUNDDEVICE_IMPORT_ERROR = None

try:
    from vosk import KaldiRecognizer, Model, SetLogLevel
    SetLogLevel(-1)
except Exception as exc:  # pragma: no cover - depends on installed package
    KaldiRecognizer = None
    Model = None
    SetLogLevel = None
    VOSK_IMPORT_ERROR = exc
else:
    VOSK_IMPORT_ERROR = None


DEFAULT_MODEL_DIR = "/opt/ros/overlay_ws/models/vosk-model-small-de-zamia-0.3"
DEFAULT_SAMPLE_RATE = 16000
DEFAULT_BLOCKSIZE = 2000
DEFAULT_TOPIC = "roboarm/speech_input"
DEFAULT_WAKE_WORD_EN = "hello"
DEFAULT_WAKE_WORD_DE = "hallo"
DEFAULT_FLUSH_SILENCE_S = 0.4
DEFAULT_QUEUE_SIZE = 128


class SpeechInputNode(Node):
    def __init__(self) -> None:
        super().__init__("speech_input")

        self.enabled = os.environ.get("ENABLE_SPEECH_CONTROLLER", "0").strip().lower() in ("1", "true", "yes", "on")
        requested_model_dir = os.environ.get("DOFBOT_VOSK_MODEL_DIR", DEFAULT_MODEL_DIR).strip()
        self.model_dir = self._resolve_model_dir(requested_model_dir)
        self.sample_rate = max(8000, int(float(os.environ.get("DOFBOT_SPEECH_SAMPLE_RATE", str(DEFAULT_SAMPLE_RATE)))))
        self.blocksize = max(400, int(float(os.environ.get("DOFBOT_SPEECH_BLOCKSIZE", str(DEFAULT_BLOCKSIZE)))))
        self.device = os.environ.get("DOFBOT_SPEECH_DEVICE", "pulse").strip()
        self.transcript_topic = os.environ.get("DOFBOT_SPEECH_TOPIC", DEFAULT_TOPIC).strip() or DEFAULT_TOPIC
        model_name = os.path.basename(self.model_dir).lower()
        default_language = "de" if "-de" in model_name or "german" in model_name else "en"
        self.language = os.environ.get("DOFBOT_SPEECH_LANGUAGE", default_language).strip().lower() or default_language
        self.flush_silence_s = max(
            0.1,
            float(os.environ.get("DOFBOT_SPEECH_FLUSH_SILENCE_S", str(DEFAULT_FLUSH_SILENCE_S))),
        )
        self.max_queue_size = max(
            1,
            int(float(os.environ.get("DOFBOT_SPEECH_QUEUE_SIZE", str(DEFAULT_QUEUE_SIZE)))),
        )
        configured_wake_word = os.environ.get("DOFBOT_WAKE_WORD", "").strip().lower()
        if configured_wake_word:
            self.wake_word = configured_wake_word
        elif self.language.startswith("de"):
            self.wake_word = DEFAULT_WAKE_WORD_DE
        else:
            self.wake_word = DEFAULT_WAKE_WORD_EN
        self.wake_word_timeout_s = max(0.5, float(os.environ.get("DOFBOT_WAKE_WORD_TIMEOUT_S", "4.0")))
        if self.device.lower() in {"default", "auto"}:
            self.device = ""

        self.audio_queue: queue.Queue[bytes] = queue.Queue(maxsize=self.max_queue_size)
        self.transcript_publisher = self.create_publisher(String, self.transcript_topic, 10)
        self.status_publisher = self.create_publisher(String, "roboarm/speech_status", 10)

        self.model: Optional[Model] = None
        self.recognizer: Optional[KaldiRecognizer] = None
        self.stream = None
        self.worker_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.last_audio_time_s = 0.0
        self.awaiting_prompt_until_s = 0.0
        self.wake_word_armed = False
        self.voice_command_mode_active = False

        self._publish_status("starting", "initializing")
        if not self.enabled:
            self._publish_status("disabled", "ENABLE_SPEECH_CONTROLLER is off")
            return

        if sd is None:
            raise RuntimeError(f"sounddevice import failed: {SOUNDDEVICE_IMPORT_ERROR}")
        if Model is None or KaldiRecognizer is None:
            raise RuntimeError(f"vosk import failed: {VOSK_IMPORT_ERROR}")
        if not os.path.isdir(self.model_dir):
            raise RuntimeError(f"Vosk model directory not found: {self.model_dir}")

        self._log_capture_devices()

        self.model = Model(self.model_dir)
        self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
        self.recognizer.SetWords(False)

        self.worker_thread = threading.Thread(target=self._run_capture_loop, daemon=True)
        self.worker_thread.start()
        self._publish_status("listening", f"model={self.model_dir} device={self.device or 'default'} rate={self.sample_rate}")
        self.get_logger().info(f"Speech input enabled with Vosk model at {self.model_dir}")

    def _publish_status(self, state: str, message: str) -> None:
        payload = {
            "state": state,
            "message": message,
            "timestamp_s": time.time(),
            "topic": self.transcript_topic,
            "language": self.language,
        }
        status = String()
        status.data = json.dumps(payload)
        self.status_publisher.publish(status)

    def _audio_callback(self, indata, frames, time_info, status) -> None:  # noqa: D401 - sounddevice callback signature
        if self.stop_event.is_set():
            return
        try:
            chunk = bytes(indata)
            self.audio_queue.put_nowait(chunk)
            self.last_audio_time_s = time.time()
        except queue.Full:
            # Drop audio instead of blocking the callback thread.
            pass

    def _emit_transcript(self, transcript: str, is_final: bool = True) -> None:
        transcript = transcript.strip()
        if not transcript:
            return

        payload = {
            "transcript": transcript,
            "final": bool(is_final),
            "timestamp_s": time.time(),
            "source": "vosk",
            "language": self.language,
        }
        message = String()
        message.data = json.dumps(payload)
        self.transcript_publisher.publish(message)
        self.get_logger().info(f"Speech transcript: {transcript}")

    def _handle_final_transcript(self, transcript: str) -> None:
        normalized = transcript.strip().lower()
        if not normalized:
            return

        now_s = time.time()

        if self.wake_word and self.wake_word in normalized:
            cleaned = normalized.replace(self.wake_word, "", 1).strip(" ,.!?;:")
            self.voice_command_mode_active = True
            self.wake_word_armed = False
            self.awaiting_prompt_until_s = 0.0

            if cleaned:
                if self._is_stop_phrase(cleaned):
                    self._emit_transcript("stop", is_final=True)
                    self.voice_command_mode_active = False
                    self._publish_status("listening", "voice command mode disabled by stop")
                    return
                self._emit_transcript(cleaned, is_final=True)
                self._publish_status("listening", f"voice command mode active; prompt={cleaned}")
                return

            self._publish_status("armed", "voice command mode enabled")
            return

        if self.voice_command_mode_active:
            if self._is_stop_phrase(normalized):
                self._emit_transcript("stop", is_final=True)
                self.voice_command_mode_active = False
                self._publish_status("listening", "voice command mode disabled by stop")
                return

            self._emit_transcript(normalized, is_final=True)
            self._publish_status("listening", f"voice command mode active; prompt={normalized}")
            return

        if self.wake_word_armed and now_s <= self.awaiting_prompt_until_s:
            self.wake_word_armed = False
            self.awaiting_prompt_until_s = 0.0
            self._emit_transcript(normalized, is_final=True)
            self._publish_status("listening", f"prompt captured: {normalized}")
            return

        self._publish_status("listening", f"ignored speech without wake word: {transcript}")

    def _is_stop_phrase(self, text: str) -> bool:
        normalized = str(text).strip().lower()
        stop_phrases = {
            "stop",
            "stopp",
            "anhalten",
            "halt",
            "pause",
        }
        return normalized in stop_phrases

    def _drain_final_result(self) -> None:
        if self.recognizer is None:
            return
        try:
            result = json.loads(self.recognizer.FinalResult())
        except json.JSONDecodeError:
            return
        transcript = str(result.get("text", "")).strip()
        if transcript:
            self._handle_final_transcript(transcript)

    def _run_capture_loop(self) -> None:
        stream_device = self._resolve_input_device(self.device)
        try:
            with sd.RawInputStream(
                samplerate=self.sample_rate,
                blocksize=self.blocksize,
                device=stream_device,
                dtype="int16",
                channels=1,
                callback=self._audio_callback,
            ):
                self._publish_status("listening", f"capturing audio from {stream_device or 'default device'}")
                silence_deadline = 0.0
                while not self.stop_event.is_set():
                    try:
                        data = self.audio_queue.get(timeout=0.2)
                    except queue.Empty:
                        if self.last_audio_time_s and silence_deadline and time.time() > silence_deadline:
                            self._drain_final_result()
                            silence_deadline = 0.0
                        continue

                    if self.recognizer is None:
                        continue

                    if self.recognizer.AcceptWaveform(data):
                        try:
                            result = json.loads(self.recognizer.Result())
                        except json.JSONDecodeError:
                            continue
                        transcript = str(result.get("text", "")).strip()
                        if transcript:
                            self._handle_final_transcript(transcript)
                        silence_deadline = 0.0
                    else:
                        silence_deadline = time.time() + self.flush_silence_s
        except Exception as exc:  # pragma: no cover - device/runtime specific
            self._publish_status("error", str(exc))
            self.get_logger().error(f"Speech capture failed: {exc}")
        finally:
            self._drain_final_result()
            self._publish_status("stopped", "speech capture loop ended")

    def _log_capture_devices(self) -> None:
        try:
            devices = sd.query_devices()
        except Exception as exc:  # pragma: no cover - runtime dependent
            self.get_logger().warning(f"Unable to enumerate capture devices: {exc}")
            self._publish_status("warning", f"Unable to enumerate capture devices: {exc}")
            return

        capture_devices = [
            (index, dev) for index, dev in enumerate(devices)
            if int(dev.get("max_input_channels", 0)) > 0
        ]

        if not capture_devices:
            self.get_logger().warning("No capture devices detected by PortAudio")
            self._publish_status("warning", "No capture devices detected by PortAudio")
            return

        self.get_logger().info("Available speech capture devices:")
        summary_parts = []
        for index, dev in capture_devices:
            name = str(dev.get("name", "unknown"))
            channels = int(dev.get("max_input_channels", 0))
            rate = int(float(dev.get("default_samplerate", 0.0)))
            self.get_logger().info(
                f"  input_index={index} | channels={channels} | rate={rate} | name={name}"
            )
            summary_parts.append(f"#{index} {name}")

        summary = "; ".join(summary_parts[:8])
        if len(summary_parts) > 8:
            summary = f"{summary}; ..."
        self._publish_status("listening", f"capture devices: {summary}")

    def _resolve_model_dir(self, requested: str) -> str:
        requested = requested.strip()
        if requested and os.path.isdir(requested):
            return requested

        candidates = [
            "/opt/ros/overlay_ws/models/vosk-model-small-de-zamia-0.3",
            "/opt/ros/overlay_ws/models/vosk-model-small-en-us-0.15",
            "/opt/ros/overlay_ws/models/vosk-model-en-us-0.22",
            "/home/martin/workspace/Flowisetest/Yahboom_DOFBot-Arm/models/vosk-model-small-de-zamia-0.3",
            "/home/martin/workspace/Flowisetest/Yahboom_DOFBot-Arm/models/vosk-model-small-en-us-0.15",
        ]

        models_root = "/opt/ros/overlay_ws/models"
        if os.path.isdir(models_root):
            try:
                dynamic_candidates = [
                    os.path.join(models_root, entry)
                    for entry in sorted(os.listdir(models_root))
                    if entry.startswith("vosk-model")
                ]
                candidates.extend(dynamic_candidates)
            except OSError:
                pass

        for candidate in candidates:
            if candidate and os.path.isdir(candidate):
                self.get_logger().warning(
                    f"Requested Vosk model directory '{requested}' not found; using '{candidate}'"
                )
                return candidate

        return requested

    def _resolve_input_device(self, requested: str):
        if requested.strip() == "":
            return None

        if requested.isdigit():
            return int(requested)

        try:
            devices = sd.query_devices()
        except Exception as exc:  # pragma: no cover - runtime dependent
            self.get_logger().warning(f"Unable to list audio devices: {exc}")
            return requested

        capture_devices = [
            (index, dev) for index, dev in enumerate(devices)
            if int(dev.get("max_input_channels", 0)) > 0
        ]

        # Accept ALSA-like hints such as "4,0" or "hw:4,0" by matching substrings in device names.
        alsa_match = re.search(r"(\d+)\s*,\s*(\d+)", requested)
        if alsa_match:
            card_id = alsa_match.group(1)
            sub_id = alsa_match.group(2)
            patterns = [f"hw:{card_id},{sub_id}", f"{card_id},{sub_id}"]
            for index, dev in capture_devices:
                dev_name = str(dev.get("name", "")).lower()
                if any(pattern in dev_name for pattern in patterns):
                    self.get_logger().info(f"Selected input device #{index}: {dev.get('name', 'unknown')}")
                    return index

        requested_lower = requested.lower()
        for index, dev in capture_devices:
            dev_name = str(dev.get("name", ""))
            if requested_lower in dev_name.lower():
                self.get_logger().info(f"Selected input device #{index}: {dev_name}")
                return index

        self.get_logger().warning(
            f"Requested speech device '{requested}' not found by name; using raw value"
        )
        return requested

    def stop(self) -> None:
        self.stop_event.set()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SpeechInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
