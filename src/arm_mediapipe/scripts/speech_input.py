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


DEFAULT_MODEL_DIR = "/opt/ros/overlay_ws/models/vosk-model-small-en-us-0.15"
#DEFAULT_MODEL_DIR = "/opt/ros/overlay_ws/models/vosk-model-small-de-zamia-0.3"
DEFAULT_SAMPLE_RATE = 16000
DEFAULT_BLOCKSIZE = 8192
DEFAULT_TOPIC = "roboarm/speech_input"
DEFAULT_WAKE_WORD_EN = "robby"
DEFAULT_WAKE_WORD_DE = "martin"
DEFAULT_STOP_ALIASES = {"heute"}
DEFAULT_FLUSH_SILENCE_S = 0.8
DEFAULT_QUEUE_SIZE = 128
DEFAULT_STREAM_START_TIMEOUT_S = 4.0
DEFAULT_STREAM_RETRY_DELAY_S = 1.0
CONTROL_MODE_GUI = "GUI"

# Restricted command vocabularies for Vosk grammar mode.
# By narrowing the search space to only known commands, the recognizer
# avoids guessing among tens of thousands of words and accuracy improves
# dramatically for command-and-control use cases.
COMMAND_GRAMMAR_DE = [
    # Wake words
    "martin",
    # Directions
    "hoch", "runter", "links", "rechts",
    # Stretch / shrink
    "vor", "zurück",
    # Home
    "home", 
    # Gripper
    "nimm", "gib",
    "auf", "zu",
    # Rotation gripper
    "dreh links", "dreh rechts",
    # Stop arm motion
    "halt", 
    # Stop
    "stop", "stopp", 
    # Catch-all for unrecognised speech
    "[unk]",
]

COMMAND_GRAMMAR_EN = [
    # Wake words
    "robby",
    # Directions
    "up", "down", "left", "right",
    # Stretch / shrink
    "forward", "backward",
    # Home
    "home",
    # Gripper
    "grip", "grab", "take", "open", "close", "release",
    # Rotation
    "wrist left", "wrist right",
    "turn left", "turn right",
    # Stop
    "stop", "halt",
    # Catch-all for unrecognised speech
    "[unk]",
]


def _normalize_device_name(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", "", text.lower())


def _normalize_phrase(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", " ", text.lower()).strip()


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
        self.stream_start_timeout_s = max(
            1.0,
            float(os.environ.get("DOFBOT_SPEECH_STREAM_START_TIMEOUT_S", str(DEFAULT_STREAM_START_TIMEOUT_S))),
        )
        self.stream_retry_delay_s = max(
            0.2,
            float(os.environ.get("DOFBOT_SPEECH_STREAM_RETRY_DELAY_S", str(DEFAULT_STREAM_RETRY_DELAY_S))),
        )
        self.log_partial_results = os.environ.get("DOFBOT_SPEECH_LOG_PARTIALS", "0").strip().lower() in ("1", "true", "yes", "on")
        configured_wake_word = os.environ.get("DOFBOT_WAKE_WORD", "").strip().lower()
        if configured_wake_word:
            self.wake_word = configured_wake_word
        elif self.language.startswith("de"):
            self.wake_word = DEFAULT_WAKE_WORD_DE
        else:
            self.wake_word = DEFAULT_WAKE_WORD_EN
        configured_wake_word_aliases = os.environ.get("DOFBOT_WAKE_WORD_ALIASES", "").strip().lower()
        self.wake_word_aliases = [alias for alias in re.split(r"[\s,;|]+", configured_wake_word_aliases) if alias]
        self.wake_word_timeout_s = max(0.5, float(os.environ.get("DOFBOT_WAKE_WORD_TIMEOUT_S", "4.0")))
        if self.device.lower() in {"default", "auto"}:
            self.device = ""

        self.audio_queue: queue.Queue[bytes] = queue.Queue(maxsize=self.max_queue_size)
        self.transcript_publisher = self.create_publisher(String, self.transcript_topic, 10)
        self.status_publisher = self.create_publisher(String, "roboarm/speech_status", 10)
        self.create_subscription(String, "roboarm/status", self._handle_bridge_status, 10)

        self.model: Optional[Model] = None
        self.recognizer: Optional[KaldiRecognizer] = None
        self.stream = None
        self.worker_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.last_audio_time_s = 0.0
        self.last_partial_text = ""
        self.awaiting_prompt_until_s = 0.0
        self.wake_word_armed = False
        self.voice_command_mode_active = False
        self.control_mode = os.environ.get("DOFBOT_CONTROL_MODE", CONTROL_MODE_GUI).strip().upper() or CONTROL_MODE_GUI
        self.mode_pause_reported = False

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
        self._reset_recognizer()

        self.worker_thread = threading.Thread(target=self._run_capture_loop, daemon=True)
        self.worker_thread.start()
        self._publish_status("listening", f"model={self.model_dir} device={self.device or 'default'} rate={self.sample_rate}")
        self.get_logger().info(f"Speech input enabled with Vosk model at {self.model_dir}")

    def _handle_bridge_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return

        if not isinstance(payload, dict):
            return

        mode = str(payload.get("control_mode", "")).strip().upper()
        if not mode:
            return

        previous_mode = self.control_mode
        self.control_mode = mode
        if previous_mode != mode:
            self.get_logger().info(f"Speech control mode updated: {previous_mode} -> {mode}")

    def _capture_allowed(self) -> bool:
        return self.control_mode != CONTROL_MODE_GUI

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
        if self.stop_event.is_set() or not self._capture_allowed():
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

    def _log_partial_result(self) -> None:
        if not self.log_partial_results or self.recognizer is None:
            return
        try:
            payload = json.loads(self.recognizer.PartialResult())
        except json.JSONDecodeError:
            return

        partial_text = str(payload.get("partial", "")).strip()
        if partial_text and partial_text != self.last_partial_text:
            self.last_partial_text = partial_text
            self.get_logger().info(f"Speech partial: {partial_text}")

    def _handle_final_transcript(self, transcript: str) -> None:
        normalized = transcript.strip().lower()
        if not normalized:
            return

        now_s = time.time()

        wake_word_match = self._match_wake_word(normalized)
        if wake_word_match:
            cleaned = normalized.replace(wake_word_match, "", 1).strip(" ,.!?;:")
            self.voice_command_mode_active = True
            self.wake_word_armed = False
            self.awaiting_prompt_until_s = 0.0

            if cleaned:
                if self._is_stop_phrase(cleaned):
                    self._emit_transcript("stop", is_final=True)
                    self.voice_command_mode_active = False
                    self._publish_status("listening", "voice command mode disabled by stop")
                    return
                if self._is_halt_phrase(cleaned):
                    self._emit_transcript("halt", is_final=True)
                    self._publish_status("armed", "motion halted; voice command mode remains active")
                    return
                self._emit_transcript(cleaned, is_final=True)
                self._publish_status("armed", f"voice command mode active; prompt={cleaned}")
                return

            self._publish_status("armed", "voice command mode enabled")
            return

        if self.voice_command_mode_active:
            if self._is_stop_phrase(normalized):
                self._emit_transcript("stop", is_final=True)
                self.voice_command_mode_active = False
                self._publish_status("listening", "voice command mode disabled by stop")
                return

            if self._is_halt_phrase(normalized):
                self._emit_transcript("halt", is_final=True)
                self._publish_status("armed", "motion halted; voice command mode remains active")
                return

            self._emit_transcript(normalized, is_final=True)
            self._publish_status("armed", f"voice command mode active; prompt={normalized}")
            return

        if self.wake_word_armed and now_s <= self.awaiting_prompt_until_s:
            self.wake_word_armed = False
            self.awaiting_prompt_until_s = 0.0
            self._emit_transcript(normalized, is_final=True)
            self._publish_status("listening", f"prompt captured: {normalized}")
            return

        self._publish_status("listening", f"ignored speech without wake word: {transcript}")

    def _match_wake_word(self, text: str) -> str:
        candidates = [self.wake_word, *self.wake_word_aliases]
        for candidate in candidates:
            if not candidate:
                continue
            if self._phrase_matches(candidate, text):
                return candidate
        return ""

    def _phrase_matches(self, candidate: str, text: str) -> bool:
        candidate_normalized = _normalize_phrase(candidate)
        text_normalized = _normalize_phrase(text)
        if not candidate_normalized or not text_normalized:
            return False

        if candidate_normalized == text_normalized:
            return True

        if candidate_normalized in text_normalized:
            return True

        candidate_compact = candidate_normalized.replace(" ", "")
        text_compact = text_normalized.replace(" ", "")
        if candidate_compact and candidate_compact in text_compact:
            return True

        candidate_tokens = [token for token in candidate_normalized.split() if token]
        text_tokens = [token for token in text_normalized.split() if token]
        return bool(candidate_tokens) and all(token in text_tokens for token in candidate_tokens)

    def _is_stop_phrase(self, text: str) -> bool:
        normalized = str(text).strip().lower()
        stop_phrases = {
            "stop",
            "stopp",
        }
        return normalized in stop_phrases

    def _is_halt_phrase(self, text: str) -> bool:
        normalized = str(text).strip().lower()
        halt_phrases = {
            "halt",
            "anhalten",
            "pause",
        }
        if self.language.startswith("de") and normalized in DEFAULT_STOP_ALIASES:
            return True
        return normalized in halt_phrases

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

    def _reset_recognizer(self) -> None:
        if self.model is None:
            self.recognizer = None
            return
        grammar = COMMAND_GRAMMAR_DE if self.language.startswith("de") else COMMAND_GRAMMAR_EN
        grammar_json = json.dumps(grammar)
        self.recognizer = KaldiRecognizer(self.model, self.sample_rate, grammar_json)
        self.recognizer.SetWords(False)
        self.get_logger().info(
            f"Recognizer initialized with {len(grammar)} grammar entries "
            f"for language '{self.language}'"
        )

    def _clear_audio_queue(self) -> None:
        while True:
            try:
                self.audio_queue.get_nowait()
            except queue.Empty:
                return

    def _run_stream_once(self, stream_device) -> None:
        with sd.RawInputStream(
            samplerate=self.sample_rate,
            blocksize=self.blocksize,
            device=stream_device,
            dtype="int16",
            channels=1,
            callback=self._audio_callback,
        ):
            self._publish_status("listening", f"capturing audio from {stream_device or 'default device'}")
            opened_at_s = time.time()
            silence_deadline = 0.0
            while not self.stop_event.is_set():
                if not self._capture_allowed():
                    break
                try:
                    data = self.audio_queue.get(timeout=0.2)
                except queue.Empty:
                    if self.last_audio_time_s == 0.0 and time.time() - opened_at_s > self.stream_start_timeout_s:
                        raise RuntimeError(
                            f"no audio callbacks received within {self.stream_start_timeout_s:.1f}s"
                        )
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
                    self.last_partial_text = ""
                    silence_deadline = 0.0
                else:
                    self._log_partial_result()
                    silence_deadline = time.time() + self.flush_silence_s

    def _run_capture_loop(self) -> None:
        stream_device = self._resolve_input_device(self.device)
        attempt = 0
        try:
            while not self.stop_event.is_set():
                if not self._capture_allowed():
                    if not self.mode_pause_reported:
                        self._publish_status("paused", f"speech capture paused in control mode '{self.control_mode}'")
                        self.get_logger().info(
                            f"Speech capture paused while control mode is '{self.control_mode}'"
                        )
                        self.mode_pause_reported = True
                    self._clear_audio_queue()
                    self.stop_event.wait(0.25)
                    continue

                if self.mode_pause_reported:
                    self._publish_status("listening", f"speech capture resumed in control mode '{self.control_mode}'")
                    self.get_logger().info(
                        f"Speech capture resumed while control mode is '{self.control_mode}'"
                    )
                    self.mode_pause_reported = False

                attempt += 1
                self._clear_audio_queue()
                self._reset_recognizer()
                self.last_audio_time_s = 0.0
                try:
                    self._run_stream_once(stream_device)
                    break
                except Exception as exc:  # pragma: no cover - device/runtime specific
                    if self.stop_event.is_set():
                        break
                    self._publish_status("warning", f"speech stream restart {attempt}: {exc}")
                    self.get_logger().warning(
                        f"Speech capture attempt {attempt} failed; retrying in {self.stream_retry_delay_s:.1f}s: {exc}"
                    )
                    self.stop_event.wait(self.stream_retry_delay_s)
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

        requested_normalized = _normalize_device_name(requested)
        if requested_normalized in {"", "default", "auto"}:
            return None

        try:
            devices = sd.query_devices()
        except Exception as exc:  # pragma: no cover - runtime dependent
            self.get_logger().warning(f"Unable to list audio devices: {exc}")
            return None

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
        requested_tokens = [token for token in re.split(r"[^a-z0-9]+", requested_lower) if token]
        for index, dev in capture_devices:
            dev_name = str(dev.get("name", ""))
            dev_name_lower = dev_name.lower()
            dev_name_normalized = _normalize_device_name(dev_name)
            dev_tokens = {token for token in re.split(r"[^a-z0-9]+", dev_name_lower) if token}

            if requested_lower in dev_name_lower:
                self.get_logger().info(f"Selected input device #{index}: {dev_name}")
                return index

            if requested_normalized and requested_normalized == dev_name_normalized:
                self.get_logger().info(f"Selected input device #{index}: {dev_name}")
                return index

            if requested_normalized and requested_normalized in dev_name_normalized:
                self.get_logger().info(f"Selected input device #{index}: {dev_name}")
                return index

            if requested_tokens and all(token in dev_tokens for token in requested_tokens):
                self.get_logger().info(f"Selected input device #{index}: {dev_name}")
                return index

        self.get_logger().warning(
            f"Requested speech device '{requested}' not found by name; using default input device"
        )
        return None

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
