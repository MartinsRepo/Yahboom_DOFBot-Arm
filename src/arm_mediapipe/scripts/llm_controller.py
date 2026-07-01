#!/usr/bin/env python3
"""LLM-driven command publisher for DOFBot bridge arbitration."""

import json
import os
import re
import shutil
import subprocess
import time
import urllib.error
import urllib.request
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


FACE_DETECTION_TOPIC = "/mediapipe/face_detection_summary"
STATUS_TOPIC = "roboarm/status"
JOINT_STATES_TOPIC = "roboarm/joint_states"
LLM_COMMAND_TOPIC = "roboarm/llm_command"
TOOL_CALL_TOPIC = "roboarm/tool_call"
TOOL_RESPONSE_TOPIC = "roboarm/tool_response"
SPEECH_INPUT_TOPIC = "roboarm/speech_input"
DEFAULT_CONFIG_PATH = "/opt/ros/overlay_ws/config/llm_controller.json"

CONTROL_MODE_GUI = "GUI"
CONTROL_MODE_LLM = "LLM"
CONTROL_MODE_AUTO = "AUTO"

SUPPORTED_ACTIONS = {
    "toggle_active",
    "power_on",
    "power_off",
    "home",
    "move_left",
    "move_right",
    "move_up",
    "move_down",
    "turn_left",
    "turn_right",
    "arm_stretch",
    "arm_shrink",
    "grip_open",
    "grip_close",
    "refresh",
}

CONTINUOUS_SPEECH_ACTIONS = {
    "move_left",
    "move_right",
    "move_up",
    "move_down",
    "turn_left",
    "turn_right",
    "arm_stretch",
    "arm_shrink",
}


class LLMController(Node):
    def __init__(self) -> None:
        super().__init__("llm_controller")

        self.config_path = os.environ.get("DOFBOT_LLM_CONFIG_PATH", DEFAULT_CONFIG_PATH).strip()
        self.file_config = self._load_file_config(self.config_path)

        self.enabled = self._env_or_config_bool("ENABLE_LLM_CONTROLLER", "enabled", False)
        self.provider = self._env_or_config_str("DOFBOT_LLM_PROVIDER", "provider", "heuristic").lower()
        self.llm_endpoint = self._env_or_config_str("DOFBOT_LLM_ENDPOINT", "endpoint", "")
        self.api_token = self._env_or_config_str("DOFBOT_LLM_API_TOKEN", "api_token", "")
        self.ollama_base_url = self._env_or_config_str("DOFBOT_OLLAMA_BASE_URL", "ollama_base_url", "http://127.0.0.1:11434")
        self.ollama_model = self._env_or_config_str("DOFBOT_OLLAMA_MODEL", "ollama_model", "llama3.1:8b")
        self.enable_agent_tools = self._env_or_config_bool("DOFBOT_AGENT_TOOLS_ENABLED", "agent_tools_enabled", True)
        self.system_prompt = self._env_or_config_str(
            "DOFBOT_LLM_SYSTEM_PROMPT",
            "system_prompt",
            (
                "Du bist ein praeziser, sicherheitsorientierter Agent zur Steuerung eines 6-DOF-Roboterarms. "
                "Gib exakt ein JSON-Objekt aus. Wenn Tool-Modus aktiv ist, liefere einen Tool-Call mit Feldern "
                "request_id, tool, params und optional constraints. Benutze nur erlaubte Tools. "
                "Wenn unklar, waehle eine sichere Neutralaktion (arm_action mit action=refresh)."
            ),
        )
        self.loop_period_s = max(0.12, self._env_or_config_float("DOFBOT_LLM_LOOP_PERIOD_S", "loop_period_s", 0.15))
        self.request_timeout_s = max(0.1, self._env_or_config_float("DOFBOT_LLM_REQUEST_TIMEOUT_S", "request_timeout_s", 20.0))
        self.request_retries = max(1, int(self._env_or_config_float("DOFBOT_LLM_REQUEST_RETRIES", "request_retries", 3.0)))
        self.retry_backoff_s = max(0.0, self._env_or_config_float("DOFBOT_LLM_RETRY_BACKOFF_S", "retry_backoff_s", 1.0))
        self.fallback_on_request_error = self._env_or_config_bool(
            "DOFBOT_LLM_FALLBACK_ON_ERROR",
            "fallback_on_request_error",
            True,
        )
        self.speech_prompt_stale_s = max(0.0, self._env_or_config_float("DOFBOT_SPEECH_PROMPT_STALE_S", "speech_prompt_stale_s", 20.0))
        self.require_speech_prompt = self._env_or_config_bool(
            "DOFBOT_LLM_REQUIRE_SPEECH_PROMPT",
            "require_speech_prompt",
            True,
        )
        speech_language = os.environ.get("DOFBOT_SPEECH_LANGUAGE", "").strip().lower()
        default_wake_word = "karli" if speech_language.startswith("de") else "hello"
        self.wake_word = self._env_or_config_str("DOFBOT_WAKE_WORD", "wake_word", default_wake_word)
        self.voice_output_enabled = self._env_or_config_bool("DOFBOT_VOICE_OUTPUT_ENABLED", "voice_output_enabled", False)
        default_voice_output_voice = "de" if speech_language.startswith("de") else "en"
        self.voice_output_voice = self._env_or_config_str(
            "DOFBOT_VOICE_OUTPUT_VOICE",
            "voice_output_voice",
            default_voice_output_voice,
        )

        self.latest_status: dict = {}
        self.latest_joint_state: Optional[JointState] = None
        self.latest_face_summary: dict = {}
        self.latest_speech_prompt: str = ""
        self.latest_speech_prompt_time_s = 0.0
        self.last_sent_action = ""
        self.last_sent_time_s = 0.0
        self.latched_motion_command: Optional[dict] = None

        self.command_publisher = self.create_publisher(String, LLM_COMMAND_TOPIC, 10)
        self.tool_call_publisher = self.create_publisher(String, TOOL_CALL_TOPIC, 10)
        self.create_subscription(String, TOOL_RESPONSE_TOPIC, self._handle_tool_response, 10)
        self.create_subscription(String, STATUS_TOPIC, self._handle_status, 10)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._handle_joint_states, 10)
        self.create_subscription(String, FACE_DETECTION_TOPIC, self._handle_face_summary, 10)
        self.create_subscription(String, SPEECH_INPUT_TOPIC, self._handle_speech_input, 10)

        self.create_timer(self.loop_period_s, self._tick)

        mode = "enabled" if self.enabled else "disabled"
        target = self._provider_target_label()
        self.get_logger().info(
            f"LLM controller {mode}; provider={self.provider}; target={target}; "
            f"timeout={self.request_timeout_s:.1f}s retries={self.request_retries}; "
            f"agent_tools={'on' if self.enable_agent_tools else 'off'}; "
            f"require_speech_prompt={'on' if self.require_speech_prompt else 'off'}"
        )
        if self.voice_output_enabled:
            self.get_logger().info("Voice output enabled")

    def _load_file_config(self, path: str) -> dict:
        if not path or not os.path.exists(path):
            return {}
        try:
            with open(path, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Failed reading LLM config file '{path}': {exc}")
            return {}

        if not isinstance(payload, dict):
            self.get_logger().warning(f"LLM config file '{path}' must contain a JSON object")
            return {}
        return payload

    def _env_or_config_str(self, env_key: str, cfg_key: str, default: str) -> str:
        raw = os.environ.get(env_key)
        if raw is not None and str(raw).strip() != "":
            return str(raw).strip()

        cfg_value = self.file_config.get(cfg_key)
        if cfg_value is None:
            return default
        return str(cfg_value).strip()

    def _env_or_config_bool(self, env_key: str, cfg_key: str, default: bool) -> bool:
        raw = os.environ.get(env_key)
        if raw is not None and str(raw).strip() != "":
            return str(raw).strip().lower() in ("1", "true", "yes", "on")

        cfg_value = self.file_config.get(cfg_key)
        if isinstance(cfg_value, bool):
            return cfg_value
        if isinstance(cfg_value, str):
            return cfg_value.strip().lower() in ("1", "true", "yes", "on")
        return default

    def _env_or_config_float(self, env_key: str, cfg_key: str, default: float) -> float:
        raw = os.environ.get(env_key)
        if raw is not None and str(raw).strip() != "":
            try:
                return float(raw)
            except (TypeError, ValueError):
                return default

        cfg_value = self.file_config.get(cfg_key)
        try:
            return float(cfg_value)
        except (TypeError, ValueError):
            return default

    def _provider_target_label(self) -> str:
        if self.provider == "ollama":
            return f"{self.ollama_base_url} [{self.ollama_model}]"
        if self.provider == "http":
            return self.llm_endpoint or "missing-endpoint"
        return "heuristic"

    def _handle_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self.latest_status = payload

    def _handle_joint_states(self, message: JointState) -> None:
        self.latest_joint_state = message

    def _handle_face_summary(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self.latest_face_summary = payload

    def _handle_speech_input(self, message: String) -> None:
        transcript = self._extract_transcript(message.data)
        if not transcript:
            return
        self.latest_speech_prompt = transcript
        self.latest_speech_prompt_time_s = time.time()
        self.get_logger().info(f"Speech prompt received: {transcript}")

    def _handle_tool_response(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Invalid tool response payload: {message.data}")
            return

        if not isinstance(payload, dict):
            return

        request_id = str(payload.get("request_id", ""))
        status = str(payload.get("status", ""))
        reason = str(payload.get("reason", ""))
        self.get_logger().info(f"Tool response: request_id={request_id} status={status} reason={reason}")

    def _tick(self) -> None:
        if not self.enabled:
            self.latched_motion_command = None
            return
        if not self.latest_status:
            return

        mode = str(self.latest_status.get("control_mode", CONTROL_MODE_GUI)).upper()
        if mode not in (CONTROL_MODE_LLM, CONTROL_MODE_AUTO):
            self.latched_motion_command = None
            return

        if not bool(self.latest_status.get("connected", False)):
            self.latched_motion_command = None
            return

        now_s = time.time()
        if now_s - self.last_sent_time_s < self.loop_period_s:
            return

        speech_prompt = self._consume_active_speech_prompt(now_s)
        command = None
        command_from_latched_motion = False
        if speech_prompt:
            command = self._speech_to_action(speech_prompt.lower().strip())
            if command:
                self.get_logger().info(f"Speech command mapped directly: '{speech_prompt}' -> {command}")
                self._update_latched_motion(command)

        if command is None and self.latched_motion_command is not None:
            command = dict(self.latched_motion_command)
            command_from_latched_motion = True

        if command is None:
            if self.require_speech_prompt:
                return
            state = self._build_state(now_s)
            command = self._query_llm_or_fallback(state)

        if not command:
            return

        if self.enable_agent_tools:
            tool_call = self._to_tool_call(command, now_s)
            if not tool_call:
                self.get_logger().warning(f"Ignoring invalid LLM tool call payload: {command}")
                return

            message = String()
            message.data = json.dumps(tool_call)
            self.tool_call_publisher.publish(message)
            self.last_sent_action = str(tool_call.get("tool", ""))
            self.last_sent_time_s = now_s
            if not command_from_latched_motion:
                self._speak_action_feedback(self.last_sent_action)
            return

        action = str(command.get("action", "")).strip()
        if action not in SUPPORTED_ACTIONS:
            self.get_logger().warning(f"Ignoring unsupported LLM action: {action}")
            return

        payload = {
            "action": action,
            "timestamp_s": now_s,
            "source": "llm_controller",
        }
        duration_ms = command.get("duration_ms")
        if duration_ms is not None:
            try:
                payload["duration_ms"] = int(duration_ms)
            except (TypeError, ValueError):
                pass

        message = String()
        message.data = json.dumps(payload)
        self.command_publisher.publish(message)
        self.last_sent_action = action
        self.last_sent_time_s = now_s
        if not command_from_latched_motion:
            self._speak_action_feedback(action)

    def _update_latched_motion(self, command: dict) -> None:
        action = str(command.get("action", "")).strip()
        if action == "refresh":
            if self.latched_motion_command is not None:
                self.get_logger().info("Latched speech motion cleared by halt/stop command")
            self.latched_motion_command = None
            return

        if action in CONTINUOUS_SPEECH_ACTIONS:
            self.latched_motion_command = dict(command)
            self.get_logger().info(f"Latched speech motion active: {action}")
            return

        self.latched_motion_command = None

    def _to_tool_call(self, payload: dict, now_s: float) -> Optional[dict]:
        if not isinstance(payload, dict):
            return None

        if "tool_call" in payload and isinstance(payload.get("tool_call"), dict):
            payload = payload.get("tool_call")

        if "tool" in payload:
            tool = str(payload.get("tool", "")).strip()
            params = payload.get("params", {})
            constraints = payload.get("constraints", {})
            request_id = str(payload.get("request_id", "")).strip() or f"tool-{int(now_s * 1000)}"
            if not isinstance(params, dict):
                params = {}
            if not isinstance(constraints, dict):
                constraints = {}
            if not tool:
                return None
            return {
                "request_id": request_id,
                "tool": tool,
                "params": params,
                "constraints": constraints,
                "timestamp_s": now_s,
                "source": "llm_controller",
            }

        action = str(payload.get("action", "")).strip()
        if action not in SUPPORTED_ACTIONS:
            return None

        params = {"action": action}
        duration_ms = payload.get("duration_ms")
        if duration_ms is not None:
            try:
                params["duration_ms"] = int(duration_ms)
            except (TypeError, ValueError):
                pass

        return {
            "request_id": f"tool-{int(now_s * 1000)}",
            "tool": "arm_action",
            "params": params,
            "constraints": {},
            "timestamp_s": now_s,
            "source": "llm_controller",
        }

    def _speak_action_feedback(self, action: str) -> None:
        if not self.voice_output_enabled:
            return

        spoken = action.replace("_", " ").strip()
        text = f"Executing {spoken}"
        binary = shutil.which("espeak-ng") or shutil.which("espeak")
        if not binary:
            self.get_logger().warning("Voice output enabled but no espeak binary found")
            return

        command = [binary]
        if self.voice_output_voice:
            command.extend(["-v", self.voice_output_voice])
        command.append(text)

        try:
            subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as exc:  # pragma: no cover - runtime specific
            self.get_logger().warning(f"Failed to play voice output: {exc}")

    def _build_state(self, now_s: float) -> dict:
        joints_deg = []
        if self.latest_joint_state and self.latest_joint_state.position:
            joints_deg = [round(value * 180.0 / 3.14159265359, 2) for value in self.latest_joint_state.position]

        return {
            "timestamp_s": now_s,
            "status": self.latest_status,
            "face": self.latest_face_summary,
            "speech_prompt": self._get_active_speech_prompt(now_s),
            "joint_angles_deg": joints_deg,
            "last_sent_action": self.last_sent_action,
        }

    def _get_active_speech_prompt(self, now_s: float) -> str:
        if not self.latest_speech_prompt:
            return ""
        if self.speech_prompt_stale_s > 0 and now_s - self.latest_speech_prompt_time_s > self.speech_prompt_stale_s:
            self.latest_speech_prompt = ""
            return ""
        return self.latest_speech_prompt

    def _consume_active_speech_prompt(self, now_s: float) -> str:
        prompt = self._get_active_speech_prompt(now_s)
        if prompt:
            self.latest_speech_prompt = ""
            self.latest_speech_prompt_time_s = 0.0
        return prompt

    def _query_llm_or_fallback(self, state: dict) -> Optional[dict]:
        if self.provider == "ollama":
            return self._query_ollama(state)
        if self.provider == "http":
            return self._query_http_endpoint(state)
        if self.provider != "heuristic":
            self.get_logger().warning(f"Unknown provider '{self.provider}', using heuristic fallback")
            self.provider = "heuristic"
        return self._heuristic_command(state)

    def _query_http_endpoint(self, state: dict) -> Optional[dict]:
        if not self.llm_endpoint:
            self.get_logger().warning("HTTP provider selected but DOFBOT_LLM_ENDPOINT is empty")
            return self._heuristic_command(state)

        request_payload = {
            "instruction": (
                "Erzeuge genau ein JSON-Objekt fuer den Roboterarm. "
                "Wenn agent_tools_enabled=true, gib einen Tool-Call aus. "
                "Sonst gib action/duration_ms aus."
            ),
            "agent_tools_enabled": self.enable_agent_tools,
            "supported_actions": sorted(SUPPORTED_ACTIONS),
            "state": state,
        }

        data = json.dumps(request_payload).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if self.api_token:
            headers["Authorization"] = f"Bearer {self.api_token}"

        request = urllib.request.Request(
            self.llm_endpoint,
            data=data,
            headers=headers,
            method="POST",
        )

        body = self._request_with_retries(request, "LLM")
        if body is None:
            if self.fallback_on_request_error:
                return self._heuristic_command(state)
            return None

        payload = self._extract_json_object(body)

        if isinstance(payload, dict):
            return payload
        self.get_logger().warning("LLM response must be a JSON object")
        return None

    def _query_ollama(self, state: dict) -> Optional[dict]:
        url = f"{self.ollama_base_url.rstrip('/')}/api/chat"
        user_prompt = {
            "agent_tools_enabled": self.enable_agent_tools,
            "supported_actions": sorted(SUPPORTED_ACTIONS),
            "state": state,
            "rules": {
                "output": "Return JSON object only",
                "allowed_tools": [
                    "arm_action",
                    "arm_home",
                    "arm_power",
                    "arm_gripper",
                    "arm_motion",
                ],
                "tool_call_schema": {
                    "request_id": "string",
                    "tool": "string",
                    "params": "object",
                    "constraints": "object optional",
                },
                "fallback_tool_call": {
                    "tool": "arm_action",
                    "params": {"action": "refresh"},
                },
                "legacy_action_schema": {
                    "action": "string",
                    "duration_ms": "int optional",
                },
            },
        }
        speech_prompt = str(state.get("speech_prompt", "")).strip()
        if speech_prompt:
            user_prompt["speech_prompt"] = speech_prompt
            user_prompt["wake_word"] = self.wake_word
            user_prompt["speech_instruction"] = (
                f"Treat the speech prompt as the user's direct instruction after the wake word '{self.wake_word}'. "
                "Map clear command keywords directly to robot actions. "
                "Accepted command vocabulary includes German and English: "
                "hoch/up, runter/down, links/left, rechts/right, stop/stopp/anhalten, "
                "home/heim, nimm/grip/grab/take, release/open/loslassen, "
                "vor/forward, zurueck/backward, rotate grip left, rotate grip right, aus/off, an/on. "
                "If unclear, use refresh."
            )
        request_payload = {
            "model": self.ollama_model,
            "stream": False,
            "messages": [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": json.dumps(user_prompt)},
            ],
            "format": "json",
        }

        headers = {"Content-Type": "application/json"}
        if self.api_token:
            headers["Authorization"] = f"Bearer {self.api_token}"

        data = json.dumps(request_payload).encode("utf-8")
        request = urllib.request.Request(url, data=data, headers=headers, method="POST")

        body = self._request_with_retries(request, "Ollama")
        if body is None:
            if self.fallback_on_request_error:
                return self._heuristic_command(state)
            return None

        root_payload = self._extract_json_object(body)
        if not isinstance(root_payload, dict):
            self.get_logger().warning("Ollama response was not valid JSON")
            return None

        message_payload = root_payload.get("message")
        if not isinstance(message_payload, dict):
            self.get_logger().warning("Ollama response missing message field")
            return None

        content = message_payload.get("content", "")
        command = self._extract_json_object(content)
        if isinstance(command, dict):
            return command
        self.get_logger().warning("Ollama message content did not contain a JSON object")
        return None

    def _request_with_retries(self, request: urllib.request.Request, label: str) -> Optional[str]:
        for attempt in range(1, self.request_retries + 1):
            try:
                with urllib.request.urlopen(request, timeout=self.request_timeout_s) as response:
                    return response.read().decode("utf-8")
            except (urllib.error.URLError, TimeoutError, ConnectionError) as exc:
                if attempt >= self.request_retries:
                    self.get_logger().warning(
                        f"{label} request failed after {attempt} attempts: {exc}"
                    )
                    return None
                self.get_logger().warning(
                    f"{label} request attempt {attempt}/{self.request_retries} failed: {exc}; retrying"
                )
                if self.retry_backoff_s > 0:
                    time.sleep(self.retry_backoff_s * attempt)
        return None

    def _extract_json_object(self, raw_text: str) -> Optional[dict]:
        text = str(raw_text).strip()
        if not text:
            return None

        try:
            payload = json.loads(text)
            if isinstance(payload, dict):
                return payload
        except json.JSONDecodeError:
            pass

        fenced = re.search(r"```(?:json)?\s*(\{.*\})\s*```", text, flags=re.DOTALL | re.IGNORECASE)
        if fenced:
            try:
                payload = json.loads(fenced.group(1))
                if isinstance(payload, dict):
                    return payload
            except json.JSONDecodeError:
                pass

        start = text.find("{")
        end = text.rfind("}")
        if start != -1 and end != -1 and end > start:
            snippet = text[start : end + 1]
            try:
                payload = json.loads(snippet)
                if isinstance(payload, dict):
                    return payload
            except json.JSONDecodeError:
                return None
        return None

    def _heuristic_command(self, state: dict) -> Optional[dict]:
        face = state.get("face", {}) if isinstance(state.get("face", {}), dict) else {}
        speech_prompt = str(state.get("speech_prompt", "")).strip().lower()

        if speech_prompt:
            return self._speech_to_action(speech_prompt)

        status = str(face.get("status", "no face"))
        center_x = face.get("center_x")

        if status == "tracking" and isinstance(center_x, (float, int)):
            if center_x < 0.42:
                return {"action": "move_left", "duration_ms": 400}
            if center_x > 0.58:
                return {"action": "move_right", "duration_ms": 400}
            return {"action": "refresh"}

        return {"action": "turn_left", "duration_ms": 450}

    @staticmethod
    def _phrase_matches(phrase: str, text: str) -> bool:
        """Match phrase against text using word boundaries to avoid partial-word hits."""
        escaped = re.escape(phrase)
        # Use word boundary for single-token phrases; space anchors for multi-word phrases.
        pattern = r"(?:^|\s)" + escaped + r"(?:\s|$)"
        return bool(re.search(pattern, text))

    def _speech_to_action(self, transcript: str) -> Optional[dict]:
        normalized = transcript.lower().strip()
        if not normalized:
            return None

        normalized = (
            normalized.replace("ä", "ae")
            .replace("ö", "oe")
            .replace("ü", "ue")
            .replace("ß", "ss")
        )

        # Keep only letters, digits and spaces; collapse whitespace.
        normalized = re.sub(r"[^a-z0-9\s]+", " ", normalized)
        normalized = re.sub(r"\s+", " ", normalized).strip()

        exact_stop_phrases = {"stop", "stopp", "anhalten", "halt", "pause", "heute"}
        if normalized in exact_stop_phrases:
            return {"action": "refresh"}

        phrase_map = [
            # Power
            (("power off", "switch off", "shutdown", "ausschalten"), {"action": "power_off"}),
            (("power on", "switch on", "wake up", "activate", "einschalten"), {"action": "power_on"}),
            # Exact single-word triggers for power (avoid matching inside longer words)
            (("aus",), {"action": "power_off"}),
            (("an",), {"action": "power_on"}),
            # Home
            (("home", "go home", "reset", "heim", "grundstellung"), {"action": "home"}),
            # Gripper rotation (must come BEFORE generic turn/grip to avoid override)
            (("rotate grip left", "wrist left", "gripper left",
              "drehe greifer links", "greifer links drehen"), {"action": "turn_left", "duration_ms": 450}),
            (("rotate grip right", "wrist right", "gripper right",
              "drehe greifer rechts", "greifer rechts drehen"), {"action": "turn_right", "duration_ms": 450}),
            # Gripper open/close
            (("open gripper", "open grip", "grip open", "open the hand",
              "release", "loslassen", "greifer auf"), {"action": "grip_open"}),
            (("close gripper", "close grip", "grip close", "close the hand",
              "grab", "take", "nimm", "nehmen", "greifen", "greifer zu"), {"action": "grip_close"}),
            # Single-word open/close (after multi-word to avoid shadowing)
            (("oeffnen",), {"action": "grip_open"}),
            (("greifen", "grip"), {"action": "grip_close"}),
            # Arm motion
            (("move left", "go left", "links"), {"action": "move_left", "duration_ms": 400}),
            (("move right", "go right", "rechts"), {"action": "move_right", "duration_ms": 400}),
            (("move up", "go up", "hoch", "nach oben"), {"action": "move_up", "duration_ms": 400}),
            (("move down", "go down", "runter", "nach unten"), {"action": "move_down", "duration_ms": 400}),
            # Single-word directional (word-boundary safe)
            (("left",), {"action": "move_left", "duration_ms": 400}),
            (("right",), {"action": "move_right", "duration_ms": 400}),
            (("up",), {"action": "move_up", "duration_ms": 400}),
            (("down",), {"action": "move_down", "duration_ms": 400}),
            # Base rotation
            (("turn left", "rotate left", "drehen links"), {"action": "turn_left", "duration_ms": 450}),
            (("turn right", "rotate right", "drehen rechts"), {"action": "turn_right", "duration_ms": 450}),
            # Arm stretch/shrink via spoken forward/backward commands
            (("vor", "forward"), {"action": "arm_stretch"}),
            (("zurueck", "backward"), {"action": "arm_shrink"}),
        ]

        for phrases, command in phrase_map:
            if any(self._phrase_matches(phrase, normalized) for phrase in phrases):
                return command

        # No keyword matched — return None so the caller treats this as unrecognised speech.
        return None

        return {"action": "refresh"}

    def _extract_transcript(self, raw_text: str) -> str:
        text = str(raw_text).strip()
        if not text:
            return ""

        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            return text

        if isinstance(payload, dict):
            transcript = payload.get("transcript")
            if transcript is not None:
                return str(transcript).strip()
        return text


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LLMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
