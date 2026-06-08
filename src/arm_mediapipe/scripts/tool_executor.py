#!/usr/bin/env python3
"""Tool executor for agentic arm control inside the ROS2 container."""

from __future__ import annotations

import json
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TOOL_CALL_TOPIC = "roboarm/tool_call"
TOOL_RESPONSE_TOPIC = "roboarm/tool_response"
LLM_COMMAND_TOPIC = "roboarm/llm_command"

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

MOTION_ACTIONS = {
    "move_left",
    "move_right",
    "move_up",
    "move_down",
    "turn_left",
    "turn_right",
    "arm_stretch",
    "arm_shrink",
}


class ToolExecutor(Node):
    def __init__(self) -> None:
        super().__init__("tool_executor")

        self.enabled = os.environ.get("ENABLE_ARM_AGENT_TOOLS", "1").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )
        self.max_duration_ms = self._read_int_env("DOFBOT_TOOL_MAX_DURATION_MS", 1200, minimum=100)
        self.allowed_actions = self._read_allowed_actions_env("DOFBOT_TOOL_ALLOWED_ACTIONS")

        self.tool_response_publisher = self.create_publisher(String, TOOL_RESPONSE_TOPIC, 10)
        self.command_publisher = self.create_publisher(String, LLM_COMMAND_TOPIC, 10)
        self.create_subscription(String, TOOL_CALL_TOPIC, self._handle_tool_call, 10)

        self.get_logger().info(
            f"Tool executor started; enabled={'yes' if self.enabled else 'no'}; "
            f"max_duration_ms={self.max_duration_ms}; allowed_actions={','.join(sorted(self.allowed_actions))}"
        )

    def _read_int_env(self, key: str, default: int, minimum: int = 0) -> int:
        raw = os.environ.get(key, "").strip()
        if not raw:
            return default
        try:
            value = int(raw)
        except ValueError:
            return default
        return max(minimum, value)

    def _read_allowed_actions_env(self, key: str) -> set[str]:
        raw = os.environ.get(key, "").strip()
        if not raw:
            return set(SUPPORTED_ACTIONS)

        requested = {part.strip() for part in raw.split(",") if part.strip()}
        allowed = requested.intersection(SUPPORTED_ACTIONS)
        if not allowed:
            return set(SUPPORTED_ACTIONS)
        return allowed

    def _validate_policy(self, mapped: dict) -> Optional[str]:
        action = str(mapped.get("action", "")).strip()
        if action not in self.allowed_actions:
            return f"action '{action}' not allowed by tool policy"

        duration_ms = mapped.get("duration_ms")
        if duration_ms is not None:
            try:
                duration_value = int(duration_ms)
            except (TypeError, ValueError):
                return "duration_ms must be an integer"
            if duration_value < 0:
                return "duration_ms must be >= 0"
            if duration_value > self.max_duration_ms:
                return f"duration_ms exceeds max_duration_ms ({self.max_duration_ms})"
        return None

    def _publish_tool_response(
        self,
        request_id: str,
        status: str,
        reason: str,
        action: str = "",
    ) -> None:
        payload = {
            "request_id": request_id,
            "status": status,
            "reason": reason,
            "action": action,
            "timestamp_s": time.time(),
            "source": "tool_executor",
        }
        message = String()
        message.data = json.dumps(payload)
        self.tool_response_publisher.publish(message)

    def _handle_tool_call(self, message: String) -> None:
        request_id = ""
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self._publish_tool_response(request_id, "rejected", "invalid JSON payload")
            return

        if not isinstance(payload, dict):
            self._publish_tool_response(request_id, "rejected", "payload must be a JSON object")
            return

        request_id = str(payload.get("request_id", "")).strip() or f"tool-{int(time.time() * 1000)}"
        if not self.enabled:
            self._publish_tool_response(request_id, "rejected", "tool executor disabled")
            return

        action_payload = self._map_tool_to_action(payload)
        if action_payload is None:
            self._publish_tool_response(request_id, "rejected", "unsupported tool call")
            return

        policy_error = self._validate_policy(action_payload)
        if policy_error:
            self._publish_tool_response(request_id, "rejected", policy_error)
            return

        command_message = String()
        command_message.data = json.dumps(action_payload)
        self.command_publisher.publish(command_message)

        action = str(action_payload.get("action", ""))
        self.get_logger().info(f"Executed tool call request_id={request_id} mapped_action={action}")
        self._publish_tool_response(request_id, "success", "command forwarded", action)

    def _map_tool_to_action(self, payload: dict) -> Optional[dict]:
        tool = str(payload.get("tool", "")).strip()
        params = payload.get("params", {})
        if not isinstance(params, dict):
            params = {}

        mapped: Optional[dict] = None

        if tool == "arm_action":
            action = str(params.get("action", "")).strip()
            if action in SUPPORTED_ACTIONS:
                mapped = {"action": action}

        elif tool == "arm_home":
            mapped = {"action": "home"}

        elif tool == "arm_power":
            state = str(params.get("state", "")).strip().lower()
            if state in {"on", "enable", "enabled", "1", "true"}:
                mapped = {"action": "power_on"}
            elif state in {"off", "disable", "disabled", "0", "false"}:
                mapped = {"action": "power_off"}

        elif tool == "arm_gripper":
            state = str(params.get("state", "")).strip().lower()
            if state in {"open", "auf", "oeffnen"}:
                mapped = {"action": "grip_open"}
            elif state in {"close", "zu", "schliessen"}:
                mapped = {"action": "grip_close"}

        elif tool == "arm_motion":
            motion = str(params.get("motion", "")).strip()
            if motion in MOTION_ACTIONS:
                mapped = {"action": motion}

        if mapped is None:
            return None

        duration_ms = params.get("duration_ms")
        if duration_ms is not None:
            try:
                mapped["duration_ms"] = int(duration_ms)
            except (TypeError, ValueError):
                pass

        mapped["timestamp_s"] = time.time()
        mapped["source"] = "tool_executor"
        return mapped


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ToolExecutor()
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
