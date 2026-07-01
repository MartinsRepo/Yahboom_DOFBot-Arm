#!/usr/bin/env bash
set -euo pipefail

OVERLAY_WS="${OVERLAY_WS:-/opt/ros/overlay_ws}"
export PYTHONPATH="${OVERLAY_WS}:${PYTHONPATH:-}"

set +u
source /opt/ros/jazzy/setup.bash
source "$OVERLAY_WS/install/setup.bash"
set -u

if [[ -d "$OVERLAY_WS/third_party/Arm_Lib" && -z "${DOFBOT_ARM_LIB_DIR:-}" ]]; then
  export DOFBOT_ARM_LIB_DIR="$OVERLAY_WS/third_party/Arm_Lib"
fi

if [[ -n "${DOFBOT_SERIAL_DEVICE:-}" && ! -e "${DOFBOT_SERIAL_DEVICE}" ]]; then
  echo "Warning: serial device ${DOFBOT_SERIAL_DEVICE} is not visible inside container." >&2
  echo "Ensure docker compose maps the device via the devices section." >&2
fi

launch_stack() {
  ros2 launch arm_mediapipe docker_stack.launch.py \
    serial_device:="${DOFBOT_SERIAL_DEVICE:-auto}" \
    arm_lib_dir:="${DOFBOT_ARM_LIB_DIR:-}" \
    camera_device:="${DOFBOT_CAMERA_DEVICE:-0}" \
    camera_width:="${DOFBOT_CAMERA_WIDTH:-640}" \
    camera_height:="${DOFBOT_CAMERA_HEIGHT:-480}" \
    camera_fps:="${DOFBOT_CAMERA_FPS:-15}" \
    camera_rotate_180:="${DOFBOT_CAMERA_ROTATE_180:-1}" \
    show_preview:="${DOFBOT_SHOW_PREVIEW:-0}" \
    face_detection_min_conf:="${DOFBOT_FACE_DETECTION_MIN_CONF:-0.60}" \
    enable_face_detection:="${ENABLE_FACE_DETECTION:-1}" \
    enable_face_mesh:="${ENABLE_FACE_MESH:-0}" \
    enable_pose:="${ENABLE_POSE:-0}" \
    enable_speech_controller:="${ENABLE_SPEECH_CONTROLLER:-0}" \
    enable_llm_controller:="${ENABLE_LLM_CONTROLLER:-0}" \
    enable_agent_tools:="${ENABLE_ARM_AGENT_TOOLS:-1}" \
    control_mode:="${DOFBOT_CONTROL_MODE:-GUI}" \
    strict_safety:="${DOFBOT_STRICT_SAFETY:-1}" \
    command_rate_limit_hz:="${DOFBOT_COMMAND_RATE_LIMIT_HZ:-8.0}" \
    llm_stale_timeout_s:="${DOFBOT_LLM_STALE_TIMEOUT_S:-2.0}" \
    manual_override_window_s:="${DOFBOT_MANUAL_OVERRIDE_WINDOW_S:-1.5}" \
    vosk_model_dir:="${DOFBOT_VOSK_MODEL_DIR:-/opt/ros/overlay_ws/models/vosk-model-small-de-zamia-0.3}" \
    speech_device:="${DOFBOT_SPEECH_DEVICE:-pulse}" \
    speech_sample_rate:="${DOFBOT_SPEECH_SAMPLE_RATE:-16000}" \
    speech_blocksize:="${DOFBOT_SPEECH_BLOCKSIZE:-2000}" \
    speech_language:="${DOFBOT_SPEECH_LANGUAGE:-de}" \
    speech_topic:="${DOFBOT_SPEECH_TOPIC:-roboarm/speech_input}" \
    speech_flush_silence_s:="${DOFBOT_SPEECH_FLUSH_SILENCE_S:-0.4}" \
    wake_word:="${DOFBOT_WAKE_WORD:-martin}" \
    wake_word_timeout_s:="${DOFBOT_WAKE_WORD_TIMEOUT_S:-4.0}"
}

  export DOFBOT_LLM_CONFIG_PATH="${DOFBOT_LLM_CONFIG_PATH:-/opt/ros/overlay_ws/config/llm_controller.json}"
  export DOFBOT_LLM_PROVIDER="${DOFBOT_LLM_PROVIDER:-ollama}"
  export DOFBOT_LLM_ENDPOINT="${DOFBOT_LLM_ENDPOINT:-}"
  export DOFBOT_LLM_API_TOKEN="${DOFBOT_LLM_API_TOKEN:-}"
  export DOFBOT_OLLAMA_BASE_URL="${DOFBOT_OLLAMA_BASE_URL:-http://127.0.0.1:11434}"
  export DOFBOT_OLLAMA_MODEL="${DOFBOT_OLLAMA_MODEL:-llama3.1:8b}"
  export DOFBOT_LLM_SYSTEM_PROMPT="${DOFBOT_LLM_SYSTEM_PROMPT:-}"
  export DOFBOT_LLM_LOOP_PERIOD_S="${DOFBOT_LLM_LOOP_PERIOD_S:-0.15}"
  export DOFBOT_LLM_REQUEST_TIMEOUT_S="${DOFBOT_LLM_REQUEST_TIMEOUT_S:-20.0}"
  export DOFBOT_LLM_REQUEST_RETRIES="${DOFBOT_LLM_REQUEST_RETRIES:-3}"
  export DOFBOT_LLM_RETRY_BACKOFF_S="${DOFBOT_LLM_RETRY_BACKOFF_S:-1.0}"
  export DOFBOT_LLM_FALLBACK_ON_ERROR="${DOFBOT_LLM_FALLBACK_ON_ERROR:-1}"
  export DOFBOT_AGENT_TOOLS_ENABLED="${DOFBOT_AGENT_TOOLS_ENABLED:-${ENABLE_ARM_AGENT_TOOLS:-1}}"
  export ENABLE_ARM_AGENT_TOOLS="${ENABLE_ARM_AGENT_TOOLS:-1}"
  export DOFBOT_TOOL_MAX_DURATION_MS="${DOFBOT_TOOL_MAX_DURATION_MS:-1200}"
  export DOFBOT_TOOL_ALLOWED_ACTIONS="${DOFBOT_TOOL_ALLOWED_ACTIONS:-}"
  export DOFBOT_VOICE_OUTPUT_ENABLED="${DOFBOT_VOICE_OUTPUT_ENABLED:-0}"
  export DOFBOT_VOICE_OUTPUT_VOICE="${DOFBOT_VOICE_OUTPUT_VOICE:-}"
  export DOFBOT_VOSK_MODEL_DIR="${DOFBOT_VOSK_MODEL_DIR:-/opt/ros/overlay_ws/models/vosk-model-small-de-zamia-0.3}"
  export DOFBOT_SPEECH_DEVICE="${DOFBOT_SPEECH_DEVICE:-pulse}"
  export DOFBOT_SPEECH_SAMPLE_RATE="${DOFBOT_SPEECH_SAMPLE_RATE:-16000}"
  export DOFBOT_SPEECH_BLOCKSIZE="${DOFBOT_SPEECH_BLOCKSIZE:-2000}"
  export DOFBOT_SPEECH_LANGUAGE="${DOFBOT_SPEECH_LANGUAGE:-de}"
  export DOFBOT_SPEECH_TOPIC="${DOFBOT_SPEECH_TOPIC:-roboarm/speech_input}"
  export DOFBOT_SPEECH_FLUSH_SILENCE_S="${DOFBOT_SPEECH_FLUSH_SILENCE_S:-0.4}"
  export DOFBOT_WAKE_WORD="${DOFBOT_WAKE_WORD:-martin}"
  export DOFBOT_WAKE_WORD_TIMEOUT_S="${DOFBOT_WAKE_WORD_TIMEOUT_S:-4.0}"

wait_for_topic() {
  local topic=$1
  local timeout_s=${2:-20}
  local elapsed=0
  while (( elapsed < timeout_s )); do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      return 0
    fi
    sleep 1
    ((elapsed++))
  done
  return 1
}

if [[ "$#" -eq 0 || "$1" == "stack" ]]; then
  launch_stack
  exit $?
fi

if [[ "$1" == "gui" ]]; then
  export ROBOCONTROL_USE_ROS_BRIDGE=1
  STACK_PID=''

  cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM
    if [[ -n "$STACK_PID" ]] && kill -0 "$STACK_PID" 2>/dev/null; then
      kill "$STACK_PID" 2>/dev/null || true
      wait "$STACK_PID" 2>/dev/null || true
    fi
    exit "$exit_code"
  }

  trap cleanup EXIT INT TERM

  launch_stack &
  STACK_PID=$!

  if ! wait_for_topic "/roboarm/status" "${DOFBOT_GUI_WAIT_TIMEOUT_S:-20}"; then
    echo "Warning: timed out waiting for /roboarm/status; starting GUI anyway." >&2
  fi

  exec python3 "$OVERLAY_WS/RoboControl.py"
fi

exec "$@"
