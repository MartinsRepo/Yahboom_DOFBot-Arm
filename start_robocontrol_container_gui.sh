#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="${DOFBOT_IMAGE_NAME:-docker-dofbot_ros:latest}"
CONTAINER_NAME="${DOFBOT_CONTAINER_NAME:-dofbot_gui}"
if [[ -n "${DOFBOT_SERIAL_DEVICE:-}" ]]; then
  SERIAL_DEVICE="$DOFBOT_SERIAL_DEVICE"
  if [[ "${SERIAL_DEVICE,,}" == "auto" || "${SERIAL_DEVICE,,}" == "default" ]]; then
    SERIAL_DEVICE=""
  fi
elif [[ -d /dev/serial/by-id ]]; then
  SERIAL_DEVICE="$(ls -1 /dev/serial/by-id/* 2>/dev/null | head -n 1 || true)"
  if [[ -z "$SERIAL_DEVICE" ]]; then
    SERIAL_DEVICE="/dev/ttyUSB0"
  fi
else
  SERIAL_DEVICE="/dev/ttyUSB0"
fi

if [[ -z "$SERIAL_DEVICE" ]]; then
  if [[ -d /dev/serial/by-id ]]; then
    SERIAL_DEVICE="$(ls -1 /dev/serial/by-id/* 2>/dev/null | head -n 1 || true)"
  fi
  if [[ -z "$SERIAL_DEVICE" ]]; then
    SERIAL_DEVICE="/dev/ttyUSB0"
  fi
fi

# Docker --device is most reliable with the real character device path.
if [[ -n "$SERIAL_DEVICE" && -e "$SERIAL_DEVICE" ]]; then
  RESOLVED_SERIAL_DEVICE="$(readlink -f "$SERIAL_DEVICE" 2>/dev/null || true)"
  if [[ -n "$RESOLVED_SERIAL_DEVICE" && -e "$RESOLVED_SERIAL_DEVICE" ]]; then
    SERIAL_DEVICE="$RESOLVED_SERIAL_DEVICE"
  fi
fi

if [[ -n "${DOFBOT_CAMERA_DEVICE:-}" ]]; then
  CAMERA_DEVICE="$DOFBOT_CAMERA_DEVICE"
else
  CAMERA_EXCLUDE_REGEX="${DOFBOT_CAMERA_EXCLUDE_REGEX:-lifecam|microsoft|hd-3000}"
  CAMERA_CANDIDATES=()

  if [[ -d /dev/v4l/by-id ]]; then
    while IFS= read -r cam_path; do
      [[ -n "$cam_path" ]] || continue
      CAMERA_CANDIDATES+=("$cam_path")
    done < <(ls -1 /dev/v4l/by-id/* 2>/dev/null || true)
  fi

  if [[ ${#CAMERA_CANDIDATES[@]} -eq 0 ]]; then
    while IFS= read -r cam_path; do
      [[ -n "$cam_path" ]] || continue
      CAMERA_CANDIDATES+=("$cam_path")
    done < <(ls -1 /dev/video* 2>/dev/null || true)
  fi

  CAMERA_DEVICE=""

  for candidate in "${CAMERA_CANDIDATES[@]}"; do
    candidate_lower="$(echo "$candidate" | tr '[:upper:]' '[:lower:]')"
    if ! [[ "$candidate_lower" =~ $CAMERA_EXCLUDE_REGEX ]]; then
      CAMERA_DEVICE="$candidate"
      break
    fi
  done

  if [[ -z "$CAMERA_DEVICE" && ${#CAMERA_CANDIDATES[@]} -gt 0 ]]; then
    CAMERA_DEVICE="${CAMERA_CANDIDATES[0]}"
    echo "Warning: no preferred robot camera found; falling back to '$CAMERA_DEVICE'." >&2
  fi

  if [[ -z "$CAMERA_DEVICE" ]]; then
    CAMERA_DEVICE="0"
  fi
fi

if [[ -n "$CAMERA_DEVICE" && "$CAMERA_DEVICE" == /dev/* && -e "$CAMERA_DEVICE" ]]; then
  RESOLVED_CAMERA_DEVICE="$(readlink -f "$CAMERA_DEVICE" 2>/dev/null || true)"
  if [[ -n "$RESOLVED_CAMERA_DEVICE" && -e "$RESOLVED_CAMERA_DEVICE" ]]; then
    CAMERA_DEVICE="$RESOLVED_CAMERA_DEVICE"
  fi
fi

if [[ -n "${DOFBOT_AUDIO_DEVICE:-}" ]]; then
  AUDIO_DEVICE="$DOFBOT_AUDIO_DEVICE"
elif [[ -d /dev/snd ]]; then
  AUDIO_DEVICE="/dev/snd"
else
  AUDIO_DEVICE=""
fi

LOG_DIR="$WS_DIR/docker_data/log"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "Error: DISPLAY is not set; X11 forwarding cannot be configured." >&2
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "Error: docker is not installed or not in PATH." >&2
  exit 1
fi

mkdir -p "$LOG_DIR"

if [[ "$SERIAL_DEVICE" == /dev/* && ! -e "$SERIAL_DEVICE" ]]; then
  echo "Error: serial device '$SERIAL_DEVICE' does not exist on host." >&2
  echo "Set DOFBOT_SERIAL_DEVICE to the correct path (for example /dev/ttyUSB0 or /dev/serial/by-id/...)." >&2
  exit 1
fi

if [[ "$CAMERA_DEVICE" == /dev/* && ! -e "$CAMERA_DEVICE" ]]; then
  echo "Warning: camera device '$CAMERA_DEVICE' does not exist on host; camera stream may fail." >&2
fi

if [[ -n "$AUDIO_DEVICE" && "$AUDIO_DEVICE" == /dev/* && ! -e "$AUDIO_DEVICE" ]]; then
  echo "Warning: audio device '$AUDIO_DEVICE' does not exist on host; speech input will be disabled." >&2
fi

echo "Using serial device: $SERIAL_DEVICE"
echo "Using camera device: $CAMERA_DEVICE"
if [[ -n "$AUDIO_DEVICE" ]]; then
  echo "Using audio device: $AUDIO_DEVICE"
fi

docker build \
  --build-arg FROM_IMAGE="${FROM_IMAGE:-ros:jazzy-ros-base}" \
  --build-arg OVERLAY_WS=/opt/ros/overlay_ws \
  -f "$WS_DIR/docker/Dockerfile" \
  -t "$IMAGE_NAME" \
  "$WS_DIR"

if command -v xhost >/dev/null 2>&1; then
  xhost +si:localuser:root >/dev/null
fi

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if command -v xhost >/dev/null 2>&1; then
    xhost -si:localuser:root >/dev/null || true
  fi
  exit "$exit_code"
}

trap cleanup EXIT INT TERM

EXISTING_CONTAINER_ID="$(docker ps -aq --filter "name=^/${CONTAINER_NAME}$" || true)"
if [[ -n "$EXISTING_CONTAINER_ID" ]]; then
  echo "Removing stale container: $CONTAINER_NAME ($EXISTING_CONTAINER_ID)"
  docker rm -f "$EXISTING_CONTAINER_ID" >/dev/null 2>&1 || true
fi

SERIAL_DEVICE_ARGS=(--device "$SERIAL_DEVICE:$SERIAL_DEVICE")
for extra_serial in /dev/ttyUSB* /dev/ttyACM*; do
  [[ -e "$extra_serial" ]] || continue
  if [[ "$extra_serial" != "$SERIAL_DEVICE" ]]; then
    SERIAL_DEVICE_ARGS+=(--device "$extra_serial:$extra_serial")
  fi
done

echo "Mapped serial devices into container:"
for ((i = 1; i < ${#SERIAL_DEVICE_ARGS[@]}; i += 2)); do
  echo "  ${SERIAL_DEVICE_ARGS[i]}"
done

CAMERA_DEVICE_ARGS=()
if [[ "$CAMERA_DEVICE" == /dev/* ]]; then
  CAMERA_DEVICE_ARGS=(--device "$CAMERA_DEVICE:$CAMERA_DEVICE")
fi

AUDIO_DEVICE_ARGS=()
if [[ -n "$AUDIO_DEVICE" && "$AUDIO_DEVICE" == /dev/* && -e "$AUDIO_DEVICE" ]]; then
  AUDIO_DEVICE_ARGS=(--device "$AUDIO_DEVICE:$AUDIO_DEVICE")
fi

docker run --rm -it \
  --name "$CONTAINER_NAME" \
  --network host \
  "${SERIAL_DEVICE_ARGS[@]}" \
  "${CAMERA_DEVICE_ARGS[@]}" \
  "${AUDIO_DEVICE_ARGS[@]}" \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
  -e DOFBOT_SERIAL_DEVICE="$SERIAL_DEVICE" \
  -e DOFBOT_CAMERA_DEVICE="$CAMERA_DEVICE" \
  -e DOFBOT_AUDIO_DEVICE="${AUDIO_DEVICE:-}" \
  -e DOFBOT_CAMERA_WIDTH="${DOFBOT_CAMERA_WIDTH:-640}" \
  -e DOFBOT_CAMERA_HEIGHT="${DOFBOT_CAMERA_HEIGHT:-480}" \
  -e DOFBOT_CAMERA_FPS="${DOFBOT_CAMERA_FPS:-15}" \
  -e DOFBOT_SHOW_PREVIEW="${DOFBOT_SHOW_PREVIEW:-0}" \
  -e DOFBOT_FACE_DETECTION_MIN_CONF="${DOFBOT_FACE_DETECTION_MIN_CONF:-0.60}" \
  -e DOFBOT_ARM_LIB_DIR="${DOFBOT_ARM_LIB_DIR:-/opt/ros/overlay_ws/third_party/Arm_Lib}" \
  -e ENABLE_FACE_DETECTION="${ENABLE_FACE_DETECTION:-1}" \
  -e ENABLE_FACE_MESH="${ENABLE_FACE_MESH:-0}" \
  -e ENABLE_POSE="${ENABLE_POSE:-0}" \
  -e ENABLE_LLM_CONTROLLER="${ENABLE_LLM_CONTROLLER:-0}" \
  -e ENABLE_ARM_AGENT_TOOLS="${ENABLE_ARM_AGENT_TOOLS:-1}" \
  -e DOFBOT_AGENT_TOOLS_ENABLED="${DOFBOT_AGENT_TOOLS_ENABLED:-${ENABLE_ARM_AGENT_TOOLS:-1}}" \
  -e DOFBOT_TOOL_MAX_DURATION_MS="${DOFBOT_TOOL_MAX_DURATION_MS:-1200}" \
  -e DOFBOT_TOOL_ALLOWED_ACTIONS="${DOFBOT_TOOL_ALLOWED_ACTIONS:-}" \
  -e DOFBOT_CONTROL_MODE="${DOFBOT_CONTROL_MODE:-GUI}" \
  -e DOFBOT_STRICT_SAFETY="${DOFBOT_STRICT_SAFETY:-1}" \
  -e DOFBOT_COMMAND_RATE_LIMIT_HZ="${DOFBOT_COMMAND_RATE_LIMIT_HZ:-8.0}" \
  -e DOFBOT_LLM_STALE_TIMEOUT_S="${DOFBOT_LLM_STALE_TIMEOUT_S:-2.0}" \
  -e DOFBOT_MANUAL_OVERRIDE_WINDOW_S="${DOFBOT_MANUAL_OVERRIDE_WINDOW_S:-1.5}" \
  -e ENABLE_SPEECH_CONTROLLER="${ENABLE_SPEECH_CONTROLLER:-0}" \
  -e DOFBOT_VOSK_MODEL_DIR="${DOFBOT_VOSK_MODEL_DIR:-/opt/ros/overlay_ws/models/vosk-model-small-de-zamia-0.3}" \
  -e DOFBOT_SPEECH_DEVICE="${DOFBOT_SPEECH_DEVICE:-default}" \
  -e DOFBOT_SPEECH_SAMPLE_RATE="${DOFBOT_SPEECH_SAMPLE_RATE:-16000}" \
  -e DOFBOT_SPEECH_BLOCKSIZE="${DOFBOT_SPEECH_BLOCKSIZE:-2000}" \
  -e DOFBOT_SPEECH_LANGUAGE="${DOFBOT_SPEECH_LANGUAGE:-de}" \
  -e DOFBOT_SPEECH_TOPIC="${DOFBOT_SPEECH_TOPIC:-roboarm/speech_input}" \
  -e DOFBOT_SPEECH_FLUSH_SILENCE_S="${DOFBOT_SPEECH_FLUSH_SILENCE_S:-0.4}" \
  -e DOFBOT_WAKE_WORD="${DOFBOT_WAKE_WORD:-martin}" \
  -e DOFBOT_WAKE_WORD_TIMEOUT_S="${DOFBOT_WAKE_WORD_TIMEOUT_S:-4.0}" \
  -e DOFBOT_LLM_CONFIG_PATH="${DOFBOT_LLM_CONFIG_PATH:-/opt/ros/overlay_ws/config/llm_controller.json}" \
  -e DOFBOT_LLM_PROVIDER="${DOFBOT_LLM_PROVIDER:-ollama}" \
  -e DOFBOT_LLM_ENDPOINT="${DOFBOT_LLM_ENDPOINT:-}" \
  -e DOFBOT_LLM_API_TOKEN="${DOFBOT_LLM_API_TOKEN:-}" \
  -e DOFBOT_OLLAMA_BASE_URL="${DOFBOT_OLLAMA_BASE_URL:-http://127.0.0.1:11434}" \
  -e DOFBOT_OLLAMA_MODEL="${DOFBOT_OLLAMA_MODEL:-llama3.1:8b}" \
  -e DOFBOT_LLM_SYSTEM_PROMPT="${DOFBOT_LLM_SYSTEM_PROMPT:-}" \
  -e DOFBOT_LLM_LOOP_PERIOD_S="${DOFBOT_LLM_LOOP_PERIOD_S:-0.15}" \
  -e DOFBOT_LLM_REQUEST_TIMEOUT_S="${DOFBOT_LLM_REQUEST_TIMEOUT_S:-20.0}" \
  -e DOFBOT_LLM_REQUEST_RETRIES="${DOFBOT_LLM_REQUEST_RETRIES:-3}" \
  -e DOFBOT_LLM_RETRY_BACKOFF_S="${DOFBOT_LLM_RETRY_BACKOFF_S:-1.0}" \
  -e DOFBOT_LLM_FALLBACK_ON_ERROR="${DOFBOT_LLM_FALLBACK_ON_ERROR:-1}" \
  -e DOFBOT_VOICE_OUTPUT_ENABLED="${DOFBOT_VOICE_OUTPUT_ENABLED:-0}" \
  -e DOFBOT_VOICE_OUTPUT_VOICE="${DOFBOT_VOICE_OUTPUT_VOICE:-}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$LOG_DIR:/opt/ros/overlay_ws/runtime_log" \
  "$IMAGE_NAME" gui
