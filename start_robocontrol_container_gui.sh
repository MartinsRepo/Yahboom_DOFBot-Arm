#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="${DOFBOT_IMAGE_NAME:-docker-dofbot_ros:latest}"
CONTAINER_NAME="${DOFBOT_CONTAINER_NAME:-dofbot_gui}"
if [[ -n "${DOFBOT_SERIAL_DEVICE:-}" ]]; then
  SERIAL_DEVICE="$DOFBOT_SERIAL_DEVICE"
elif [[ -d /dev/serial/by-id ]]; then
  SERIAL_DEVICE="$(ls -1 /dev/serial/by-id/* 2>/dev/null | head -n 1 || true)"
  if [[ -z "$SERIAL_DEVICE" ]]; then
    SERIAL_DEVICE="/dev/ttyUSB0"
  fi
else
  SERIAL_DEVICE="/dev/ttyUSB0"
fi

if [[ -n "${DOFBOT_CAMERA_DEVICE:-}" ]]; then
  CAMERA_DEVICE="$DOFBOT_CAMERA_DEVICE"
elif [[ -e /dev/video0 ]]; then
  CAMERA_DEVICE="/dev/video0"
else
  CAMERA_DEVICE="0"
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

echo "Using serial device: $SERIAL_DEVICE"
echo "Using camera device: $CAMERA_DEVICE"

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

CAMERA_DEVICE_ARGS=()
if [[ "$CAMERA_DEVICE" == /dev/* ]]; then
  CAMERA_DEVICE_ARGS=(--device "$CAMERA_DEVICE:$CAMERA_DEVICE")
fi

docker run --rm -it \
  --name "$CONTAINER_NAME" \
  --network host \
  --device "$SERIAL_DEVICE:$SERIAL_DEVICE" \
  "${CAMERA_DEVICE_ARGS[@]}" \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
  -e DOFBOT_SERIAL_DEVICE="$SERIAL_DEVICE" \
  -e DOFBOT_CAMERA_DEVICE="$CAMERA_DEVICE" \
  -e DOFBOT_CAMERA_WIDTH="${DOFBOT_CAMERA_WIDTH:-640}" \
  -e DOFBOT_CAMERA_HEIGHT="${DOFBOT_CAMERA_HEIGHT:-480}" \
  -e DOFBOT_CAMERA_FPS="${DOFBOT_CAMERA_FPS:-15}" \
  -e DOFBOT_SHOW_PREVIEW="${DOFBOT_SHOW_PREVIEW:-0}" \
  -e DOFBOT_FACE_DETECTION_MIN_CONF="${DOFBOT_FACE_DETECTION_MIN_CONF:-0.60}" \
  -e DOFBOT_ARM_LIB_DIR="${DOFBOT_ARM_LIB_DIR:-/opt/ros/overlay_ws/third_party/Arm_Lib}" \
  -e ENABLE_FACE_DETECTION="${ENABLE_FACE_DETECTION:-1}" \
  -e ENABLE_FACE_MESH="${ENABLE_FACE_MESH:-0}" \
  -e ENABLE_POSE="${ENABLE_POSE:-0}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$LOG_DIR:/opt/ros/overlay_ws/runtime_log" \
  "$IMAGE_NAME" gui