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
    serial_device:="${DOFBOT_SERIAL_DEVICE:-/dev/ttyUSB0}" \
    arm_lib_dir:="${DOFBOT_ARM_LIB_DIR:-}" \
    camera_device:="${DOFBOT_CAMERA_DEVICE:-0}" \
    camera_width:="${DOFBOT_CAMERA_WIDTH:-640}" \
    camera_height:="${DOFBOT_CAMERA_HEIGHT:-480}" \
    camera_fps:="${DOFBOT_CAMERA_FPS:-15}" \
    show_preview:="${DOFBOT_SHOW_PREVIEW:-0}" \
    face_detection_min_conf:="${DOFBOT_FACE_DETECTION_MIN_CONF:-0.60}" \
    enable_face_detection:="${ENABLE_FACE_DETECTION:-1}" \
    enable_face_mesh:="${ENABLE_FACE_MESH:-0}" \
    enable_pose:="${ENABLE_POSE:-0}"
}

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
