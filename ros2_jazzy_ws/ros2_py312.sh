#!/usr/bin/env bash
set -eo pipefail

# Run ROS2 CLI using host Python 3.12 plus locally copied host libs.
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_PY="/run/host/usr/bin/python3.12"
ROS2_ENTRY="/opt/ros/jazzy/bin/ros2"

if [[ ! -x "$HOST_PY" ]]; then
  echo "Error: $HOST_PY not found or not executable." >&2
  exit 1
fi

if [[ ! -f "$ROS2_ENTRY" ]]; then
  echo "Error: $ROS2_ENTRY not found." >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
export LD_LIBRARY_PATH="$WS_DIR/.host-libs:${LD_LIBRARY_PATH:-}"

exec "$HOST_PY" "$ROS2_ENTRY" "$@"
