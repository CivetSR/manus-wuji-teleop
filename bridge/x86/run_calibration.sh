#!/usr/bin/env bash
# Start the Manus ROS2 publisher and interactive calibration tool.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f /home/omen/srworkspace/manus-hand-viz/scripts/env.sh ]]; then
  # shellcheck source=/dev/null
  source /home/omen/srworkspace/manus-hand-viz/scripts/env.sh
else
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  # shellcheck source=/dev/null
  source "${HOME}/ros2_ws/install/setup.bash"
fi

MANUS_PUB=""
if pgrep -f manus_data_publisher >/dev/null 2>&1; then
  echo "==> manus_data_publisher already running"
else
  echo "==> Starting manus_data_publisher..."
  ros2 run manus_ros2 manus_data_publisher &
  MANUS_PUB=$!
  sleep 2
fi

cleanup() {
  [[ -n "${MANUS_PUB}" ]] && kill "${MANUS_PUB}" 2>/dev/null || true
}
trap cleanup EXIT

python3 "${ROOT}/x86/calibrate_manus.py" "$@"
