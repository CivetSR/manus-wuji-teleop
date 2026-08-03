#!/usr/bin/env bash
# Start full Manus -> Wuji teleop on x86 (manus_data_publisher + TCP bridge).
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=/dev/null
source "${TELEOP_ROOT}/scripts/activate_base.sh"
X86="${TELEOP_ROOT}/bridge/x86"
ENV_SH="${TELEOP_ROOT}/manus/scripts/env.sh"
export WUJI_RETARGETING_ROOT="${WUJI_RETARGETING_ROOT:-${TELEOP_ROOT}/../wuji-retargeting}"
export WUJI_DESCRIPTION_ROOT="${WUJI_DESCRIPTION_ROOT:-${TELEOP_ROOT}/deps/wuji-description}"
export PYTHONPATH="${X86}:${TELEOP_ROOT}/bridge${PYTHONPATH:+:${PYTHONPATH}}"

if [[ -f "${ENV_SH}" ]]; then
  # shellcheck source=/dev/null
  source "${ENV_SH}"
fi

export WUJI_BACKEND_HOST="${WUJI_BACKEND_HOST:-${ROBOT_HOST:-127.0.0.1}}"
export WUJI_BACKEND_PORT="${WUJI_BACKEND_PORT:-${ROBOT_PORT:-9500}}"

MANUS_PUB=""
if pgrep -f manus_data_publisher >/dev/null 2>&1; then
  echo "==> manus_data_publisher already running"
else
  echo "==> Starting manus_data_publisher (Manus SDK)..."
  ros2 run manus_ros2 manus_data_publisher &
  MANUS_PUB=$!
  sleep 2
fi

echo "==> Waiting for /manus_glove_* topics (wear gloves and move fingers)..."
for _ in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -q '/manus_glove_[0-9]\+$'; then
    echo "==> Manus glove topic ready"
    break
  fi
  sleep 1
done
if ! ros2 topic list 2>/dev/null | grep -q '/manus_glove_[0-9]\+$'; then
  echo "WARNING: No /manus_glove_* topic yet — bridge will still start but Wuji won't move until data flows"
fi

cleanup() {
  [[ -n "${MANUS_PUB}" ]] && kill "${MANUS_PUB}" 2>/dev/null || true
}
trap cleanup EXIT

echo "==> Starting MANUS/IK client -> backend (${WUJI_BACKEND_HOST}:${WUJI_BACKEND_PORT})"
exec "${TELEOP_PYTHON}" "${X86}/manus_wuji_bridge.py" \
  --host "${WUJI_BACKEND_HOST}" \
  --port "${WUJI_BACKEND_PORT}" \
  "$@"
