#!/usr/bin/env bash
# MuJoCo Wuji Hand **2** simulator + TCP bridge (:9500).
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WUJI_DESC="${TELEOP_ROOT}/deps/wuji-description"
HAND2_MJCF="${WUJI_DESC}/hand2/hand2_beta1/body/mjcf/left.xml"
BRIDGE="${TELEOP_ROOT}/bridge"
SIM="${TELEOP_ROOT}/sim"

if [[ ! -f "${HAND2_MJCF}" ]]; then
  echo "==> Cloning wuji-description (Hand 2 MJCF only)..."
  mkdir -p "${TELEOP_ROOT}/deps"
  git clone --depth 1 https://github.com/wuji-technology/wuji-description.git "${WUJI_DESC}"
fi

if [[ ! -f "${HAND2_MJCF}" ]]; then
  echo "ERROR: Hand 2 model missing at ${HAND2_MJCF}"
  echo "Do NOT use hand/body (Hand 1). See docs/JOINT_LAYOUT.md"
  exit 1
fi

if ! python3 -c "import mujoco" 2>/dev/null; then
  echo "==> Installing mujoco..."
  pip3 install -r "${SIM}/requirements.txt"
fi

export PYTHONPATH="${BRIDGE}:${SIM}:${PYTHONPATH:-}"

SIDES="${SIDES:-left}"
HOST="${SIM_HOST:-0.0.0.0}"
PORT="${SIM_PORT:-9500}"
EXTRA=()
if [[ "${HEADLESS:-0}" == "1" ]]; then
  EXTRA+=(--headless)
fi

echo "==> Starting MuJoCo Wuji Hand 2 sim (${SIDES}) on ${HOST}:${PORT}"
echo "    Model: ${HAND2_MJCF}"
exec python3 -m wuji_hand_sim \
  --host "${HOST}" \
  --port "${PORT}" \
  --sides "${SIDES}" \
  "${EXTRA[@]}" \
  "$@"
