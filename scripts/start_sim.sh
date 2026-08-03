#!/usr/bin/env bash
# MuJoCo Wuji Hand 2 simulator + TCP bridge (:9500).
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WUJI_DESC="${TELEOP_ROOT}/deps/wuji-description"
BRIDGE="${TELEOP_ROOT}/bridge"
SIM="${TELEOP_ROOT}/sim"

if [[ ! -d "${WUJI_DESC}/hand2" ]]; then
  echo "==> Cloning wuji-description (Hand2 MJCF)..."
  git clone --depth 1 https://github.com/wuji-technology/wuji-description.git "${WUJI_DESC}"
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

echo "==> Starting MuJoCo Wuji Hand sim (${SIDES}) on ${HOST}:${PORT}"
exec python3 -m wuji_hand_sim \
  --host "${HOST}" \
  --port "${PORT}" \
  --sides "${SIDES}" \
  "${EXTRA[@]}" \
  "$@"
