#!/usr/bin/env bash
# Real Wuji Hand 2 TCP server (run on Jetson / robot host with hands connected).
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${TELEOP_ROOT}/bridge${PYTHONPATH:+:${PYTHONPATH}}"

if systemctl is-active --quiet apex-tool 2>/dev/null; then
  echo "WARNING: apex-tool is running. Stop it if connection fails:"
  echo "  sudo systemctl stop apex-tool"
fi

CONTROL_HZ="${WUJI_CONTROL_HZ:-100}"
CUTOFF_HZ="${WUJI_CUTOFF_HZ:-5.0}"
MAX_SPEED="${WUJI_MAX_JOINT_SPEED:-2.0}"
PORT="${WUJI_BRIDGE_PORT:-9500}"

# Real hand requires wuji-sdk wheel (see setup.sh)
exec python3 -m wuji_manus_bridge \
  --host 0.0.0.0 \
  --port "${PORT}" \
  --control-hz "${CONTROL_HZ}" \
  --cutoff-hz "${CUTOFF_HZ}" \
  --max-joint-speed "${MAX_SPEED}" \
  "$@"
