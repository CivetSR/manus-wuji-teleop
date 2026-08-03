#!/usr/bin/env bash
# Start Wuji Hand 2 <-> Manus bridge TCP server on the robot host.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
export PYTHONPATH="${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

# Hand Ethernet (eth10 -> 192.168.1.x)
if [[ -x /home/nvidia/srworkspace/teleop_setup/setup_wuji_hand2_network.sh ]]; then
  bash /home/nvidia/srworkspace/teleop_setup/setup_wuji_hand2_network.sh || true
fi

# Avoid fighting Apex tool for the same Hand sockets
if systemctl is-active --quiet apex-tool 2>/dev/null; then
  echo "WARNING: apex-tool is running. Stop it if connection fails:"
  echo "  sudo systemctl stop apex-tool"
fi

# Defaults match Wuji official tutorial (LowPass 5 Hz, 100 Hz control)
CONTROL_HZ="${WUJI_CONTROL_HZ:-100}"
CUTOFF_HZ="${WUJI_CUTOFF_HZ:-5.0}"
MAX_SPEED="${WUJI_MAX_JOINT_SPEED:-2.0}"
PORT="${WUJI_BRIDGE_PORT:-9500}"

exec python3 -m wuji_manus_bridge \
  --host 0.0.0.0 \
  --port "${PORT}" \
  --control-hz "${CONTROL_HZ}" \
  --cutoff-hz "${CUTOFF_HZ}" \
  --max-joint-speed "${MAX_SPEED}" \
  "$@"
