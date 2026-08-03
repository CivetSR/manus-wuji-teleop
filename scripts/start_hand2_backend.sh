#!/usr/bin/env bash
# Local TCP backend: x86 wuji-sdk -> Ethernet switch -> Wuji Hand 2.
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "${TELEOP_ROOT}/scripts/activate_base.sh"
export PYTHONPATH="${TELEOP_ROOT}/bridge${PYTHONPATH:+:${PYTHONPATH}}"

BIND="${WUJI_BACKEND_BIND:-127.0.0.1}"
PORT="${WUJI_BACKEND_PORT:-9500}"
SIDES="${WUJI_SIDES:-both}"
CONTROL_HZ="${WUJI_CONTROL_HZ:-100}"
CUTOFF_HZ="${WUJI_CUTOFF_HZ:-5.0}"
MAX_SPEED="${WUJI_MAX_JOINT_SPEED:-2.0}"
COMMAND_TIMEOUT_MS="${WUJI_COMMAND_TIMEOUT_MS:-200}"

EXTRA=()
[[ -n "${WUJI_LEFT_IP:-}" ]] && EXTRA+=(--left-ip "${WUJI_LEFT_IP}")
[[ -n "${WUJI_RIGHT_IP:-}" ]] && EXTRA+=(--right-ip "${WUJI_RIGHT_IP}")

echo "==> Starting local Hand2 backend on ${BIND}:${PORT}"
echo "    Network hands: sides=${SIDES}; explicit IPs are optional"
echo "    Topology: x86 -> Ethernet switch -> Wuji Hand 2"
if [[ "${BIND}" != "127.0.0.1" ]]; then
  echo "    NOTE: non-loopback bind enables an optional remote TCP client"
fi

exec "${TELEOP_PYTHON}" -m wuji_manus_bridge \
  --host "${BIND}" \
  --port "${PORT}" \
  --sides "${SIDES}" \
  --control-hz "${CONTROL_HZ}" \
  --cutoff-hz "${CUTOFF_HZ}" \
  --max-joint-speed "${MAX_SPEED}" \
  --command-timeout-ms "${COMMAND_TIMEOUT_MS}" \
  "${EXTRA[@]}" \
  "$@"
