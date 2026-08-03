#!/usr/bin/env bash
# MANUS/IK client -> local Hand2 or MuJoCo TCP backend.
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WUJI_BACKEND_HOST="${WUJI_BACKEND_HOST:-${ROBOT_HOST:-127.0.0.1}}"
export WUJI_BACKEND_PORT="${WUJI_BACKEND_PORT:-${ROBOT_PORT:-9500}}"

echo "==> Checking Wuji backend at ${WUJI_BACKEND_HOST}:${WUJI_BACKEND_PORT}..."
if ! timeout 2 bash -c "echo >/dev/tcp/${WUJI_BACKEND_HOST}/${WUJI_BACKEND_PORT}" 2>/dev/null; then
  echo "ERROR: Nothing listening on ${WUJI_BACKEND_HOST}:${WUJI_BACKEND_PORT}"
  echo "  Sim:  ${TELEOP_ROOT}/scripts/start_sim.sh"
  echo "  Hand2: ${TELEOP_ROOT}/scripts/start_hand2_backend.sh  (same x86 by default)"
  exit 1
fi

exec "${TELEOP_ROOT}/bridge/x86/run_teleop.sh" "$@"
