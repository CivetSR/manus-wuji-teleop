#!/usr/bin/env bash
# Manus glove -> Wuji (sim default 127.0.0.1:9500, real robot set ROBOT_HOST).
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROBOT_HOST="${ROBOT_HOST:-127.0.0.1}"
export ROBOT_PORT="${ROBOT_PORT:-9500}"

echo "==> Checking Wuji endpoint at ${ROBOT_HOST}:${ROBOT_PORT}..."
if ! timeout 2 bash -c "echo >/dev/tcp/${ROBOT_HOST}/${ROBOT_PORT}" 2>/dev/null; then
  echo "ERROR: Nothing listening on ${ROBOT_HOST}:${ROBOT_PORT}"
  echo "  Sim:  ${TELEOP_ROOT}/scripts/start_sim.sh"
  echo "  Real: ${TELEOP_ROOT}/scripts/start_robot_server.sh  (on Jetson)"
  exit 1
fi

exec "${TELEOP_ROOT}/bridge/x86/run_teleop.sh" "$@"
