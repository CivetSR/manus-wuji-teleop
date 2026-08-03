#!/usr/bin/env bash
# Launch headless MuJoCo, run official IK through localhost TCP, verify state motion.
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "${TELEOP_ROOT}/scripts/activate_base.sh"
PORT="${SMOKE_PORT:-19500}"
SIDE="${SMOKE_SIDE:-left}"
BACKEND="${RETARGET_BACKEND:-sdk}"
LOG_FILE="$(mktemp -t manus-wuji-mujoco.XXXXXX.log)"
SIM_PID=""

cleanup() {
  if [[ -n "${SIM_PID}" ]]; then
    kill "${SIM_PID}" 2>/dev/null || true
    wait "${SIM_PID}" 2>/dev/null || true
  fi
  rm -f "${LOG_FILE}"
}
trap cleanup EXIT

HEADLESS=1 SIDES="${SIDE}" SIM_HOST=127.0.0.1 SIM_PORT="${PORT}" \
  "${TELEOP_ROOT}/scripts/start_sim.sh" >"${LOG_FILE}" 2>&1 &
SIM_PID=$!

for _ in $(seq 1 100); do
  if timeout 0.2 bash -c "echo >/dev/tcp/127.0.0.1/${PORT}" 2>/dev/null; then
    break
  fi
  if ! kill -0 "${SIM_PID}" 2>/dev/null; then
    echo "ERROR: headless MuJoCo backend exited during startup"
    cat "${LOG_FILE}"
    exit 1
  fi
  sleep 0.1
done

if ! kill -0 "${SIM_PID}" 2>/dev/null; then
  cat "${LOG_FILE}"
  exit 1
fi

export WUJI_RETARGETING_ROOT="${WUJI_RETARGETING_ROOT:-${TELEOP_ROOT}/../wuji-retargeting}"
export WUJI_DESCRIPTION_ROOT="${WUJI_DESCRIPTION_ROOT:-${TELEOP_ROOT}/deps/wuji-description}"
export PYTHONPATH="${TELEOP_ROOT}/bridge/x86:${TELEOP_ROOT}/bridge:${PYTHONPATH:-}"
"${TELEOP_PYTHON}" "${TELEOP_ROOT}/bridge/examples/ik_tcp_mujoco_smoke.py" \
  --host 127.0.0.1 \
  --port "${PORT}" \
  --side "${SIDE}" \
  --retarget-backend "${BACKEND}"
