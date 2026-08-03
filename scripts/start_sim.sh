#!/usr/bin/env bash
# MuJoCo Wuji Hand **2** simulator + TCP bridge (:9500).
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "${TELEOP_ROOT}/scripts/activate_base.sh"
export WUJI_RETARGETING_ROOT="${WUJI_RETARGETING_ROOT:-${TELEOP_ROOT}/../wuji-retargeting}"
WUJI_RETARGETING_ROOT="$(realpath -m "${WUJI_RETARGETING_ROOT}")"
export WUJI_RETARGETING_ROOT
export WUJI_DESCRIPTION_ROOT="${WUJI_DESCRIPTION_ROOT:-${TELEOP_ROOT}/deps/wuji-description}"
WUJI_DESCRIPTION_ROOT="$(realpath -m "${WUJI_DESCRIPTION_ROOT}")"
export WUJI_DESCRIPTION_ROOT
SIDES="${SIDES:-left}"
if [[ "${SIDES}" != "left" && "${SIDES}" != "right" ]]; then
  echo "ERROR: SIDES must be left or right, got ${SIDES}" >&2
  exit 2
fi
HAND2_MODEL_ROOT="${WUJI_DESCRIPTION_ROOT}/hand2/hand2_beta1/body"
HAND2_MJCF="${HAND2_MODEL_ROOT}/mjcf/${SIDES}.xml"
BRIDGE="${TELEOP_ROOT}/bridge"
SIM="${TELEOP_ROOT}/sim"

if [[ ! -f "${HAND2_MJCF}" ]]; then
  echo "ERROR: pinned Hand2 Beta1 model missing at ${HAND2_MJCF}"
  echo "Set WUJI_DESCRIPTION_ROOT or run setup.sh."
  exit 1
fi

export PYTHONPATH="${BRIDGE}/x86:${BRIDGE}:${SIM}:${PYTHONPATH:-}"

HOST="${SIM_HOST:-127.0.0.1}"
PORT="${SIM_PORT:-9500}"
COMMAND_TIMEOUT_MS="${WUJI_COMMAND_TIMEOUT_MS:-200}"
EXTRA=()
if [[ "${HEADLESS:-0}" == "1" ]]; then
  EXTRA+=(--headless)
fi

echo "==> Starting MuJoCo Wuji Hand 2 sim (${SIDES}) on ${HOST}:${PORT}"
echo "    Official model: ${HAND2_MJCF}"
exec "${TELEOP_PYTHON}" -m wuji_hand_sim \
  --host "${HOST}" \
  --port "${PORT}" \
  --sides "${SIDES}" \
  --command-timeout-ms "${COMMAND_TIMEOUT_MS}" \
  "${EXTRA[@]}" \
  "$@"
