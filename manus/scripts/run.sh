#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/scripts/env.sh"
PYTHON="${PYTHON:-$(command -v python3.10 || command -v python3)}"
PORT="${PORT:-9876}"

BRIDGE="${ROOT}/bridge/skeleton_bridge.out"
if [[ ! -x "${BRIDGE}" ]]; then
  echo "Building skeleton bridge..."
  make -C "${ROOT}/bridge" MANUS_SDK="${MANUS_SDK}"
fi

echo "Starting viewer (using ${PYTHON})..."
"${PYTHON}" "${ROOT}/viewer/hand_skeleton_viewer.py" --port "${PORT}" &
VIEWER_PID=$!

cleanup() {
  kill "${VIEWER_PID}" 2>/dev/null || true
  kill "${BRIDGE_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT

echo "Starting Manus skeleton bridge..."
"${BRIDGE}" --port "${PORT}" &
BRIDGE_PID=$!

wait "${VIEWER_PID}"
