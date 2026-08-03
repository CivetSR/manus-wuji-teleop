#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-$(command -v python3.10 || command -v python3)}"
PORT="${PORT:-9876}"

echo "Starting mock skeleton publisher..."
"${PYTHON}" "${ROOT}/viewer/mock_bridge.py" --port "${PORT}" &
MOCK_PID=$!

cleanup() {
  kill "${MOCK_PID}" 2>/dev/null || true
  kill "${VIEWER_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 0.3
echo "Starting viewer UI (using ${PYTHON})..."
"${PYTHON}" "${ROOT}/viewer/hand_skeleton_viewer.py" --port "${PORT}" &
VIEWER_PID=$!

wait "${VIEWER_PID}"
