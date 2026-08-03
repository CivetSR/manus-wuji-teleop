#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/scripts/env.sh"

BIN="${ROOT}/bridge/haptic_cycle.out"
if [[ ! -x "${BIN}" ]]; then
  make -C "${ROOT}/bridge" haptic MANUS_SDK="${MANUS_SDK}"
fi

exec "${BIN}" "$@"
