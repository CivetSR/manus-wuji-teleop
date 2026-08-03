#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/scripts/env.sh"

PAIR="${ROOT}/bridge/pair_gloves.out"
if [[ ! -x "${PAIR}" ]]; then
  make -C "${ROOT}/bridge" pair MANUS_SDK="${MANUS_SDK}"
fi

exec "${PAIR}"
