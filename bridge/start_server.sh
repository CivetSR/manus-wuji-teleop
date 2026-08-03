#!/usr/bin/env bash
# Compatibility entry point for the local Ethernet Hand2 backend.
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${TELEOP_ROOT}/scripts/start_hand2_backend.sh" "$@"
