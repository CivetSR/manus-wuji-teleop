#!/usr/bin/env bash
# Avoid incompatible globally installed ROS pytest plugins on mixed Python hosts.
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${TELEOP_ROOT}"
# shellcheck source=/dev/null
source "${TELEOP_ROOT}/scripts/activate_base.sh"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
exec "${TELEOP_PYTHON}" -m pytest -q "$@"
