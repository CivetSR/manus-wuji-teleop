#!/usr/bin/env bash
# Source this file to select the single supported teleop interpreter.

TELEOP_ROOT="${TELEOP_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
CONDA_ROOT="${CONDA_ROOT:-${HOME}/miniconda3}"
CONDA_SH="${CONDA_ROOT}/etc/profile.d/conda.sh"
_TELEOP_NOUNSET_WAS_ON=0
if [[ $- == *u* ]]; then
  _TELEOP_NOUNSET_WAS_ON=1
  set +u
fi

if [[ ! -f "${CONDA_SH}" ]]; then
  echo "ERROR: conda initialization script not found: ${CONDA_SH}" >&2
  (( _TELEOP_NOUNSET_WAS_ON )) && set -u
  unset _TELEOP_NOUNSET_WAS_ON
  return 1
fi

# shellcheck source=/dev/null
source "${CONDA_SH}"
conda activate base
export TELEOP_PYTHON="${CONDA_PREFIX}/bin/python"
export PYTHONNOUSERSITE=1

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: ROS Humble setup not found at /opt/ros/humble/setup.bash" >&2
  (( _TELEOP_NOUNSET_WAS_ON )) && set -u
  unset _TELEOP_NOUNSET_WAS_ON
  return 1
fi

if ! "${TELEOP_PYTHON}" -c \
  'import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 10) else 1)'; then
  echo "ERROR: conda base must use Python 3.10: ${TELEOP_PYTHON}" >&2
  (( _TELEOP_NOUNSET_WAS_ON )) && set -u
  unset _TELEOP_NOUNSET_WAS_ON
  return 1
fi

(( _TELEOP_NOUNSET_WAS_ON )) && set -u
unset _TELEOP_NOUNSET_WAS_ON
