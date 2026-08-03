#!/usr/bin/env bash
# Source before Manus SDK / ROS2 manus packages.
TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export TELEOP_ROOT
export MANUS_SDK="${MANUS_SDK:-${HOME}/ManusSDK}"
export LD_LIBRARY_PATH="${MANUS_SDK}/lib:${LD_LIBRARY_PATH:-}"

__manus_nounset=0
[[ $- == *u* ]] && __manus_nounset=1 && set +u

if [[ -f /opt/ros/humble/setup.bash ]]; then
  export PATH="/usr/bin:${PATH}"
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  export PATH="/usr/bin:${PATH}"
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
fi

ROS2_WS="${ROS2_WS:-${HOME}/ros2_ws}"
if [[ -f "${ROS2_WS}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ROS2_WS}/install/setup.bash"
fi

[[ $__manus_nounset -eq 1 ]] && set -u
unset __manus_nounset
