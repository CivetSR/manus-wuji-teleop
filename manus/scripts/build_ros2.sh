#!/usr/bin/env bash
# Build manus_ros2 + manus_ros2_msgs into ~/ros2_ws (or $ROS2_WS).
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS2_WS="${ROS2_WS:-${HOME}/ros2_ws}"
MANUS_SDK="${MANUS_SDK:-${HOME}/ManusSDK}"

if [[ ! -f "${MANUS_SDK}/include/ManusSDK.h" ]]; then
  echo "ERROR: Manus SDK not found at ${MANUS_SDK}"
  echo "Install: extract Manus SDK and copy SDKClient_Linux/ManusSDK to ~/ManusSDK"
  exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]] && [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: ROS 2 not found. Install ROS 2 Humble (Ubuntu 22.04) first."
  exit 1
fi

mkdir -p "${ROS2_WS}/src"
ln -sfn "${TELEOP_ROOT}/manus/ros2" "${ROS2_WS}/src/manus_ros2"
ln -sfn "${TELEOP_ROOT}/manus/manus_ros2_msgs" "${ROS2_WS}/src/manus_ros2_msgs"

# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
export MANUS_SDK
cd "${ROS2_WS}"
rosdep install --from-paths src --ignore-src -r -y || true
colcon build --packages-select manus_ros2_msgs manus_ros2
echo "Built. Run: source ${ROS2_WS}/install/setup.bash"
