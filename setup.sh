#!/usr/bin/env bash
# One-shot environment setup for Manus + Wuji teleop (Ubuntu 22.04 recommended).
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${TELEOP_ROOT}"

echo "==> [1/5] System packages"
sudo apt-get update
sudo apt-get install -y \
  build-essential git curl \
  libusb-1.0-0-dev zlib1g-dev libudev-dev \
  python3-pip python3-yaml \
  ros-humble-desktop ros-humble-rmw-cyclonedds-cpp 2>/dev/null || \
  sudo apt-get install -y build-essential git curl \
    libusb-1.0-0-dev zlib1g-dev libudev-dev python3-pip python3-yaml

echo "==> [2/5] Manus dongle udev rules"
sudo cp manus/config/70-manus-hid.rules /etc/udev/rules.d/70-manus-hid.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "==> [3/5] Python deps (MuJoCo sim + bridge)"
pip3 install -r requirements.txt

echo "==> [4/5] wuji-description (Hand2 MJCF for MuJoCo)"
if [[ ! -d deps/wuji-description/hand2 ]]; then
  git clone --depth 1 https://github.com/wuji-technology/wuji-description.git deps/wuji-description
fi

echo "==> [5/5] Manus SDK + ROS2 (optional, for real gloves)"
MANUS_SDK="${MANUS_SDK:-${HOME}/ManusSDK}"
if [[ -f "${MANUS_SDK}/include/ManusSDK.h" ]]; then
  echo "    Manus SDK found — building ROS2 packages..."
  bash manus/scripts/build_ros2.sh
  make -C manus/bridge MANUS_SDK="${MANUS_SDK}" || true
else
  echo "    SKIP: No Manus SDK at ${MANUS_SDK}"
  echo "    For real gloves: install Manus Core SDK to ~/ManusSDK, then re-run:"
  echo "      bash manus/scripts/build_ros2.sh"
fi

chmod +x scripts/*.sh bridge/x86/*.sh manus/scripts/*.sh 2>/dev/null || true

cat <<EOF

Setup complete.

Sim-only (no gloves):
  Terminal 1: ${TELEOP_ROOT}/scripts/start_sim.sh
  Terminal 2: python3 bridge/examples/x86_client_stub.py --host 127.0.0.1 --side left --demo

Full Manus teleop (sim):
  Terminal 1: ${TELEOP_ROOT}/scripts/start_sim.sh
  Terminal 2: ${TELEOP_ROOT}/scripts/start_manus_teleop.sh

See CURSOR_DEPLOY.md for Cursor agent instructions.

EOF
