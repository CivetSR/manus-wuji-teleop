#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "==> Installing system packages for Manus SDK (integrated mode)"
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  libusb-1.0-0-dev \
  zlib1g-dev \
  libudev-dev \
  python3-pyqt5

echo "==> Installing udev rules for Manus dongle"
sudo cp "${ROOT}/config/70-manus-hid.rules" /etc/udev/rules.d/70-manus-hid.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

MANUS_SDK="${MANUS_SDK:-${HOME}/ManusSDK}"
if [[ -f "${MANUS_SDK}/include/ManusSDK.h" ]]; then
  echo "==> Manus SDK found at ${MANUS_SDK}"
  echo "==> Building skeleton bridge..."
  make -C "${ROOT}/bridge" MANUS_SDK="${MANUS_SDK}"
else
  echo "WARN: Manus SDK not found at ${MANUS_SDK}"
  echo "      Extract MANUS_Core_*_SDK.zip and copy SDKClient_Linux/ManusSDK to ~/ManusSDK"
fi

echo
echo "Setup complete."
echo "Next steps:"
echo "  source ${ROOT}/scripts/env.sh"
echo "  ${ROOT}/scripts/run_mock.sh          # demo without gloves"
echo "  ${ROOT}/scripts/run.sh               # real gloves + 3D viewer"
echo "  ros2 run manus_ros2 manus_data_publisher   # ROS2 topics (after env.sh)"
