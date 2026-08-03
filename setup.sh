#!/usr/bin/env bash
# One-shot environment setup for Manus + Wuji teleop (Ubuntu 22.04 recommended).
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${TELEOP_ROOT}"
CONDA_ROOT="${CONDA_ROOT:-${HOME}/miniconda3}"
PYPI_INDEX="${TELEOP_PYPI_INDEX:-https://pypi.tuna.tsinghua.edu.cn/simple}"

echo "==> [1/6] System packages"
sudo apt-get update
sudo apt-get install -y \
  build-essential git curl \
  libusb-1.0-0-dev zlib1g-dev libudev-dev \
  python3-pip python3-yaml \
  ros-humble-desktop ros-humble-rmw-cyclonedds-cpp 2>/dev/null || \
  sudo apt-get install -y build-essential git curl \
    libusb-1.0-0-dev zlib1g-dev libudev-dev python3-pip python3-yaml

echo "==> [2/6] Manus dongle udev rules"
sudo cp manus/config/70-manus-hid.rules /etc/udev/rules.d/70-manus-hid.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "==> [3/6] unified conda base Python 3.10"
# shellcheck source=/dev/null
source "${TELEOP_ROOT}/scripts/activate_base.sh"
"${TELEOP_PYTHON}" -m pip install -i "${PYPI_INDEX}" -r requirements.txt

echo "==> [4/6] official wuji-retargeting core (editable, no video extras)"
WUJI_RETARGETING_ROOT="${WUJI_RETARGETING_ROOT:-${TELEOP_ROOT}/../wuji-retargeting}"
WUJI_RETARGETING_ROOT="$(realpath -m "${WUJI_RETARGETING_ROOT}")"
if [[ ! -f "${WUJI_RETARGETING_ROOT}/pyproject.toml" ]]; then
  echo "ERROR: official checkout not found at ${WUJI_RETARGETING_ROOT}"
  echo "Clone it adjacent to this repo (with submodules), or set WUJI_RETARGETING_ROOT."
  exit 1
fi
"${TELEOP_PYTHON}" -m pip install -i "${PYPI_INDEX}" -e "${WUJI_RETARGETING_ROOT}"

echo "==> [5/6] pinned wuji-description v2026.8.3 model"
WUJI_DESCRIPTION_COMMIT="8271644a78d69ed9a4adcf9165d882c64ad33dfa"
WUJI_DESCRIPTION_ROOT="${WUJI_DESCRIPTION_ROOT:-${TELEOP_ROOT}/deps/wuji-description}"
if [[ -L "${WUJI_DESCRIPTION_ROOT}" ]]; then
  if [[ ! -e "${WUJI_DESCRIPTION_ROOT}" ]]; then
    echo "ERROR: WUJI_DESCRIPTION_ROOT is a broken symbolic link: ${WUJI_DESCRIPTION_ROOT}"
    exit 1
  fi
  echo "    Resolving existing model symlink: ${WUJI_DESCRIPTION_ROOT}"
fi
WUJI_DESCRIPTION_ROOT="$(realpath -m "${WUJI_DESCRIPTION_ROOT}")"
if [[ -e "${WUJI_DESCRIPTION_ROOT}" ]]; then
  if ! git -C "${WUJI_DESCRIPTION_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "ERROR: existing WUJI_DESCRIPTION_ROOT is not a git checkout: ${WUJI_DESCRIPTION_ROOT}"
    exit 1
  fi
  CHECKOUT_TOP="$(git -C "${WUJI_DESCRIPTION_ROOT}" rev-parse --show-toplevel)"
  if [[ "$(realpath -m "${CHECKOUT_TOP}")" != "${WUJI_DESCRIPTION_ROOT}" ]]; then
    echo "ERROR: WUJI_DESCRIPTION_ROOT is not the checkout root: ${WUJI_DESCRIPTION_ROOT}"
    echo "Actual checkout root: ${CHECKOUT_TOP}"
    exit 1
  fi
  DESCRIPTION_HEAD="$(git -C "${WUJI_DESCRIPTION_ROOT}" rev-parse HEAD)"
  if [[ "${DESCRIPTION_HEAD}" != "${WUJI_DESCRIPTION_COMMIT}" ]]; then
    echo "ERROR: existing wuji-description checkout has the wrong revision."
    echo "Expected: ${WUJI_DESCRIPTION_COMMIT}"
    echo "Actual:   ${DESCRIPTION_HEAD}"
    echo "Refusing to mutate an existing checkout; fix it explicitly and rerun setup."
    exit 1
  fi
  if [[ -n "$(git -C "${WUJI_DESCRIPTION_ROOT}" status --porcelain --untracked-files=no)" ]]; then
    echo "ERROR: existing wuji-description checkout has tracked modifications."
    echo "Refusing modified model assets at ${WUJI_DESCRIPTION_ROOT}"
    exit 1
  fi
else
  mkdir -p "$(dirname "${WUJI_DESCRIPTION_ROOT}")"
  git clone https://github.com/wuji-technology/wuji-description.git "${WUJI_DESCRIPTION_ROOT}"
  git -C "${WUJI_DESCRIPTION_ROOT}" checkout --detach "${WUJI_DESCRIPTION_COMMIT}"
fi
if [[ "$(git -C "${WUJI_DESCRIPTION_ROOT}" rev-parse HEAD)" != "${WUJI_DESCRIPTION_COMMIT}" ]]; then
  echo "ERROR: wuji-description pin verification failed at ${WUJI_DESCRIPTION_ROOT}"
  exit 1
fi
for asset in \
  hand2/hand2_beta1/body/urdf/left.urdf \
  hand2/hand2_beta1/body/urdf/right.urdf \
  hand2/hand2_beta1/body/mjcf/left.xml \
  hand2/hand2_beta1/body/mjcf/right.xml; do
  if [[ ! -f "${WUJI_DESCRIPTION_ROOT}/${asset}" ]]; then
    echo "ERROR: pinned Hand2 Beta1 model is incomplete: ${WUJI_DESCRIPTION_ROOT}/${asset}"
    exit 1
  fi
done
if [[ -L "${WUJI_DESCRIPTION_ROOT}/hand2/hand2_beta1/body" ]]; then
  echo "ERROR: pinned Hand2 Beta1 body must not be a symbolic link."
  exit 1
fi

echo "==> [6/6] Manus SDK + ROS2 (optional, for real gloves)"
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
  Terminal 2: ${TELEOP_ROOT}/scripts/run_ik_mujoco_smoke.sh

Full Manus teleop (real Hand2, same x86):
  Terminal 1: ${TELEOP_ROOT}/scripts/start_hand2_backend.sh
  Terminal 2: ${TELEOP_ROOT}/scripts/start_manus_teleop.sh

See CURSOR_DEPLOY.md for Cursor agent instructions.

EOF
