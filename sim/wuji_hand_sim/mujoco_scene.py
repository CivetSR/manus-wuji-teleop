"""MuJoCo scene: Wuji Hand **2** only (hand2/hand2_beta1 MJCF)."""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Optional

import mujoco
import mujoco.viewer

log = logging.getLogger("wuji_hand_sim.mujoco")

TELEOP_ROOT = Path(__file__).resolve().parents[2]
HAND2_REL = Path("hand2/hand2_beta1/body/mjcf")


def _find_wuji_description() -> Path:
    candidates = [
        TELEOP_ROOT / "deps" / "wuji-description",
        TELEOP_ROOT.parent / "wuji-description",
    ]
    for root in candidates:
        if (root / HAND2_REL / "left.xml").is_file():
            return root
    raise FileNotFoundError(
        "Wuji Hand 2 MJCF not found. Run: bash setup.sh\n"
        "  or: git clone https://github.com/wuji-technology/wuji-description.git "
        f"{TELEOP_ROOT / 'deps' / 'wuji-description'}"
    )


WUJI_DESC = _find_wuji_description()
DEFAULT_MJCF = {
    "left": WUJI_DESC / HAND2_REL / "left.xml",
    "right": WUJI_DESC / HAND2_REL / "right.xml",
}

# Reject Hand 1 paths explicitly
_FORBIDDEN_PARTS = ("/hand/body/", "/hand/body-with-soft/", "finger1_joint")


def mjcf_for_side(side: str) -> Path:
    path = DEFAULT_MJCF.get(side.lower())
    if path is None or not path.is_file():
        raise FileNotFoundError(f"Hand 2 MJCF for side={side!r} not found at {path}")
    resolved = str(path.resolve())
    if any bad in resolved for bad in _FORBIDDEN_PARTS):
        raise RuntimeError(f"Refusing Hand 1 model path: {path}")
    if "hand2" not in resolved:
        raise RuntimeError(f"Expected hand2 in path, got: {path}")
    return path


def _assert_hand2_model(model: mujoco.MjModel, mjcf: Path) -> None:
    if model.nu != 20:
        log.warning("Hand 2 expects 20 actuators, model has nu=%d", model.nu)
    # MuJoCo stores model name in model names buffer; check XML path as primary guard
    if "hand2" not in str(mjcf):
        raise RuntimeError(f"Not a Hand 2 MJCF: {mjcf}")
    log.info("Loaded Wuji Hand 2: %s (nu=%d)", mjcf.name, model.nu)


class MujocoScene:
    """Single Hand 2 model. Protocol index i -> actuator i."""

    NUM_ACTUATORS = 20

    def __init__(self, side: str, *, headless: bool = False) -> None:
        self.side = side.lower()
        self.headless = headless
        mjcf = mjcf_for_side(self.side)
        log.info("Loading Hand 2 MJCF: %s", mjcf)
        self.model = mujoco.MjModel.from_xml_path(str(mjcf))
        _assert_hand2_model(self.model, mjcf)
        self.data = mujoco.MjData(self.model)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sim_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._stop.clear()
        target = self._headless_loop if self.headless else self._viewer_loop
        self._sim_thread = threading.Thread(target=target, name="mujoco-sim", daemon=True)
        self._sim_thread.start()
        log.info("MuJoCo Hand 2 scene started (%s, headless=%s)", self.side, self.headless)

    def stop(self) -> None:
        self._stop.set()
        if self._sim_thread:
            self._sim_thread.join(timeout=2.0)

    def set_ctrl(self, position: list[float]) -> None:
        with self._lock:
            n = min(len(position), self.model.nu)
            for i in range(n):
                self.data.ctrl[i] = float(position[i])

    def read_joint_state(self) -> tuple[list[float], list[float]]:
        with self._lock:
            pos = [float(self.data.qpos[i]) for i in range(min(20, self.model.nq))]
            vel = [float(self.data.qvel[i]) for i in range(min(20, self.model.nv))]
        while len(pos) < 20:
            pos.append(0.0)
        while len(vel) < 20:
            vel.append(0.0)
        return pos, vel

    def _step_once(self) -> None:
        mujoco.mj_step(self.model, self.data)

    def _headless_loop(self) -> None:
        dt = self.model.opt.timestep
        period = max(dt, 0.001)
        next_t = time.monotonic()
        while not self._stop.is_set():
            next_t += period
            with self._lock:
                self._step_once()
            sleep_t = next_t - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.monotonic()

    def _viewer_loop(self) -> None:
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                dt = self.model.opt.timestep
                while viewer.is_running() and not self._stop.is_set():
                    t0 = time.monotonic()
                    with self._lock:
                        self._step_once()
                        viewer.sync()
                    sleep_t = dt - (time.monotonic() - t0)
                    if sleep_t > 0:
                        time.sleep(sleep_t)
        except Exception as exc:  # noqa: BLE001
            log.warning("Viewer closed: %s", exc)
