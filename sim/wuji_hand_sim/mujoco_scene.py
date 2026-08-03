"""MuJoCo scene: load wuji-description Hand2 MJCF and run physics + viewer."""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Optional

import mujoco
import mujoco.viewer

log = logging.getLogger("wuji_hand_sim.mujoco")

# Default: wuji-description Hand2 Beta1 (https://github.com/wuji-technology/wuji-description)
REPO_ROOT = Path(__file__).resolve().parents[2]
WUJI_DESC = REPO_ROOT / "deps" / "wuji-description"
DEFAULT_MJCF = {
    "left": WUJI_DESC / "hand2" / "hand2_beta1" / "body" / "mjcf" / "left.xml",
    "right": WUJI_DESC / "hand2" / "hand2_beta1" / "body" / "mjcf" / "right.xml",
}


def mjcf_for_side(side: str) -> Path:
    path = DEFAULT_MJCF.get(side.lower())
    if path is None or not path.is_file():
        raise FileNotFoundError(
            f"MJCF for side={side!r} not found at {path}. "
            "Clone wuji-description: git clone https://github.com/wuji-technology/wuji-description.git"
        )
    return path


class MujocoScene:
    """Single-hand MuJoCo model. Protocol index i -> actuator i (20 position actuators)."""

    NUM_ACTUATORS = 20

    def __init__(self, side: str, *, headless: bool = False) -> None:
        self.side = side.lower()
        self.headless = headless
        mjcf = mjcf_for_side(self.side)
        log.info("Loading MJCF %s", mjcf)
        self.model = mujoco.MjModel.from_xml_path(str(mjcf))
        self.data = mujoco.MjData(self.model)
        if self.model.nu != self.NUM_ACTUATORS:
            log.warning(
                "Expected %d actuators, model has nu=%d — verify joint layout",
                self.NUM_ACTUATORS,
                self.model.nu,
            )
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sim_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._stop.clear()
        target = self._headless_loop if self.headless else self._viewer_loop
        self._sim_thread = threading.Thread(
            target=target, name="mujoco-sim", daemon=True
        )
        self._sim_thread.start()
        log.info("MuJoCo scene started (%s hand, headless=%s)", self.side, self.headless)

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
        """Physics step; caller must hold _lock. Never call mj_step from another thread while viewer.sync runs."""
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
        """MuJoCo requires mj_step and viewer.sync in the same thread."""
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
