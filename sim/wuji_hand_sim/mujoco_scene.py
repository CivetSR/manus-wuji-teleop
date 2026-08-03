"""MuJoCo scene using pinned wuji-description Hand2 Beta1."""

from __future__ import annotations

import logging
import math
import threading
import time
from pathlib import Path
from typing import Optional

import mujoco
import mujoco.viewer
import numpy as np

from joint_order import (
    hand2_protocol_joint_names,
    mujoco_actuator_joint_names,
    strict_joint_name_permutation,
)
from wuji_retargeting_adapter import official_hand2_model_paths

log = logging.getLogger("wuji_hand_sim.mujoco")

def mjcf_for_side(side: str) -> Path:
    return official_hand2_model_paths(side).mjcf


def _assert_hand2_model(model: mujoco.MjModel, mjcf: Path) -> None:
    if model.nu != 20:
        raise RuntimeError(f"Hand2 requires exactly 20 actuators, model has nu={model.nu}")
    if "/hand2/hand2_beta1/body/" not in str(mjcf.resolve()):
        raise RuntimeError(f"Not the pinned Hand2 Beta1 MJCF: {mjcf}")
    log.info("Loaded Wuji Hand 2: %s (nu=%d)", mjcf.name, model.nu)


class MujocoScene:
    """Single Hand2 model with name-verified protocol ordering."""

    NUM_ACTUATORS = 20

    def __init__(self, side: str, *, headless: bool = False) -> None:
        self.side = side.lower()
        self.headless = headless
        mjcf = mjcf_for_side(self.side)
        log.info("Loading Hand 2 MJCF: %s", mjcf)
        self.model = mujoco.MjModel.from_xml_path(str(mjcf))
        _assert_hand2_model(self.model, mjcf)
        self.data = mujoco.MjData(self.model)
        protocol_names = hand2_protocol_joint_names(self.side)
        actuator_names = mujoco_actuator_joint_names(self.model)
        self._protocol_to_actuator = strict_joint_name_permutation(
            protocol_names, actuator_names
        )
        self._protocol_qpos_addr: list[int] = []
        self._protocol_dof_addr: list[int] = []
        for name in protocol_names:
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise ValueError(f"Hand2 MJCF is missing protocol joint {name!r}")
            if self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError(f"Hand2 protocol joint {name!r} is not scalar hinge")
            self._protocol_qpos_addr.append(int(self.model.jnt_qposadr[joint_id]))
            self._protocol_dof_addr.append(int(self.model.jnt_dofadr[joint_id]))
        log.info(
            "Verified TCP->actuator permutation: %s",
            self._protocol_to_actuator.tolist(),
        )
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
        command = np.asarray(position, dtype=np.float64)
        if command.shape != (self.NUM_ACTUATORS,):
            raise ValueError(
                f"Hand2 command must have shape ({self.NUM_ACTUATORS},), got {command.shape}"
            )
        if not np.isfinite(command).all():
            raise ValueError("Hand2 command contains NaN or infinity")
        actuator_command = command[self._protocol_to_actuator]
        with self._lock:
            self.data.ctrl[:] = actuator_command

    def read_joint_state(self) -> tuple[list[float], list[float]]:
        with self._lock:
            pos = [float(self.data.qpos[address]) for address in self._protocol_qpos_addr]
            vel = [float(self.data.qvel[address]) for address in self._protocol_dof_addr]
        if not all(math.isfinite(value) for value in pos + vel):
            raise RuntimeError("MuJoCo produced a non-finite Hand2 joint state")
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
