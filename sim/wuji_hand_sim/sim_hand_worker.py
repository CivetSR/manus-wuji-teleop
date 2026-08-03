"""Simulated hand worker — same interface as bundle HandWorker, drives MuJoCo."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, List, Optional

from wuji_manus_bridge.protocol import FINGER_NAMES, NUM_JOINTS, normalize_positions
from wuji_manus_bridge.smoother import JointSmoother
from wuji_manus_bridge.tactile import EMPTY_FINGER

from .mujoco_scene import MujocoScene

log = logging.getLogger("wuji_hand_sim.hand")


class SimHandWorker:
    """Drop-in replacement for wuji_manus_bridge.hand_worker.HandWorker."""

    def __init__(
        self,
        side: str,
        scene: MujocoScene,
        *,
        serial_number: str = "SIM",
        control_hz: float = 100.0,
        cutoff_hz: float = 5.0,
        max_joint_speed_rad_s: float = 2.0,
        haptic_scale_n: float = 2.0,
        device_name: Optional[str] = None,
    ) -> None:
        self.side = side
        self.scene = scene
        self.serial_number = serial_number
        self.control_hz = control_hz
        self.cutoff_hz = cutoff_hz
        self.max_joint_speed_rad_s = max_joint_speed_rad_s
        self.haptic_scale_n = haptic_scale_n
        self.device_name = device_name or f"mujoco_sim_{side}"

        self._lock = threading.Lock()
        self._enabled = False
        self._positions = [0.0] * NUM_JOINTS
        self._velocities = [0.0] * NUM_JOINTS
        self._efforts = [0.0] * NUM_JOINTS
        self._fingers: List[Dict[str, float]] = [dict(EMPTY_FINGER) for _ in range(5)]
        self._cmd_seq = 0
        self._state_seq = 0
        self._tactile_seq = 0

        self._smoother = JointSmoother(
            num_joints=NUM_JOINTS,
            cutoff_hz=cutoff_hz,
            control_hz=control_hz,
            max_joint_speed_rad_s=max_joint_speed_rad_s,
        )

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    def info(self) -> Dict[str, Any]:
        return {
            "side": self.side,
            "serial_number": self.serial_number,
            "connected": self._connected,
            "enabled": self._enabled,
            "cutoff_hz": self.cutoff_hz,
            "control_hz": self.control_hz,
            "max_joint_speed_rad_s": self.max_joint_speed_rad_s,
            "realtime_controller": False,
            "simulation": True,
        }

    def connect(self) -> None:
        log.info("Sim hand %s ready (MuJoCo)", self.side)
        pos, vel = self.scene.read_joint_state()
        with self._lock:
            self._positions = pos
            self._velocities = vel
            self._smoother.reset(pos)
        self._connected = True

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._enabled = bool(enabled)
            if not enabled:
                self._smoother.set_target(list(self._positions))

    def set_joint_target(self, position: List[float], enable: bool = True) -> None:
        pos = normalize_positions(position)
        with self._lock:
            if enable:
                self._enabled = True
            self._smoother.set_target(pos)
            self._cmd_seq += 1

    def snapshot_joint_state(self) -> Dict[str, Any]:
        pos, vel = self.scene.read_joint_state()
        with self._lock:
            self._positions = pos
            self._velocities = vel
            self._state_seq += 1
            return {
                "side": self.side,
                "seq": self._state_seq,
                "t_ms": int(time.time() * 1000),
                "position": list(pos),
                "velocity": list(vel),
                "effort": list(self._efforts),
            }

    def snapshot_tactile(self) -> Dict[str, Any]:
        with self._lock:
            fingers = [dict(f) for f in self._fingers]
            powers = [float(f.get("haptic_01", 0.0)) for f in fingers]
            return {
                "side": self.side,
                "seq": self._tactile_seq,
                "t_ms": int(time.time() * 1000),
                "fingers": fingers,
                "haptic_powers": powers,
            }

    def start_loop(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._control_loop, name=f"sim-ctrl-{self.side}", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def disconnect(self) -> None:
        self.stop()
        self._connected = False

    def _control_loop(self) -> None:
        period = 1.0 / max(self.control_hz, 1.0)
        next_t = time.monotonic()
        while not self._stop.is_set():
            next_t += period
            try:
                self._tick()
            except Exception as exc:  # noqa: BLE001
                log.warning("%s control tick error: %s", self.side, exc)
            sleep_t = next_t - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.monotonic()

    def _tick(self) -> None:
        with self._lock:
            enabled = self._enabled
            cmd = self._smoother.step() if enabled else None
        if not enabled or cmd is None:
            return
        self.scene.set_ctrl(cmd)
