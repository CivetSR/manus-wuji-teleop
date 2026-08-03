"""Per-hand worker: connect Wuji Hand 2, smooth cmds, stream tactile."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Dict, List, Optional

import wuji_sdk
from wuji_sdk import JointCommand, LowPass

from .protocol import FINGER_NAMES, NUM_JOINTS, normalize_positions
from .smoother import JointSmoother
from .tactile import EMPTY_FINGER, decode_fingertip_frame

log = logging.getLogger("wuji_manus_bridge.hand")

FINGERTIP_METHODS = (
    "fingertip_thumb_data",
    "fingertip_index_data",
    "fingertip_middle_data",
    "fingertip_ring_data",
    "fingertip_pinky_data",
)


class HandWorker:
    def __init__(
        self,
        side: str,
        serial_number: str,
        control_hz: float = 100.0,
        cutoff_hz: float = 5.0,
        max_joint_speed_rad_s: float = 2.0,
        haptic_scale_n: float = 2.0,
        device_name: Optional[str] = None,
    ) -> None:
        self.side = side
        self.serial_number = serial_number
        self.control_hz = control_hz
        self.cutoff_hz = cutoff_hz
        self.max_joint_speed_rad_s = max_joint_speed_rad_s
        self.haptic_scale_n = haptic_scale_n
        self.device_name = device_name or f"manus_bridge_{side}"

        self._mgr = wuji_sdk.SdkManager.instance()
        self._hand: Optional[Any] = None
        self._publisher = None
        self._state_sub = None
        self._tactile_subs: List[Any] = []
        self._rt_cm = None  # realtime_controller context if available

        self._lock = threading.Lock()
        self._enabled = False
        self._positions = [0.0] * NUM_JOINTS
        self._velocities: List[float] = []
        self._efforts: List[float] = []
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
            "realtime_controller": self._rt_cm is not None,
        }

    def connect(self) -> None:
        log.info("Connecting %s sn=%s ...", self.side, self.serial_number)
        opts = wuji_sdk.ConnectOptions()
        opts.timeout_ms = 20000
        opts.retry_count = 5
        hand = self._mgr.connect(
            sn=self.serial_number, device_name=self.device_name, options=opts
        )
        if not hand.is_connected:
            raise RuntimeError(f"Failed to connect {self.serial_number}")

        hand.enable()
        self._hand = hand

        # Prefer official realtime_controller + LowPass when available (Hand v1).
        # Hand 2 currently lacks it; we fall back to host-side JointSmoother.
        if hasattr(hand, "realtime_controller"):
            try:
                self._rt_cm = hand.realtime_controller(LowPass(cutoff_hz=self.cutoff_hz))
                self._rt_cm.__enter__()
                log.info("%s: opened realtime_controller LowPass(%.1f Hz)", self.side, self.cutoff_hz)
            except Exception as exc:  # noqa: BLE001
                log.warning("%s: realtime_controller unavailable (%s); using host LPF", self.side, exc)
                self._rt_cm = None

        self._publisher = hand.joint_command().publish()

        def _state_cb(frame) -> None:
            pos = [0.0] * NUM_JOINTS
            vel = [0.0] * NUM_JOINTS
            eff = [0.0] * NUM_JOINTS
            if hasattr(frame, "joints"):
                for joint in frame.joints:
                    nid = int(joint.nid)
                    if 0 <= nid < NUM_JOINTS:
                        pos[nid] = float(joint.position)
                        vel[nid] = float(getattr(joint, "velocity", 0.0) or 0.0)
                        eff[nid] = float(getattr(joint, "effort", 0.0) or 0.0)
            elif hasattr(frame, "position"):
                for i, p in enumerate(list(frame.position)[:NUM_JOINTS]):
                    pos[i] = float(p)
            with self._lock:
                self._positions = pos
                self._velocities = vel
                self._efforts = eff
                self._state_seq += 1
                if not self._smoother.initialized:
                    self._smoother.reset(pos)

        self._state_sub = hand.joint_states().subscribe_with_callback(_state_cb)

        for idx, method_name in enumerate(FINGERTIP_METHODS):
            getter = getattr(hand, method_name, None)
            if getter is None:
                continue

            def _make_cb(finger_idx: int) -> Callable:
                def _cb(frame) -> None:
                    decoded = decode_fingertip_frame(
                        frame.data, haptic_scale_n=self.haptic_scale_n
                    )
                    with self._lock:
                        self._fingers[finger_idx] = decoded
                        self._tactile_seq += 1

                return _cb

            try:
                sub = getter().subscribe_with_callback(_make_cb(idx))
                self._tactile_subs.append(sub)
            except Exception as exc:  # noqa: BLE001
                log.warning("%s: tactile %s failed: %s", self.side, FINGER_NAMES[idx], exc)

        self._connected = True
        log.info("%s connected", self.side)

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._enabled = bool(enabled)
            if not enabled and self._connected:
                # Hold current pose (don't slam to zero)
                self._smoother.set_target(list(self._positions))

    def set_joint_target(self, position: List[float], enable: bool = True) -> None:
        pos = normalize_positions(position)
        with self._lock:
            if enable:
                self._enabled = True
            self._smoother.set_target(pos)
            self._cmd_seq += 1

    def snapshot_joint_state(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "side": self.side,
                "seq": self._state_seq,
                "t_ms": int(time.time() * 1000),
                "position": list(self._positions),
                "velocity": list(self._velocities),
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
            target=self._control_loop, name=f"wuji-ctrl-{self.side}", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def disconnect(self) -> None:
        self.stop()
        try:
            for sub in self._tactile_subs:
                try:
                    sub.close()
                except Exception:  # noqa: BLE001
                    pass
            self._tactile_subs.clear()
            if self._state_sub is not None:
                self._state_sub.close()
                self._state_sub = None
            if self._publisher is not None:
                self._publisher.close()
                self._publisher = None
            if self._rt_cm is not None:
                try:
                    self._rt_cm.__exit__(None, None, None)
                except Exception:  # noqa: BLE001
                    pass
                self._rt_cm = None
            if self._hand is not None:
                try:
                    self._hand.disable()
                except Exception:  # noqa: BLE001
                    pass
                self._mgr.disconnect(self.device_name)
                self._hand = None
        finally:
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
                # overrun: resync
                next_t = time.monotonic()

    def _tick(self) -> None:
        if self._publisher is None:
            return
        with self._lock:
            enabled = self._enabled
            cmd = self._smoother.step() if enabled else None
        if not enabled or cmd is None:
            return
        commands = [
            JointCommand(position=float(p), velocity=0.0, effort=0.0) for p in cmd
        ]
        self._publisher.send(commands)
