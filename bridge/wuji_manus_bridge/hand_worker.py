"""Per-hand worker: connect Wuji Hand 2, smooth cmds, stream tactile."""

from __future__ import annotations

import importlib.metadata
import logging
import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

import wuji_sdk
from wuji_sdk import JointCommand, LowPass

from .protocol import FINGER_NAMES, NUM_JOINTS, normalize_positions
from .smoother import JointSmoother
from .tactile import (
    EMPTY_FINGER,
    FingertipFormat,
    decode_fingertip_frame,
    format_for_finger,
)

log = logging.getLogger("wuji_manus_bridge.hand")

WUJI_SDK_VERSION = importlib.metadata.version("wuji-sdk")
if WUJI_SDK_VERSION != "2026.8.3":
    raise RuntimeError(
        f"wuji-sdk {WUJI_SDK_VERSION} is unsupported; exactly 2026.8.3 is required"
    )

FINGERTIP_METHODS = (
    "fingertip_thumb_data",
    "fingertip_index_data",
    "fingertip_middle_data",
    "fingertip_ring_data",
    "fingertip_pinky_data",
)


@dataclass(frozen=True)
class DiscoveredHand2:
    side: str
    address: str
    serial_number: str


def discover_hand2_devices(sides: List[str]) -> Dict[str, DiscoveredHand2]:
    """Discover network Hand2 devices and identify them by reported handedness."""

    manager = wuji_sdk.SdkManager.instance()
    devices = [
        device
        for device in manager.scan()
        if str(device.sn).upper().startswith("WH")
    ]
    if not devices:
        raise RuntimeError("no Wuji Hand 2 found on the network")

    wanted = set(sides)
    discovered: Dict[str, DiscoveredHand2] = {}
    for index, device in enumerate(devices):
        hand = manager.connect(
            address=str(device.address),
            device_name=f"wuji_hand_2_probe_{index}",
        )
        try:
            side = str(hand.handedness().get()).lower()
            if side not in ("left", "right"):
                raise RuntimeError(
                    f"Hand2 {device.sn}@{device.address} reported invalid handedness {side!r}"
                )
            if side in discovered:
                raise RuntimeError(
                    f"multiple network Hand2 devices reported handedness={side}"
                )
            discovered[side] = DiscoveredHand2(
                side=side,
                address=str(device.address),
                serial_number=str(device.sn),
            )
            log.info("Discovered %s Hand2 %s at %s", side, device.sn, device.address)
        finally:
            hand.disconnect()
            time.sleep(0.2)

    missing = sorted(wanted - set(discovered))
    if missing:
        listing = ", ".join(
            f"{hand.side}:{hand.serial_number}@{hand.address}"
            for hand in discovered.values()
        )
        raise RuntimeError(f"missing network Hand2 sides {missing}; discovered [{listing}]")
    return discovered


class HandWorker:
    def __init__(
        self,
        side: str,
        address: str,
        control_hz: float = 100.0,
        cutoff_hz: float = 5.0,
        max_joint_speed_rad_s: float = 2.0,
        haptic_scale_n: float = 2.0,
        device_name: Optional[str] = None,
    ) -> None:
        self.side = side
        self.address = address
        self.serial_number = ""
        self.control_hz = control_hz
        self.cutoff_hz = cutoff_hz
        self.max_joint_speed_rad_s = max_joint_speed_rad_s
        self.haptic_scale_n = haptic_scale_n
        self.device_name = device_name or f"manus_bridge_{side}"

        self._mgr = wuji_sdk.SdkManager.instance()
        self._hand: Optional[Any] = None
        self._publisher = None
        self._publisher_error: Optional[BaseException] = None
        self._publisher_ready = threading.Event()
        self._publisher_thread_id: Optional[int] = None
        self._state_sub = None
        self._tactile_subs: List[Any] = []
        self._tactile_formats: List[FingertipFormat] = [
            format_for_finger(index) for index in range(5)
        ]
        self._rt_cm = None  # realtime_controller context if available

        self._lock = threading.Lock()
        self._state_changed = threading.Condition(self._lock)
        self._enabled = False
        self._hardware_enabled = False
        self._hardware_enable_attempted = False
        self._state_ready = False
        self._has_command = False
        self._pending_target: Optional[List[float]] = None
        self._positions = [0.0] * NUM_JOINTS
        self._velocities: List[float] = []
        self._efforts: List[float] = []
        self._fingers: List[Dict[str, Any]] = [dict(EMPTY_FINGER) for _ in range(5)]
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
        self._control_wake = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    def info(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "side": self.side,
                "address": self.address,
                "serial_number": self.serial_number,
                "connected": self._connected,
                "enabled": self._enabled,
                "hardware_enabled": self._hardware_enabled,
                "state_ready": self._state_ready,
                "cutoff_hz": self.cutoff_hz,
                "control_hz": self.control_hz,
                "max_joint_speed_rad_s": self.max_joint_speed_rad_s,
                "realtime_controller": self._rt_cm is not None,
                "publisher_single_threaded": True,
            }

    def connect(self) -> None:
        log.info("Connecting %s Hand2 at %s ...", self.side, self.address)
        opts = wuji_sdk.ConnectOptions()
        opts.timeout_ms = 20000
        opts.retry_count = 5
        hand = self._mgr.connect(
            address=self.address, device_name=self.device_name, options=opts
        )
        if not hand.is_connected:
            raise RuntimeError(f"failed to connect Hand2 at {self.address}")

        self._hand = hand
        try:
            reported_side = str(hand.handedness().get()).lower()
            if reported_side != self.side:
                raise RuntimeError(
                    f"Hand2 at {self.address} reports {reported_side!r}, "
                    f"expected {self.side!r}"
                )
            self.serial_number = str(hand.serial_number)
            online = int(hand.online_joints_count().get())
            if online != NUM_JOINTS:
                raise RuntimeError(
                    f"{self.side} Hand2 must report all {NUM_JOINTS} online joints, got {online}"
                )
        except BaseException:
            hand.disconnect()
            self._hand = None
            raise

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

        def _state_cb(frame) -> None:
            try:
                pos = [0.0] * NUM_JOINTS
                vel = [0.0] * NUM_JOINTS
                eff = [0.0] * NUM_JOINTS
                if hasattr(frame, "joints"):
                    joints = list(frame.joints)
                    seen: set[int] = set()
                    for joint in joints:
                        nid = int(joint.nid)
                        if not 0 <= nid < NUM_JOINTS or nid in seen:
                            raise ValueError(f"invalid or duplicate joint nid={nid}")
                        seen.add(nid)
                        pos[nid] = float(joint.position)
                        vel[nid] = float(getattr(joint, "velocity", 0.0) or 0.0)
                        eff[nid] = float(getattr(joint, "effort", 0.0) or 0.0)
                    if seen != set(range(NUM_JOINTS)):
                        raise ValueError(
                            f"joint state must contain nids 0..19, got {sorted(seen)}"
                        )
                elif hasattr(frame, "position"):
                    raw_position = list(frame.position)
                    if len(raw_position) != NUM_JOINTS:
                        raise ValueError(
                            f"joint state position must contain {NUM_JOINTS} values"
                        )
                    pos = [float(value) for value in raw_position]
                else:
                    raise ValueError("joint state has neither joints nor position")
                if not all(math.isfinite(value) for value in pos + vel + eff):
                    raise ValueError("joint state contains NaN or infinity")
            except (TypeError, ValueError) as exc:
                log.warning("%s: dropping invalid joint state: %s", self.side, exc)
                return

            with self._state_changed:
                self._positions = pos
                self._velocities = vel
                self._efforts = eff
                self._state_seq += 1
                if not self._state_ready:
                    self._smoother.reset(pos)
                    self._state_ready = True
                    if self._pending_target is not None:
                        self._smoother.set_target(self._pending_target)
                self._state_changed.notify_all()

        self._state_sub = hand.joint_states().subscribe_with_callback(_state_cb)

        for idx, method_name in enumerate(FINGERTIP_METHODS):
            getter = getattr(hand, method_name, None)
            if getter is None:
                continue
            try:
                info = hand.get_fingertip_info(idx)
            except Exception as exc:  # noqa: BLE001 - safe official fallback
                self._tactile_formats[idx] = format_for_finger(idx)
                log.warning(
                    "%s: %s tactile metadata unavailable (%s); using official %d-point layout",
                    self.side,
                    FINGER_NAMES[idx],
                    exc,
                    self._tactile_formats[idx].point_count,
                )
            else:
                try:
                    self._tactile_formats[idx] = format_for_finger(idx, info)
                except (TypeError, ValueError) as exc:
                    raise RuntimeError(
                        f"{self.side} {FINGER_NAMES[idx]} returned unsupported "
                        f"fingertip metadata; refusing to guess its payload layout: {exc}"
                    ) from exc
                log.info(
                    "%s %s tactile: model=%s points=%d rate_hz=%s digest=%s",
                    self.side,
                    FINGER_NAMES[idx],
                    getattr(info, "model", "unknown"),
                    self._tactile_formats[idx].point_count,
                    getattr(info, "rate_hz", "unknown"),
                    self._tactile_formats[idx].digest,
                )

            def _make_cb(finger_idx: int) -> Callable:
                def _cb(frame) -> None:
                    sensor_format = self._tactile_formats[finger_idx]
                    frame_digest = getattr(frame, "info_digest", None)
                    if (
                        sensor_format.digest is not None
                        and frame_digest is not None
                        and int(frame_digest) != sensor_format.digest
                    ):
                        log.warning(
                            "%s: dropping %s tactile frame with stale info digest %s != %s",
                            self.side,
                            FINGER_NAMES[finger_idx],
                            frame_digest,
                            sensor_format.digest,
                        )
                        return
                    try:
                        decoded = decode_fingertip_frame(
                            frame.data,
                            sensor_format=sensor_format,
                            haptic_scale_n=self.haptic_scale_n,
                        )
                    except (TypeError, ValueError, struct.error) as exc:
                        log.warning(
                            "%s: dropping invalid %s tactile frame: %s",
                            self.side,
                            FINGER_NAMES[finger_idx],
                            exc,
                        )
                        return
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
        log.info(
            "%s Hand2 connected: sn=%s address=%s",
            self.side,
            self.serial_number,
            self.address,
        )

    def set_enabled(self, enabled: bool) -> None:
        requested = bool(enabled)
        with self._state_changed:
            if requested and not self._connected:
                raise RuntimeError(f"{self.side} Hand2 is not connected")
            if requested and (self._thread is None or not self._thread.is_alive()):
                raise RuntimeError(f"{self.side} Hand2 control loop is not running")
            self._enabled = requested
            if not requested:
                self._has_command = False
                self._pending_target = None
                if self._state_ready:
                    self._smoother.set_target(list(self._positions))
            self._control_wake.set()

            deadline = time.monotonic() + 1.0
            while self._hardware_enabled != requested:
                if self._publisher_error is not None:
                    raise RuntimeError(
                        f"{self.side} Hand2 publisher failed: {self._publisher_error}"
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    if requested:
                        self._enabled = False
                        self._has_command = False
                        self._pending_target = None
                        self._control_wake.set()
                    raise TimeoutError(
                        f"{self.side} Hand2 did not reach hardware_enabled={requested}"
                    )
                self._state_changed.wait(remaining)

    def set_joint_target(self, position: List[float]) -> None:
        pos = normalize_positions(position)
        with self._lock:
            if not self._enabled or not self._hardware_enabled:
                raise RuntimeError(f"{self.side} Hand2 is not armed")
            self._pending_target = list(pos)
            self._has_command = True
            if self._state_ready:
                self._smoother.set_target(pos)
            self._cmd_seq += 1
        self._control_wake.set()

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
        self._control_wake.clear()
        self._publisher_ready.clear()
        self._publisher_error = None
        self._thread = threading.Thread(
            target=self._control_loop, name=f"wuji-ctrl-{self.side}", daemon=True
        )
        self._thread.start()
        if not self._publisher_ready.wait(timeout=2.0):
            raise TimeoutError(f"{self.side} Hand2 publisher did not initialize")
        if self._publisher_error is not None:
            error = self._publisher_error
            self.stop()
            raise RuntimeError(f"{self.side} Hand2 publisher initialization failed: {error}")

    def stop(self) -> None:
        self._stop.set()
        self._control_wake.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                raise TimeoutError(f"{self.side} Hand2 control loop did not stop")
            self._thread = None

    def disconnect(self) -> None:
        if self._thread and self._thread.is_alive():
            try:
                self.set_enabled(False)
            except Exception as exc:  # noqa: BLE001 - still stop and disconnect
                log.error("%s: disable before disconnect failed: %s", self.side, exc)
        try:
            self.stop()
        except Exception as exc:  # noqa: BLE001 - still close subscriptions
            log.error("%s: control loop shutdown failed: %s", self.side, exc)
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
            if self._rt_cm is not None:
                try:
                    self._rt_cm.__exit__(None, None, None)
                except Exception:  # noqa: BLE001
                    pass
                self._rt_cm = None
            if self._hand is not None:
                if self._hardware_enabled or self._hardware_enable_attempted:
                    # The control thread normally performs this transition. This
                    # path is only a last-resort attempt after a failed join.
                    try:
                        self._hand.disable()
                    except Exception as exc:  # noqa: BLE001
                        log.error("%s: final hardware disable failed: %s", self.side, exc)
                    else:
                        self._hardware_enabled = False
                        self._hardware_enable_attempted = False
                self._hand.disconnect()
                self._hand = None
        finally:
            with self._state_changed:
                self._enabled = False
                self._hardware_enabled = False
                self._hardware_enable_attempted = False
                self._connected = False
                self._state_changed.notify_all()

    def _control_loop(self) -> None:
        publisher = None
        self._publisher_thread_id = threading.get_ident()
        period = 1.0 / max(self.control_hz, 1.0)
        next_t = time.monotonic()
        try:
            if self._hand is None:
                raise RuntimeError("Hand2 connection disappeared before publisher creation")
            # wuji-sdk publisher creation, every send, and close are intentionally
            # owned by this one thread.
            publisher = self._hand.joint_command().publish()
            with self._state_changed:
                self._publisher = publisher
                self._publisher_ready.set()
                self._state_changed.notify_all()

            while not self._stop.is_set():
                next_t += period
                try:
                    self._tick(publisher)
                except Exception as exc:  # noqa: BLE001
                    log.error("%s control tick failed: %s", self.side, exc)
                    with self._state_changed:
                        self._publisher_error = exc
                        self._enabled = False
                        self._state_changed.notify_all()
                    break
                wait_s = next_t - time.monotonic()
                if wait_s > 0.0:
                    self._control_wake.wait(wait_s)
                    self._control_wake.clear()
                else:
                    next_t = time.monotonic()
        except BaseException as exc:
            with self._state_changed:
                self._publisher_error = exc
                self._publisher_ready.set()
                self._state_changed.notify_all()
        finally:
            with self._state_changed:
                must_disable = (
                    self._hardware_enabled or self._hardware_enable_attempted
                ) and self._hand is not None
                hand = self._hand
            disable_succeeded = False
            if must_disable and hand is not None:
                try:
                    hand.disable()
                except Exception as exc:  # noqa: BLE001
                    log.error("%s: hardware disable on publisher exit failed: %s", self.side, exc)
                else:
                    disable_succeeded = True
            with self._state_changed:
                if disable_succeeded:
                    self._hardware_enabled = False
                    self._hardware_enable_attempted = False
                self._enabled = False
                self._has_command = False
                self._state_changed.notify_all()
            if publisher is not None:
                try:
                    publisher.close()
                except Exception as exc:  # noqa: BLE001
                    log.error("%s: publisher close failed: %s", self.side, exc)
            with self._state_changed:
                self._publisher = None
                self._publisher_thread_id = None
                self._publisher_ready.set()
                self._state_changed.notify_all()

    def _tick(self, publisher: Any) -> None:
        with self._state_changed:
            hand = self._hand
            if hand is None:
                return
            requested_enabled = self._enabled
            hardware_enabled = self._hardware_enabled
            enable_attempted = self._hardware_enable_attempted

        if requested_enabled and not hardware_enabled:
            with self._state_changed:
                self._hardware_enable_attempted = True
            try:
                hand.enable()
            except BaseException:
                try:
                    hand.disable()
                except Exception as disable_exc:  # noqa: BLE001
                    log.error(
                        "%s: rollback disable after enable failure failed: %s",
                        self.side,
                        disable_exc,
                    )
                else:
                    with self._state_changed:
                        self._hardware_enable_attempted = False
                with self._state_changed:
                    self._enabled = False
                    self._hardware_enabled = False
                    self._has_command = False
                    self._state_changed.notify_all()
                raise
            with self._state_changed:
                self._hardware_enabled = True
                requested_enabled = self._enabled
                self._state_changed.notify_all()
            if not requested_enabled:
                hand.disable()
                with self._state_changed:
                    self._hardware_enabled = False
                    self._hardware_enable_attempted = False
                    self._state_changed.notify_all()
                return
        elif not requested_enabled and (hardware_enabled or enable_attempted):
            hand.disable()
            with self._state_changed:
                self._hardware_enabled = False
                self._hardware_enable_attempted = False
                self._state_changed.notify_all()
            return

        with self._state_changed:
            if (
                not self._enabled
                or not self._hardware_enabled
                or not self._state_ready
                or not self._has_command
            ):
                return
            command = self._smoother.step()
            if len(command) != NUM_JOINTS or not all(
                math.isfinite(value) for value in command
            ):
                raise RuntimeError(
                    f"{self.side} smoother produced an invalid joint command"
                )
            commands = [
                JointCommand(position=float(value), velocity=0.0, effort=0.0)
                for value in command
            ]
        publisher.send(commands)
