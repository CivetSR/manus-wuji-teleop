"""Shared fail-safe TCP control semantics for real and simulated Hand 2 backends."""

from __future__ import annotations

import logging
import select
import socket
import threading
import time
from typing import Any, Dict, Iterable, Mapping, Protocol

from .protocol import (
    PROTOCOL_VERSION,
    dumps,
    loads_line,
    normalize_positions,
)

log = logging.getLogger("wuji_manus_bridge.control")

DEFAULT_COMMAND_TIMEOUT_S = 0.2
DEFAULT_MAX_COMMAND_AGE_MS = 250
DEFAULT_MAX_FUTURE_SKEW_MS = 1000
MAX_JSON_LINE_BYTES = 1 << 20


class ControllableHand(Protocol):
    """Interface implemented by the real and MuJoCo hand workers."""

    side: str
    control_hz: float
    cutoff_hz: float
    max_joint_speed_rad_s: float

    @property
    def connected(self) -> bool: ...

    def info(self) -> Dict[str, Any]: ...

    def set_enabled(self, enabled: bool) -> None: ...

    def set_joint_target(self, position: list[float]) -> None: ...

    def snapshot_joint_state(self) -> Dict[str, Any]: ...

    def snapshot_tactile(self) -> Dict[str, Any]: ...


class ControlError(RuntimeError):
    """Protocol-level safety rejection with a stable error code."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def _strict_nonnegative_int(value: Any, field: str) -> int:
    if type(value) is not int or value < 0:
        raise ControlError(f"bad_{field}", f"{field} must be a non-negative JSON integer")
    return value


class ControlAuthority:
    """Single-writer lease, arm state, replay checks, and command deadman."""

    def __init__(
        self,
        hands: Mapping[str, ControllableHand],
        *,
        command_timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S,
        max_command_age_ms: int = DEFAULT_MAX_COMMAND_AGE_MS,
        max_future_skew_ms: int = DEFAULT_MAX_FUTURE_SKEW_MS,
    ) -> None:
        if not 0.1 <= float(command_timeout_s) <= 0.25:
            raise ValueError("command_timeout_s must be in the safety range [0.1, 0.25]")
        if int(max_command_age_ms) <= 0:
            raise ValueError("max_command_age_ms must be positive")
        self.hands = dict(hands)
        self.command_timeout_s = float(command_timeout_s)
        self.max_command_age_ms = int(max_command_age_ms)
        self.max_future_skew_ms = int(max_future_skew_ms)

        self._lock = threading.RLock()
        self._owner: object | None = None
        self._armed: set[str] = set()
        self._last_command_monotonic: dict[str, float] = {}
        self._last_seq: dict[str, int] = {}
        self._last_t_ms: dict[str, int] = {}

    def resolve_sides(self, side: Any) -> list[str]:
        value = str(side).lower()
        if value == "both":
            return list(self.hands)
        if value in self.hands:
            return [value]
        raise ControlError("bad_side", f"unknown side {value!r}")

    def arm(self, client: object, sides: Iterable[str]) -> None:
        targets = tuple(sides)
        if not targets:
            raise ControlError("bad_side", "arm request contains no available side")
        now = time.monotonic()
        with self._lock:
            if self._owner is not None and self._owner is not client:
                raise ControlError(
                    "control_busy",
                    "another client owns the control lease",
                )
            if self._owner is None:
                self._last_seq.clear()
                self._last_t_ms.clear()
            self._owner = client
            newly_armed: list[str] = []
            try:
                for side in targets:
                    if side not in self._armed:
                        try:
                            self.hands[side].set_enabled(True)
                        except Exception:
                            # Enabling may have reached hardware before the SDK
                            # reported an error/timeout.  Always issue a
                            # conservative disable attempt for that side.
                            try:
                                self.hands[side].set_enabled(False)
                            except Exception as disable_exc:  # noqa: BLE001
                                log.error(
                                    "%s failed to disable after arm failure: %s",
                                    side,
                                    disable_exc,
                                )
                            raise
                        self._armed.add(side)
                        self._last_command_monotonic[side] = now
                        newly_armed.append(side)
            except Exception:
                for side in reversed(newly_armed):
                    try:
                        self._disable_side_locked(side)
                    except Exception as disable_exc:  # noqa: BLE001
                        log.error(
                            "%s failed to disable while rolling back arm: %s",
                            side,
                            disable_exc,
                        )
                if not self._armed:
                    self._owner = None
                raise

    def disarm(self, client: object, sides: Iterable[str]) -> None:
        targets = tuple(sides)
        with self._lock:
            if self._owner is not client:
                raise ControlError("not_controller", "client does not own the control lease")
            first_error: Exception | None = None
            for side in targets:
                try:
                    self._disable_side_locked(side)
                except Exception as exc:  # noqa: BLE001
                    log.error("%s failed explicit disarm: %s", side, exc)
                    if first_error is None:
                        first_error = exc
            if not self._armed:
                self._owner = None
            if first_error is not None:
                raise first_error

    def accept_command(
        self,
        client: object,
        side: str,
        position: list[float],
        *,
        seq: Any,
        t_ms: Any,
        now_wall_ms: int | None = None,
        now_monotonic: float | None = None,
    ) -> None:
        sequence = _strict_nonnegative_int(seq, "seq")
        timestamp_ms = _strict_nonnegative_int(t_ms, "t_ms")
        wall_ms = int(time.time() * 1000) if now_wall_ms is None else int(now_wall_ms)
        monotonic_now = time.monotonic() if now_monotonic is None else float(now_monotonic)

        with self._lock:
            if self._owner is not client:
                raise ControlError("not_controller", "client does not own the control lease")
            if side not in self._armed:
                raise ControlError("not_armed", f"{side} is not armed")

            previous_seq = self._last_seq.get(side)
            if previous_seq is not None and sequence <= previous_seq:
                raise ControlError(
                    "stale_seq",
                    f"seq must increase strictly; last={previous_seq}, got={sequence}",
                )
            previous_t_ms = self._last_t_ms.get(side)
            if previous_t_ms is not None and timestamp_ms <= previous_t_ms:
                raise ControlError(
                    "stale_timestamp",
                    f"t_ms must increase strictly; last={previous_t_ms}, got={timestamp_ms}",
                )

            age_ms = wall_ms - timestamp_ms
            if age_ms > self.max_command_age_ms:
                raise ControlError(
                    "stale_timestamp",
                    f"command is {age_ms} ms old; maximum is {self.max_command_age_ms} ms",
                )
            if age_ms < -self.max_future_skew_ms:
                raise ControlError(
                    "future_timestamp",
                    f"command timestamp is {-age_ms} ms in the future",
                )

            # The worker does not auto-enable. Reaching this call therefore
            # cannot bypass the explicit arm lease above.
            self.hands[side].set_joint_target(position)
            self._last_seq[side] = sequence
            self._last_t_ms[side] = timestamp_ms
            self._last_command_monotonic[side] = monotonic_now

    def expire(self, *, now_monotonic: float | None = None) -> list[str]:
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        with self._lock:
            expired = [
                side
                for side in self._armed
                if now - self._last_command_monotonic.get(side, now)
                >= self.command_timeout_s
            ]
            for side in expired:
                log.warning("%s command deadman expired; disabling", side)
                try:
                    self._disable_side_locked(side)
                except Exception as exc:  # noqa: BLE001
                    log.error("%s failed deadman disable: %s", side, exc)
            if not self._armed:
                self._owner = None
            return sorted(expired)

    def disconnect(self, client: object) -> None:
        with self._lock:
            if self._owner is not client:
                return
            for side in tuple(self._armed):
                try:
                    self._disable_side_locked(side)
                except Exception as exc:  # noqa: BLE001
                    log.error("%s failed disconnect disable: %s", side, exc)
            self._owner = None

    def status_for(self, client: object) -> Dict[str, Any]:
        with self._lock:
            return {
                "controller": self._owner is client,
                "control_lease_held": self._owner is not None,
                "armed_sides": sorted(self._armed),
                "command_timeout_ms": int(round(self.command_timeout_s * 1000.0)),
                "max_command_age_ms": self.max_command_age_ms,
            }

    def _disable_side_locked(self, side: str) -> None:
        if side not in self._armed:
            return
        try:
            self.hands[side].set_enabled(False)
        finally:
            self._armed.discard(side)
            self._last_command_monotonic.pop(side, None)


class SafeClientSession(threading.Thread):
    """Shared JSONL session used verbatim by real and MuJoCo servers."""

    def __init__(
        self,
        conn: socket.socket,
        addr: Any,
        hands: Mapping[str, ControllableHand],
        authority: ControlAuthority,
        tactile_hz: float,
        state_hz: float,
        *,
        server_name: str,
    ) -> None:
        super().__init__(daemon=True, name=f"client-{addr[0]}:{addr[1]}")
        self.conn = conn
        self.addr = addr
        self.hands = dict(hands)
        self.authority = authority
        self.tactile_hz = tactile_hz
        self.state_hz = state_hz
        self.server_name = server_name
        self._buf = b""
        self._stop_event = threading.Event()
        self._hello_done = False

    def stop(self) -> None:
        self._stop_event.set()
        try:
            self.conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.conn.close()
        except OSError:
            pass

    def send(self, msg: dict[str, Any]) -> None:
        try:
            self.conn.sendall(dumps(msg))
        except (OSError, TypeError, ValueError) as exc:
            log.warning("send failed to %s: %s", self.addr, exc)
            self._stop_event.set()

    def run(self) -> None:
        log.info("Client connected %s", self.addr)
        self.conn.setblocking(False)
        last_tactile = 0.0
        last_state = 0.0
        tactile_period = 1.0 / max(self.tactile_hz, 1.0)
        state_period = 1.0 / max(self.state_hz, 1.0)

        try:
            while not self._stop_event.is_set():
                now = time.monotonic()
                self.authority.expire(now_monotonic=now)
                try:
                    readable, _, _ = select.select([self.conn], [], [], 0.005)
                except (OSError, ValueError):
                    break
                if readable:
                    try:
                        chunk = self.conn.recv(65536)
                    except (BlockingIOError, ConnectionResetError, OSError):
                        chunk = b""
                    if not chunk:
                        break
                    self._buf += chunk
                    if len(self._buf) > MAX_JSON_LINE_BYTES and b"\n" not in self._buf:
                        self._error("line_too_long", "JSON line exceeds 1 MiB")
                        break
                    while b"\n" in self._buf:
                        line, self._buf = self._buf.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        if len(line) > MAX_JSON_LINE_BYTES:
                            self._error("line_too_long", "JSON line exceeds 1 MiB")
                            continue
                        try:
                            msg = loads_line(line)
                        except (TypeError, ValueError, UnicodeError) as exc:
                            self._error("bad_json", str(exc))
                            continue
                        self._handle_message(msg)

                if self._hello_done:
                    if now - last_tactile >= tactile_period:
                        last_tactile = now
                        for hand in self.hands.values():
                            if hand.connected:
                                snapshot = hand.snapshot_tactile()
                                snapshot["type"] = "tactile"
                                self.send(snapshot)
                    if now - last_state >= state_period:
                        last_state = now
                        for hand in self.hands.values():
                            if hand.connected:
                                snapshot = hand.snapshot_joint_state()
                                snapshot["type"] = "joint_state"
                                self.send(snapshot)
        finally:
            self.authority.disconnect(self)
            log.info("Client disconnected %s", self.addr)
            self.stop()

    def _handle_message(self, msg: dict[str, Any]) -> None:
        message_type = msg.get("type")
        if message_type == "hello":
            version = msg.get("protocol_version")
            if type(version) is not int or version != PROTOCOL_VERSION:
                self._error(
                    "protocol_mismatch",
                    f"protocol_version must equal {PROTOCOL_VERSION}",
                )
                return
            self._hello_done = True
            self.send(self._hello_ack())
            return

        if not self._hello_done:
            self._error("no_hello", "send hello first")
            return

        if message_type == "ping":
            self.send({"type": "pong", "t_ms": int(time.time() * 1000)})
            return

        if message_type == "enable":
            enabled = msg.get("enabled")
            if type(enabled) is not bool:
                self._error("bad_enable", "enabled must be a JSON boolean")
                return
            try:
                sides = self.authority.resolve_sides(msg.get("side", "both"))
                if enabled:
                    self.authority.arm(self, sides)
                else:
                    self.authority.disarm(self, sides)
            except ControlError as exc:
                self._error(exc.code, str(exc))
                return
            except Exception as exc:  # noqa: BLE001 - SDK arm/disable failure
                self._error("enable_failed", str(exc))
                return
            self.send(
                {
                    "type": "status",
                    "ok": True,
                    "message": f"enable {msg.get('side', 'both')}={enabled}",
                    **self.authority.status_for(self),
                }
            )
            return

        if message_type == "joint_cmd":
            side = str(msg.get("side", "")).lower()
            if side not in self.hands:
                self._error("bad_side", f"unknown side {side!r}")
                return
            command_enable = msg.get("enable", True)
            if type(command_enable) is not bool:
                self._error("bad_enable", "joint_cmd.enable must be a JSON boolean")
                return
            if not command_enable:
                try:
                    self.authority.disarm(self, [side])
                except ControlError as exc:
                    self._error(exc.code, str(exc))
                    return
                self.send(
                    {
                        "type": "status",
                        "ok": True,
                        "message": f"enable {side}=False",
                        **self.authority.status_for(self),
                    }
                )
                return
            try:
                position = normalize_positions(msg.get("position"))
                self.authority.accept_command(
                    self,
                    side,
                    position,
                    seq=msg.get("seq"),
                    t_ms=msg.get("t_ms"),
                )
            except ControlError as exc:
                self._error(exc.code, str(exc))
            except (TypeError, ValueError) as exc:
                self._error("bad_position", str(exc))
            except Exception as exc:  # noqa: BLE001 - worker-local failure
                self._error("command_failed", str(exc))
            return

        if message_type == "get_status":
            self.send(
                {
                    "type": "status",
                    "ok": True,
                    "message": "ok",
                    "hands": {side: hand.info() for side, hand in self.hands.items()},
                    **self.authority.status_for(self),
                }
            )
            return

        self._error("unknown_type", str(message_type))

    def _hello_ack(self) -> dict[str, Any]:
        first_hand = next(iter(self.hands.values()), None)
        return {
            "type": "hello_ack",
            "protocol_version": PROTOCOL_VERSION,
            "server": self.server_name,
            "hands": {side: hand.info() for side, hand in self.hands.items()},
            "control_hz": first_hand.control_hz if first_hand else 100.0,
            "cutoff_hz": first_hand.cutoff_hz if first_hand else 5.0,
            "max_joint_speed_rad_s": (
                first_hand.max_joint_speed_rad_s if first_hand else 2.0
            ),
            **self.authority.status_for(self),
            "joint_layout": {
                "order": "finger_major",
                "fingers": ["thumb", "index", "middle", "ring", "pinky"],
                "joints_per_finger": 4,
                "num_joints": 20,
                "index_formula": "finger*4 + joint (joint=0..3 = J1..J4)",
                "unit": "rad",
            },
            "haptic_hint": {
                "manus_api": (
                    "CoreSdk_VibrateFingersForGlove(gloveId, float powers[5])"
                ),
                "powers_order": ["thumb", "index", "middle", "ring", "pinky"],
                "powers_range": [0.0, 1.0],
                "source_field": "tactile.haptic_powers",
            },
        }

    def _error(self, code: str, message: str) -> None:
        self.send({"type": "error", "code": code, "message": message})
