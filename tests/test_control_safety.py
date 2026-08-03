from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any

import pytest

from wuji_manus_bridge.control import ControlAuthority, ControlError, SafeClientSession
from wuji_manus_bridge.protocol import dumps, loads_line, normalize_positions
from wuji_manus_bridge.server import BridgeServer
from wuji_hand_sim.sim_server import SimBridgeServer


class FakeHand:
    def __init__(self, side: str = "left") -> None:
        self.side = side
        self.control_hz = 100.0
        self.cutoff_hz = 5.0
        self.max_joint_speed_rad_s = 2.0
        self.connected = True
        self.enabled = False
        self.enable_calls: list[bool] = []
        self.targets: list[list[float]] = []

    def info(self) -> dict[str, Any]:
        return {"side": self.side, "connected": True, "enabled": self.enabled}

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled
        self.enable_calls.append(enabled)

    def set_joint_target(self, position: list[float]) -> None:
        if not self.enabled:
            raise RuntimeError("not armed")
        self.targets.append(list(position))

    def snapshot_joint_state(self) -> dict[str, Any]:
        return {"side": self.side, "position": [0.0] * 20}

    def snapshot_tactile(self) -> dict[str, Any]:
        return {"side": self.side, "haptic_powers": [0.0] * 5}


@pytest.mark.parametrize("server_type", [BridgeServer, SimBridgeServer])
def test_server_deadman_watchdog_runs_without_a_client_session(server_type: type) -> None:
    expired = threading.Event()

    class Authority:
        def expire(self) -> None:
            expired.set()

    server = server_type.__new__(server_type)
    server._stop = threading.Event()
    server.authority = Authority()
    thread = threading.Thread(target=server._watchdog_loop)
    thread.start()
    try:
        assert expired.wait(timeout=0.2)
    finally:
        server._stop.set()
        thread.join(timeout=0.2)
    assert not thread.is_alive()


def valid_command(authority: ControlAuthority, client: object, *, seq: int = 1) -> None:
    authority.accept_command(
        client,
        "left",
        [0.1] * 20,
        seq=seq,
        t_ms=10_000 + seq,
        now_wall_ms=10_000 + seq,
    )


def test_safe_session_thread_processes_hello() -> None:
    hand = FakeHand()
    authority = ControlAuthority({"left": hand})
    server_socket, client_socket = socket.socketpair()
    session = SafeClientSession(
        server_socket,
        ("local", 0),
        {"left": hand},
        authority,
        tactile_hz=1.0,
        state_hz=1.0,
        server_name="test",
    )
    client_socket.settimeout(1.0)
    session.start()
    try:
        client_socket.sendall(
            b'{"type":"hello","protocol_version":1,"client":"test"}\n'
        )
        payload = b""
        while b"\n" not in payload:
            payload += client_socket.recv(65536)
        reply = json.loads(payload.split(b"\n", 1)[0])
        assert reply["type"] == "hello_ack"
        assert reply["hands"]["left"]["connected"] is True
    finally:
        client_socket.close()
        session.join(timeout=1.0)
        if session.is_alive():
            session.stop()
            session.join(timeout=1.0)
    assert not session.is_alive()


def test_no_command_or_enable_before_explicit_arm() -> None:
    hand = FakeHand()
    authority = ControlAuthority({"left": hand})
    client = object()

    assert hand.enable_calls == []
    with pytest.raises(ControlError, match="control lease"):
        valid_command(authority, client)
    assert hand.enable_calls == []
    assert hand.targets == []

    authority.arm(client, ["left"])
    assert hand.enable_calls == [True]
    valid_command(authority, client)
    assert len(hand.targets) == 1


def test_partial_arm_failure_always_attempts_disable() -> None:
    class PartiallyFailingHand(FakeHand):
        def set_enabled(self, enabled: bool) -> None:
            super().set_enabled(enabled)
            if enabled:
                raise RuntimeError("enable acknowledgement failed")

    hand = PartiallyFailingHand()
    authority = ControlAuthority({"left": hand})
    client = object()
    with pytest.raises(RuntimeError, match="acknowledgement"):
        authority.arm(client, ["left"])
    assert hand.enable_calls == [True, False]
    assert hand.enabled is False
    assert authority.status_for(client)["control_lease_held"] is False


def test_disconnect_disables_and_releases_control_lease() -> None:
    hand = FakeHand()
    authority = ControlAuthority({"left": hand})
    first = object()
    second = object()
    authority.arm(first, ["left"])
    valid_command(authority, first)

    authority.disconnect(first)
    assert hand.enable_calls[-1] is False
    assert not hand.enabled

    authority.arm(second, ["left"])
    assert hand.enable_calls[-1] is True


def test_disconnect_attempts_every_side_even_if_one_disable_reports_failure() -> None:
    class DisableReportingFailureHand(FakeHand):
        def set_enabled(self, enabled: bool) -> None:
            super().set_enabled(enabled)
            if not enabled:
                raise RuntimeError("disable acknowledgement failed")

    left = DisableReportingFailureHand("left")
    right = FakeHand("right")
    authority = ControlAuthority({"left": left, "right": right})
    client = object()
    authority.arm(client, ["left", "right"])

    authority.disconnect(client)

    assert left.enable_calls == [True, False]
    assert right.enable_calls == [True, False]
    assert authority.status_for(client)["control_lease_held"] is False


def test_deadman_disables_after_200_ms_and_stops_accepting_commands() -> None:
    hand = FakeHand()
    authority = ControlAuthority({"left": hand}, command_timeout_s=0.2)
    client = object()
    authority.arm(client, ["left"])
    accepted_at = time.monotonic()
    authority.accept_command(
        client,
        "left",
        [0.2] * 20,
        seq=1,
        t_ms=1000,
        now_wall_ms=1000,
        now_monotonic=accepted_at,
    )

    assert authority.expire(now_monotonic=accepted_at + 0.199) == []
    assert authority.expire(now_monotonic=accepted_at + 0.201) == ["left"]
    assert hand.enable_calls[-1] is False
    with pytest.raises(ControlError, match="control lease"):
        authority.accept_command(
            client,
            "left",
            [0.3] * 20,
            seq=2,
            t_ms=1001,
            now_wall_ms=1001,
        )
    assert len(hand.targets) == 1


def test_second_control_client_is_rejected_until_release() -> None:
    hand = FakeHand()
    authority = ControlAuthority({"left": hand})
    first = object()
    second = object()
    authority.arm(first, ["left"])

    with pytest.raises(ControlError) as error:
        authority.arm(second, ["left"])
    assert error.value.code == "control_busy"
    with pytest.raises(ControlError) as error:
        valid_command(authority, second)
    assert error.value.code == "not_controller"
    assert hand.enable_calls == [True]


def test_replayed_or_stale_sequence_and_timestamp_are_rejected() -> None:
    hand = FakeHand()
    authority = ControlAuthority({"left": hand})
    client = object()
    authority.arm(client, ["left"])
    authority.accept_command(
        client,
        "left",
        [0.0] * 20,
        seq=10,
        t_ms=20_000,
        now_wall_ms=20_000,
    )

    with pytest.raises(ControlError) as error:
        authority.accept_command(
            client,
            "left",
            [0.0] * 20,
            seq=10,
            t_ms=20_001,
            now_wall_ms=20_001,
        )
    assert error.value.code == "stale_seq"

    with pytest.raises(ControlError) as error:
        authority.accept_command(
            client,
            "left",
            [0.0] * 20,
            seq=11,
            t_ms=19_999,
            now_wall_ms=20_001,
        )
    assert error.value.code == "stale_timestamp"

    with pytest.raises(ControlError) as error:
        authority.accept_command(
            client,
            "left",
            [0.0] * 20,
            seq=11,
            t_ms=20_001,
            now_wall_ms=20_500,
        )
    assert error.value.code == "stale_timestamp"
    assert len(hand.targets) == 1


@pytest.mark.parametrize(
    "bad",
    [
        [0.0] * 19,
        [0.0] * 21,
        [0.0] * 19 + [float("nan")],
        [0.0] * 19 + [float("inf")],
        [0.0] * 19 + [True],
        [0.0] * 19 + ["0.0"],
        tuple([0.0] * 20),
    ],
)
def test_bad_joint_vectors_are_never_padded_truncated_or_coerced(bad: Any) -> None:
    with pytest.raises(ValueError):
        normalize_positions(bad)


@pytest.mark.parametrize("payload", ["NaN", "Infinity", "-Infinity"])
def test_nonstandard_json_nonfinite_tokens_are_rejected(payload: str) -> None:
    message = (
        '{"type":"joint_cmd","position":['
        + ",".join(["0"] * 19 + [payload])
        + "]}"
    )
    with pytest.raises(ValueError):
        loads_line(message)


def test_server_never_serializes_nonstandard_nonfinite_json() -> None:
    with pytest.raises(ValueError):
        dumps({"type": "tactile", "haptic_powers": [float("nan")]})
