from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any

import pytest

from wuji_client import WujiClient


def test_connect_waits_for_valid_hello_ack() -> None:
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    port = listener.getsockname()[1]
    errors: list[BaseException] = []

    def serve() -> None:
        try:
            conn, _addr = listener.accept()
            with conn:
                payload = b""
                while b"\n" not in payload:
                    payload += conn.recv(65536)
                assert json.loads(payload.split(b"\n", 1)[0])["type"] == "hello"
                time.sleep(0.35)
                conn.sendall(
                    b'{"type":"hello_ack","protocol_version":1,'
                    b'"hands":{"left":{"connected":true}}}\n'
                )
                conn.settimeout(1.0)
                try:
                    conn.recv(65536)
                except (OSError, socket.timeout):
                    pass
        except BaseException as exc:  # noqa: BLE001 - surfaced in the test thread
            errors.append(exc)
        finally:
            listener.close()

    thread = threading.Thread(target=serve)
    thread.start()
    client = WujiClient("127.0.0.1", port)
    try:
        client.connect()
        assert client.hello_ack["hands"]["left"]["connected"] is True
    finally:
        client.close()
        thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert errors == []


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
def test_client_never_coerces_or_resizes_bad_joint_vectors(bad: Any) -> None:
    client = WujiClient("127.0.0.1")
    client.send = lambda _message: None  # type: ignore[method-assign]
    with pytest.raises(ValueError):
        client.send_joint_cmd("left", bad)


def test_rx_callback_exception_does_not_kill_receive_thread() -> None:
    client_sock, server_sock = socket.socketpair()
    client = WujiClient("127.0.0.1")
    client.sock = client_sock
    client._alive = True
    received: list[int] = []
    second_received = threading.Event()

    def on_tactile(msg: dict[str, Any]) -> None:
        received.append(int(msg["seq"]))
        if msg["seq"] == 1:
            raise RuntimeError("intentional callback failure")
        second_received.set()

    client._on_tactile = on_tactile
    thread = threading.Thread(target=client._rx_loop)
    thread.start()
    try:
        server_sock.sendall(
            b'{"type":"tactile","seq":1}\n'
            b'{"type":"tactile","seq":2}\n'
        )
        assert second_received.wait(timeout=1.0)
        assert received == [1, 2]
    finally:
        client._close_socket()
        server_sock.close()
        thread.join(timeout=1.0)
    assert not thread.is_alive()


def test_rx_eof_marks_socket_disconnected() -> None:
    client_sock, server_sock = socket.socketpair()
    client = WujiClient("127.0.0.1")
    client.sock = client_sock
    client._alive = True
    thread = threading.Thread(target=client._rx_loop)
    thread.start()

    server_sock.close()
    thread.join(timeout=1.0)

    assert not thread.is_alive()
    assert client._alive is False
    assert client.sock is None
    with pytest.raises(ConnectionError, match="not connected"):
        client.send({"type": "ping"})
