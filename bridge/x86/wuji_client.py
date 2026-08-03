"""TCP JSONL client for the local Hand2 or MuJoCo backend."""

from __future__ import annotations

import json
import logging
import math
import socket
import threading
import time
from typing import Any, Callable, Optional

log = logging.getLogger("manus_wuji_bridge.client")


class WujiClient:
    def __init__(self, host: str, port: int = 9500) -> None:
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None
        self._buf = b""
        self._lock = threading.Lock()
        self._alive = False
        self._rx_thread: Optional[threading.Thread] = None
        self._on_tactile: Optional[Callable[[dict[str, Any]], None]] = None
        self._on_message: Optional[Callable[[dict[str, Any]], None]] = None
        self.hello_ack: dict[str, Any] = {}
        self._hello_event = threading.Event()
        self.enabled = False
        self._seq = 0

    def connect(
        self,
        *,
        on_tactile: Callable[[dict[str, Any]], None] | None = None,
        on_message: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self._on_tactile = on_tactile
        self._on_message = on_message
        self._buf = b""
        self.hello_ack = {}
        self._hello_event.clear()
        self.sock = socket.create_connection((self.host, self.port), timeout=5.0)
        self.sock.settimeout(1.0)
        self._alive = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self.send(
            {
                "type": "hello",
                "client": "manus_wuji_bridge",
                "protocol_version": 1,
                "features": ["joint_cmd", "tactile", "haptic"],
            }
        )
        if not self._hello_event.wait(timeout=5.0):
            self._close_socket()
            raise TimeoutError(
                f"Wuji backend {self.host}:{self.port} did not return hello_ack"
            )
        if self.hello_ack.get("protocol_version") != 1 or not isinstance(
            self.hello_ack.get("hands"), dict
        ):
            invalid_ack = self.hello_ack
            self._close_socket()
            raise RuntimeError(f"invalid Wuji hello_ack: {invalid_ack}")

    def close(self) -> None:
        try:
            self.set_enable(False)
        except Exception:
            pass
        self._close_socket()
        if self._rx_thread and self._rx_thread is not threading.current_thread():
            self._rx_thread.join(timeout=2.0)
        self._rx_thread = None

    def _close_socket(self) -> None:
        self._alive = False
        with self._lock:
            sock, self.sock = self.sock, None
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def send(self, obj: dict[str, Any]) -> None:
        data = (
            json.dumps(
                obj,
                separators=(",", ":"),
                ensure_ascii=False,
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
        with self._lock:
            if self.sock is None:
                raise ConnectionError("Wuji backend socket is not connected")
            self.sock.sendall(data)

    def set_enable(self, enabled: bool, side: str = "both") -> None:
        if type(enabled) is not bool:
            raise ValueError("enabled must be a boolean")
        if side not in ("left", "right", "both"):
            raise ValueError(f"side must be left, right, or both, got {side!r}")
        self.send({"type": "enable", "side": side, "enabled": enabled})
        self.enabled = enabled

    def send_joint_cmd(self, side: str, position: list[float], *, enable: bool = True) -> None:
        if side not in ("left", "right"):
            raise ValueError(f"side must be left or right, got {side!r}")
        if type(enable) is not bool:
            raise ValueError("joint command enable must be a boolean")
        if not isinstance(position, list):
            raise ValueError("joint command position must be a list")
        if any(type(value) not in (int, float) for value in position):
            raise ValueError(
                "joint command values must be numbers (booleans/strings are forbidden)"
            )
        pos = [float(value) for value in position]
        if len(pos) != 20:
            raise ValueError(f"joint command must contain exactly 20 values, got {len(pos)}")
        if not all(math.isfinite(value) for value in pos):
            raise ValueError("joint command contains NaN or infinity")
        self._seq += 1
        self.send(
            {
                "type": "joint_cmd",
                "side": side,
                "seq": self._seq,
                "t_ms": int(time.time() * 1000),
                "position": pos,
                "enable": enable,
            }
        )

    def _rx_loop(self) -> None:
        sock = self.sock
        assert sock is not None
        try:
            while self._alive:
                try:
                    chunk = sock.recv(65536)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                self._buf += chunk
                while b"\n" in self._buf:
                    line, self._buf = self._buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        msg = json.loads(
                            line.decode("utf-8"),
                            parse_constant=lambda token: (_reject_nonfinite(token)),
                        )
                    except (json.JSONDecodeError, UnicodeError, ValueError):
                        continue
                    if not isinstance(msg, dict):
                        continue
                    mtype = msg.get("type")
                    if mtype == "hello_ack":
                        self.hello_ack = msg
                        self._hello_event.set()
                    if mtype == "tactile" and self._on_tactile:
                        self._invoke_callback(self._on_tactile, msg, "tactile")
                    if self._on_message:
                        self._invoke_callback(self._on_message, msg, "message")
        finally:
            self._alive = False
            with self._lock:
                if self.sock is sock:
                    self.sock = None
            try:
                sock.close()
            except OSError:
                pass

    @staticmethod
    def _invoke_callback(
        callback: Callable[[dict[str, Any]], None],
        msg: dict[str, Any],
        name: str,
    ) -> None:
        try:
            callback(msg)
        except Exception:  # noqa: BLE001 - callbacks must not kill the RX thread
            log.exception("WujiClient %s callback failed; continuing receive loop", name)


def _reject_nonfinite(token: str) -> None:
    raise ValueError(f"non-standard JSON numeric token is forbidden: {token}")
