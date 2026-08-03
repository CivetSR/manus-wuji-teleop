"""TCP JSONL client for Jetson wuji_manus_bridge server."""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any, Callable, Optional


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
        time.sleep(0.3)

    def close(self) -> None:
        self._alive = False
        try:
            self.set_enable(False)
        except Exception:
            pass
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def send(self, obj: dict[str, Any]) -> None:
        if not self.sock:
            return
        data = (json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n").encode(
            "utf-8"
        )
        with self._lock:
            self.sock.sendall(data)

    def set_enable(self, enabled: bool, side: str = "both") -> None:
        self.send({"type": "enable", "side": side, "enabled": enabled})
        self.enabled = enabled

    def send_joint_cmd(self, side: str, position: list[float], *, enable: bool = True) -> None:
        pos = list(position[:20]) + [0.0] * max(0, 20 - len(position))
        self._seq += 1
        self.send(
            {
                "type": "joint_cmd",
                "side": side,
                "seq": self._seq,
                "t_ms": int(time.time() * 1000),
                "position": pos[:20],
                "enable": enable,
            }
        )

    def _rx_loop(self) -> None:
        assert self.sock is not None
        while self._alive:
            try:
                chunk = self.sock.recv(65536)
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
                    msg = json.loads(line.decode("utf-8"))
                except json.JSONDecodeError:
                    continue
                mtype = msg.get("type")
                if mtype == "hello_ack":
                    self.hello_ack = msg
                if mtype == "tactile" and self._on_tactile:
                    self._on_tactile(msg)
                if self._on_message:
                    self._on_message(msg)
