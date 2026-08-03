#!/usr/bin/env python3
"""Minimal TCP client stub for the Wuji Manus bridge (no Manus SDK required).

Use this on the x86 machine to verify connectivity before wiring Manus.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time


def recv_loop(sock: socket.socket) -> None:
    buf = b""
    while True:
        chunk = sock.recv(65536)
        if not chunk:
            break
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            if not line.strip():
                continue
            msg = json.loads(line.decode("utf-8"))
            mtype = msg.get("type")
            if mtype == "tactile":
                powers = msg.get("haptic_powers", [])
                print(
                    f"[tactile {msg.get('side')}] haptic={['%.2f'%p for p in powers]}",
                    flush=True,
                )
            elif mtype == "joint_state":
                pos = msg.get("position", [])
                print(
                    f"[state {msg.get('side')}] j0={pos[0]:+.3f} j4={pos[4]:+.3f}"
                    if len(pos) > 4
                    else f"[state {msg.get('side')}]",
                    flush=True,
                )
            elif mtype in ("hello_ack", "status", "error", "pong"):
                print(json.dumps(msg, ensure_ascii=False), flush=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1", help="Hand2/MuJoCo backend host")
    ap.add_argument("--port", type=int, default=9500)
    ap.add_argument("--side", default="left", choices=("left", "right"))
    ap.add_argument("--demo", action="store_true", help="Send a slow pinky wave")
    args = ap.parse_args()

    sock = socket.create_connection((args.host, args.port), timeout=5)
    sock.sendall(
        (json.dumps({"type": "hello", "client": "x86_stub", "protocol_version": 1}) + "\n").encode()
    )
    t = threading.Thread(target=recv_loop, args=(sock,), daemon=True)
    t.start()
    time.sleep(0.5)
    sock.sendall(
        (json.dumps({"type": "enable", "side": args.side, "enabled": True}) + "\n").encode()
    )

    if not args.demo:
        print("Connected. Use --demo to wave pinky. Ctrl+C to quit.")
        try:
            while True:
                time.sleep(1)
                sock.sendall((json.dumps({"type": "ping"}) + "\n").encode())
        except KeyboardInterrupt:
            pass
        finally:
            sock.close()
        return 0

    # Slow cosine on pinky flex joints (indices 16,18,19) — quiet if LPF works
    seq = 0
    start = time.monotonic()
    try:
        while True:
            tsec = time.monotonic() - start
            y = (1 - math.cos(2 * math.pi * tsec / 4.0)) * 0.25  # small amp
            pos = [0.0] * 20
            pos[16] = y
            pos[18] = y
            pos[19] = y
            msg = {
                "type": "joint_cmd",
                "side": args.side,
                "seq": seq,
                "t_ms": int(time.time() * 1000),
                "position": pos,
                "enable": True,
            }
            sock.sendall((json.dumps(msg) + "\n").encode())
            seq += 1
            time.sleep(0.01)  # 100 Hz
    except KeyboardInterrupt:
        pass
    finally:
        sock.sendall(
            (json.dumps({"type": "enable", "side": args.side, "enabled": False}) + "\n").encode()
        )
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
