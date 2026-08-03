"""TCP JSONL bridge server: Manus x86 <-> Wuji Hand 2."""

from __future__ import annotations

import argparse
import logging
import os
import select
import socket
import threading
import time
from typing import Dict, List, Optional, Tuple

from .hand_worker import HandWorker
from .protocol import (
    DEFAULT_TCP_PORT,
    PROTOCOL_VERSION,
    dumps,
    loads_line,
    normalize_positions,
)

log = logging.getLogger("wuji_manus_bridge.server")

DEFAULT_LEFT_SN = "WH2JA01260723001"
DEFAULT_RIGHT_SN = "WH2KA01260722001"


def read_serial_env() -> Tuple[str, str]:
    left = os.environ.get("WUJI_LEFT_SERIAL", DEFAULT_LEFT_SN)
    right = os.environ.get("WUJI_RIGHT_SERIAL", DEFAULT_RIGHT_SN)
    env_file = os.environ.get("WUJI_SERIAL_ENV", "/etc/apex/wuji_serial.env")
    if os.path.isfile(env_file):
        with open(env_file, encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if line.startswith("WUJI_LEFT_SERIAL="):
                    left = line.split("=", 1)[1].strip()
                elif line.startswith("WUJI_RIGHT_SERIAL="):
                    right = line.split("=", 1)[1].strip()
    return left, right


class ClientSession(threading.Thread):
    def __init__(
        self,
        conn: socket.socket,
        addr,
        hands: Dict[str, HandWorker],
        tactile_hz: float,
        state_hz: float,
    ) -> None:
        super().__init__(daemon=True, name=f"client-{addr[0]}:{addr[1]}")
        self.conn = conn
        self.addr = addr
        self.hands = hands
        self.tactile_hz = tactile_hz
        self.state_hz = state_hz
        self._buf = b""
        self._stop = threading.Event()
        self._hello_done = False

    def stop(self) -> None:
        self._stop.set()
        try:
            self.conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.conn.close()
        except OSError:
            pass

    def send(self, msg: dict) -> None:
        try:
            self.conn.sendall(dumps(msg))
        except OSError as exc:
            log.warning("send failed to %s: %s", self.addr, exc)
            self._stop.set()

    def run(self) -> None:
        log.info("Client connected %s", self.addr)
        self.conn.setblocking(False)
        last_tactile = 0.0
        last_state = 0.0
        tactile_period = 1.0 / max(self.tactile_hz, 1.0)
        state_period = 1.0 / max(self.state_hz, 1.0)

        try:
            while not self._stop.is_set():
                now = time.monotonic()
                r, _, _ = select.select([self.conn], [], [], 0.005)
                if r:
                    try:
                        chunk = self.conn.recv(65536)
                    except BlockingIOError:
                        chunk = b""
                    if not chunk:
                        break
                    self._buf += chunk
                    while b"\n" in self._buf:
                        line, self._buf = self._buf.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            msg = loads_line(line)
                        except Exception as exc:  # noqa: BLE001
                            self.send({"type": "error", "code": "bad_json", "message": str(exc)})
                            continue
                        self._handle(msg)

                if self._hello_done:
                    if now - last_tactile >= tactile_period:
                        last_tactile = now
                        for side, hand in self.hands.items():
                            if hand.connected:
                                snap = hand.snapshot_tactile()
                                snap["type"] = "tactile"
                                self.send(snap)
                    if now - last_state >= state_period:
                        last_state = now
                        for side, hand in self.hands.items():
                            if hand.connected:
                                snap = hand.snapshot_joint_state()
                                snap["type"] = "joint_state"
                                self.send(snap)
        finally:
            log.info("Client disconnected %s", self.addr)
            self.stop()

    def _handle(self, msg: dict) -> None:
        mtype = msg.get("type")
        if mtype == "hello":
            hands_info = {s: h.info() for s, h in self.hands.items()}
            self.send(
                {
                    "type": "hello_ack",
                    "protocol_version": PROTOCOL_VERSION,
                    "server": "wuji_manus_bridge",
                    "hands": hands_info,
                    "control_hz": next(iter(self.hands.values())).control_hz if self.hands else 100.0,
                    "cutoff_hz": next(iter(self.hands.values())).cutoff_hz if self.hands else 5.0,
                    "max_joint_speed_rad_s": (
                        next(iter(self.hands.values())).max_joint_speed_rad_s if self.hands else 2.0
                    ),
                    "joint_layout": {
                        "order": "finger_major",
                        "fingers": ["thumb", "index", "middle", "ring", "pinky"],
                        "joints_per_finger": 4,
                        "num_joints": 20,
                        "index_formula": "finger*4 + joint  (joint=0..3 = J1..J4)",
                        "unit": "rad",
                    },
                    "haptic_hint": {
                        "manus_api": "CoreSdk_VibrateFingersForGlove(gloveId, float powers[5])",
                        "powers_order": ["thumb", "index", "middle", "ring", "pinky"],
                        "powers_range": [0.0, 1.0],
                        "source_field": "tactile.haptic_powers",
                    },
                }
            )
            self._hello_done = True
            return

        if mtype == "ping":
            self.send({"type": "pong", "t_ms": int(time.time() * 1000)})
            return

        if mtype == "enable":
            side = str(msg.get("side", "both")).lower()
            enabled = bool(msg.get("enabled", True))
            targets = self._resolve_sides(side)
            for s in targets:
                self.hands[s].set_enabled(enabled)
            self.send({"type": "status", "ok": True, "message": f"enable {side}={enabled}"})
            return

        if mtype == "joint_cmd":
            side = str(msg.get("side", "")).lower()
            if side not in self.hands:
                self.send({"type": "error", "code": "bad_side", "message": f"unknown side {side}"})
                return
            if not self._hello_done:
                self.send({"type": "error", "code": "no_hello", "message": "send hello first"})
                return
            pos = normalize_positions(msg.get("position"))
            enable = bool(msg.get("enable", True))
            self.hands[side].set_joint_target(pos, enable=enable)
            return

        if mtype == "get_status":
            self.send(
                {
                    "type": "status",
                    "ok": True,
                    "message": "ok",
                    "hands": {s: h.info() for s, h in self.hands.items()},
                }
            )
            return

        self.send({"type": "error", "code": "unknown_type", "message": str(mtype)})

    def _resolve_sides(self, side: str) -> List[str]:
        if side == "both":
            return list(self.hands.keys())
        if side in self.hands:
            return [side]
        return []


class BridgeServer:
    def __init__(
        self,
        host: str,
        port: int,
        left_sn: str,
        right_sn: str,
        control_hz: float,
        cutoff_hz: float,
        max_joint_speed_rad_s: float,
        tactile_hz: float,
        state_hz: float,
        haptic_scale_n: float,
        sides: str,
    ) -> None:
        self.host = host
        self.port = port
        self.tactile_hz = tactile_hz
        self.state_hz = state_hz
        self.hands: Dict[str, HandWorker] = {}
        wanted = []
        if sides in ("left", "both"):
            wanted.append(("left", left_sn))
        if sides in ("right", "both"):
            wanted.append(("right", right_sn))

        for side, sn in wanted:
            self.hands[side] = HandWorker(
                side=side,
                serial_number=sn,
                control_hz=control_hz,
                cutoff_hz=cutoff_hz,
                max_joint_speed_rad_s=max_joint_speed_rad_s,
                haptic_scale_n=haptic_scale_n,
            )

        self._sock: Optional[socket.socket] = None
        self._clients: List[ClientSession] = []
        self._stop = threading.Event()

    def start_hands(self) -> None:
        # Ensure hand network route before connect
        net_script = "/home/nvidia/srworkspace/teleop_setup/setup_wuji_hand2_network.sh"
        if os.path.isfile(net_script):
            os.system(f"bash {net_script} >/tmp/wuji_hand2_net.log 2>&1 || true")

        errors = []
        for side, hand in self.hands.items():
            try:
                hand.connect()
                hand.start_loop()
            except Exception as exc:  # noqa: BLE001
                log.error("Failed to connect %s: %s", side, exc)
                errors.append(f"{side}: {exc}")
        if not any(h.connected for h in self.hands.values()):
            raise RuntimeError("No hands connected: " + "; ".join(errors))

    def serve_forever(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(4)
        self._sock.settimeout(1.0)
        log.info("Listening on %s:%d (protocol v%d)", self.host, self.port, PROTOCOL_VERSION)
        try:
            while not self._stop.is_set():
                try:
                    conn, addr = self._sock.accept()
                except socket.timeout:
                    continue
                session = ClientSession(
                    conn, addr, self.hands, self.tactile_hz, self.state_hz
                )
                self._clients.append(session)
                session.start()
                self._clients = [c for c in self._clients if c.is_alive()]
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self._stop.set()
        for c in self._clients:
            c.stop()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        for hand in self.hands.values():
            hand.disconnect()


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Wuji Hand 2 Manus bridge TCP server")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=DEFAULT_TCP_PORT)
    p.add_argument("--sides", choices=("both", "left", "right"), default="both")
    p.add_argument("--left-sn", default=None)
    p.add_argument("--right-sn", default=None)
    p.add_argument("--control-hz", type=float, default=100.0, help="Motor command rate (Hz)")
    p.add_argument(
        "--cutoff-hz",
        type=float,
        default=5.0,
        help="Low-pass cutoff (Hz). Official Wuji tutorial default is 5.0",
    )
    p.add_argument(
        "--max-joint-speed",
        type=float,
        default=2.0,
        help="Slew-rate limit rad/s (reduces motor chatter)",
    )
    p.add_argument("--tactile-hz", type=float, default=50.0, help="Tactile stream to client")
    p.add_argument("--state-hz", type=float, default=50.0, help="Joint state stream to client")
    p.add_argument(
        "--haptic-scale-n",
        type=float,
        default=2.0,
        help="Newtons mapped to haptic_powers=1.0",
    )
    p.add_argument("-v", "--verbose", action="store_true")
    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = build_argparser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    left_sn, right_sn = read_serial_env()
    if args.left_sn:
        left_sn = args.left_sn
    if args.right_sn:
        right_sn = args.right_sn

    # Stop Apex tool if it holds the hands
    if os.system("systemctl is-active --quiet apex-tool 2>/dev/null") == 0:
        log.warning("apex-tool is active; it may hold Hand connections. Consider: sudo systemctl stop apex-tool")

    server = BridgeServer(
        host=args.host,
        port=args.port,
        left_sn=left_sn,
        right_sn=right_sn,
        control_hz=args.control_hz,
        cutoff_hz=args.cutoff_hz,
        max_joint_speed_rad_s=args.max_joint_speed,
        tactile_hz=args.tactile_hz,
        state_hz=args.state_hz,
        haptic_scale_n=args.haptic_scale_n,
        sides=args.sides,
    )
    server.start_hands()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("Interrupted")
        server.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
