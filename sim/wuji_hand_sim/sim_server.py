"""TCP JSONL bridge server for MuJoCo simulation (same protocol as real robot)."""

from __future__ import annotations

import argparse
import logging
import select
import socket
import threading
import time
from typing import Dict, List, Optional

from wuji_manus_bridge.protocol import (
    DEFAULT_TCP_PORT,
    PROTOCOL_VERSION,
    dumps,
    loads_line,
    normalize_positions,
)

from .mujoco_scene import MujocoScene
from .sim_hand_worker import SimHandWorker

log = logging.getLogger("wuji_hand_sim.server")


class ClientSession(threading.Thread):
    def __init__(
        self,
        conn: socket.socket,
        addr,
        hands: Dict[str, SimHandWorker],
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
                    except (BlockingIOError, ConnectionResetError, OSError):
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
                        self._on_message(msg)

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

    def _on_message(self, msg: dict) -> None:
        mtype = msg.get("type")
        if mtype == "hello":
            hands_info = {s: h.info() for s, h in self.hands.items()}
            self.send(
                {
                    "type": "hello_ack",
                    "protocol_version": PROTOCOL_VERSION,
                    "server": "wuji_hand_sim",
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
            for s in self._resolve_sides(side):
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


class SimBridgeServer:
    def __init__(
        self,
        host: str,
        port: int,
        scene: MujocoScene,
        sides: str,
        control_hz: float,
        cutoff_hz: float,
        max_joint_speed_rad_s: float,
        tactile_hz: float,
        state_hz: float,
        haptic_scale_n: float,
    ) -> None:
        self.host = host
        self.port = port
        self.scene = scene
        self.tactile_hz = tactile_hz
        self.state_hz = state_hz
        self.hands: Dict[str, SimHandWorker] = {}

        if sides in ("left", "both"):
            self.hands["left"] = SimHandWorker(
                "left",
                scene,
                serial_number="SIM-LEFT",
                control_hz=control_hz,
                cutoff_hz=cutoff_hz,
                max_joint_speed_rad_s=max_joint_speed_rad_s,
                haptic_scale_n=haptic_scale_n,
            )
        if sides in ("right", "both"):
            if sides == "both":
                log.warning(
                    "Bimanual MuJoCo not implemented yet; only left hand is simulated. "
                    "Use --sides left or --sides right."
                )
            else:
                self.hands["right"] = SimHandWorker(
                    "right",
                    scene,
                    serial_number="SIM-RIGHT",
                    control_hz=control_hz,
                    cutoff_hz=cutoff_hz,
                    max_joint_speed_rad_s=max_joint_speed_rad_s,
                    haptic_scale_n=haptic_scale_n,
                )

        self._sock: Optional[socket.socket] = None
        self._clients: List[ClientSession] = []
        self._stop = threading.Event()

    def start_hands(self) -> None:
        self.scene.start()
        for hand in self.hands.values():
            hand.connect()
            hand.start_loop()
        log.info("Sim hands ready: %s", list(self.hands.keys()))

    def serve_forever(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(4)
        self._sock.settimeout(1.0)
        log.info("MuJoCo sim listening on %s:%d (protocol v%d)", self.host, self.port, PROTOCOL_VERSION)
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
        self.scene.stop()


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Wuji Hand 2 MuJoCo sim + Manus TCP bridge")
    p.add_argument("--host", default="0.0.0.0", help="TCP bind address")
    p.add_argument("--port", type=int, default=DEFAULT_TCP_PORT)
    p.add_argument("--sides", choices=("left", "right", "both"), default="left")
    p.add_argument("--headless", action="store_true", help="No MuJoCo viewer window")
    p.add_argument("--control-hz", type=float, default=100.0)
    p.add_argument("--cutoff-hz", type=float, default=5.0)
    p.add_argument("--max-joint-speed", type=float, default=2.0)
    p.add_argument("--tactile-hz", type=float, default=50.0)
    p.add_argument("--state-hz", type=float, default=50.0)
    p.add_argument("--haptic-scale-n", type=float, default=2.0)
    p.add_argument("-v", "--verbose", action="store_true")
    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = build_argparser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    side_for_scene = "left" if args.sides == "both" else args.sides
    scene = MujocoScene(side_for_scene, headless=args.headless)
    server = SimBridgeServer(
        host=args.host,
        port=args.port,
        scene=scene,
        sides=args.sides if args.sides != "both" else "left",
        control_hz=args.control_hz,
        cutoff_hz=args.cutoff_hz,
        max_joint_speed_rad_s=args.max_joint_speed,
        tactile_hz=args.tactile_hz,
        state_hz=args.state_hz,
        haptic_scale_n=args.haptic_scale_n,
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
