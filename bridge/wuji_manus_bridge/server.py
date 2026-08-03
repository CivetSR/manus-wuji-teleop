"""Local TCP backend for network-connected Wuji Hand 2 devices."""

from __future__ import annotations

import argparse
import atexit
import logging
import os
import signal
import socket
import threading
from typing import Dict, List, Optional

from .control import (
    DEFAULT_COMMAND_TIMEOUT_S,
    DEFAULT_MAX_COMMAND_AGE_MS,
    ControlAuthority,
    SafeClientSession,
)
from .hand_worker import HandWorker, discover_hand2_devices
from .protocol import (
    DEFAULT_TCP_PORT,
    PROTOCOL_VERSION,
)

log = logging.getLogger("wuji_manus_bridge.server")


class BridgeServer:
    def __init__(
        self,
        host: str,
        port: int,
        left_address: Optional[str],
        right_address: Optional[str],
        control_hz: float,
        cutoff_hz: float,
        max_joint_speed_rad_s: float,
        tactile_hz: float,
        state_hz: float,
        haptic_scale_n: float,
        sides: str,
        command_timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S,
        max_command_age_ms: int = DEFAULT_MAX_COMMAND_AGE_MS,
    ) -> None:
        self.host = host
        self.port = port
        self.tactile_hz = tactile_hz
        self.state_hz = state_hz
        self.hands: Dict[str, HandWorker] = {}
        wanted: List[str] = []
        if sides in ("left", "both"):
            wanted.append("left")
        if sides in ("right", "both"):
            wanted.append("right")

        addresses = {
            "left": left_address,
            "right": right_address,
        }
        missing = [side for side in wanted if not addresses[side]]
        if missing:
            discovered = discover_hand2_devices(missing)
            for side in missing:
                addresses[side] = discovered[side].address

        for side in wanted:
            address = addresses[side]
            if not address:
                raise RuntimeError(f"no network address resolved for {side} Hand2")
            self.hands[side] = HandWorker(
                side=side,
                address=address,
                control_hz=control_hz,
                cutoff_hz=cutoff_hz,
                max_joint_speed_rad_s=max_joint_speed_rad_s,
                haptic_scale_n=haptic_scale_n,
            )

        self.authority = ControlAuthority(
            self.hands,
            command_timeout_s=command_timeout_s,
            max_command_age_ms=max_command_age_ms,
        )
        self._sock: Optional[socket.socket] = None
        self._clients: List[SafeClientSession] = []
        self._stop = threading.Event()
        self._watchdog_thread: Optional[threading.Thread] = None
        self._shutdown_lock = threading.Lock()
        self._shutdown_started = False

    def start_hands(self) -> None:
        errors = []
        for side, hand in self.hands.items():
            try:
                hand.connect()
                hand.start_loop()
            except Exception as exc:  # noqa: BLE001
                log.error("Failed to connect %s: %s", side, exc)
                hand.disconnect()
                errors.append(f"{side}: {exc}")
        if not any(h.connected for h in self.hands.values()):
            raise RuntimeError("No hands connected: " + "; ".join(errors))

    def serve_forever(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(4)
        self._sock.settimeout(1.0)
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="wuji-deadman-watchdog",
            daemon=False,
        )
        self._watchdog_thread.start()
        log.info("Listening on %s:%d (protocol v%d)", self.host, self.port, PROTOCOL_VERSION)
        try:
            while not self._stop.is_set():
                try:
                    conn, addr = self._sock.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self._stop.is_set():
                        break
                    raise
                session = SafeClientSession(
                    conn,
                    addr,
                    self.hands,
                    self.authority,
                    self.tactile_hz,
                    self.state_hz,
                    server_name="wuji_manus_bridge",
                )
                self._clients.append(session)
                session.start()
                self._clients = [c for c in self._clients if c.is_alive()]
        finally:
            self.shutdown()

    def _watchdog_loop(self) -> None:
        while not self._stop.wait(0.025):
            try:
                self.authority.expire()
            except Exception as exc:  # noqa: BLE001
                log.error("Deadman watchdog failed to disarm: %s", exc)

    def shutdown(self) -> None:
        with self._shutdown_lock:
            if self._shutdown_started:
                return
            self._shutdown_started = True
        self._stop.set()
        watchdog = self._watchdog_thread
        if watchdog is not None and watchdog is not threading.current_thread():
            watchdog.join(timeout=1.0)
            if watchdog.is_alive():
                log.error("Deadman watchdog did not stop")
            self._watchdog_thread = None
        clients = list(self._clients)
        for client in clients:
            client.stop()
        for client in clients:
            client.join(timeout=2.0)
            if client.is_alive():
                log.error("Client session %s did not stop before hand shutdown", client.addr)
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        for hand in self.hands.values():
            hand.disconnect()


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Local TCP backend for Ethernet Wuji Hand 2 devices"
    )
    p.add_argument(
        "--host",
        default="127.0.0.1",
        help="TCP bind address; use 0.0.0.0 only for an optional remote deployment",
    )
    p.add_argument("--port", type=int, default=DEFAULT_TCP_PORT)
    p.add_argument("--sides", choices=("both", "left", "right"), default="both")
    p.add_argument(
        "--left-ip",
        default=os.environ.get("WUJI_LEFT_IP"),
        help="Left Hand2 address; omit to discover by reported handedness",
    )
    p.add_argument(
        "--right-ip",
        default=os.environ.get("WUJI_RIGHT_IP"),
        help="Right Hand2 address; omit to discover by reported handedness",
    )
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
        "--command-timeout-ms",
        type=int,
        default=200,
        choices=range(100, 251),
        metavar="100..250",
        help="Deadman timeout for accepted joint commands",
    )
    p.add_argument(
        "--max-command-age-ms",
        type=int,
        default=DEFAULT_MAX_COMMAND_AGE_MS,
        help="Reject commands older than this wall-clock age",
    )
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

    server = BridgeServer(
        host=args.host,
        port=args.port,
        left_address=args.left_ip,
        right_address=args.right_ip,
        control_hz=args.control_hz,
        cutoff_hz=args.cutoff_hz,
        max_joint_speed_rad_s=args.max_joint_speed,
        tactile_hz=args.tactile_hz,
        state_hz=args.state_hz,
        haptic_scale_n=args.haptic_scale_n,
        sides=args.sides,
        command_timeout_s=args.command_timeout_ms / 1000.0,
        max_command_age_ms=args.max_command_age_ms,
    )
    atexit.register(server.shutdown)

    def _shutdown_from_signal(signum, _frame) -> None:
        log.info("Received signal %d; shutting down", signum)
        server.shutdown()

    signal.signal(signal.SIGINT, _shutdown_from_signal)
    signal.signal(signal.SIGTERM, _shutdown_from_signal)
    server.start_hands()
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
