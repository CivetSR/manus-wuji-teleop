#!/usr/bin/env python3
"""Glove-free official IK -> TCP -> MuJoCo end-to-end smoke."""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

TELEOP_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TELEOP_ROOT / "bridge" / "x86"))

from synthetic_hand import synthetic_mediapipe_hand  # noqa: E402
from wuji_client import WujiClient  # noqa: E402
from wuji_retargeting_adapter import (  # noqa: E402
    add_retarget_backend_argument,
    create_retarget_backend,
)


class StateCollector:
    def __init__(self, side: str) -> None:
        self.side = side
        self.samples: list[np.ndarray] = []
        self._condition = threading.Condition()

    def on_message(self, msg: dict[str, Any]) -> None:
        if msg.get("type") != "joint_state" or msg.get("side") != self.side:
            return
        raw = msg.get("position")
        if not isinstance(raw, list) or len(raw) != 20:
            return
        sample = np.asarray(raw, dtype=np.float64)
        if sample.shape != (20,) or not np.isfinite(sample).all():
            return
        with self._condition:
            self.samples.append(sample)
            self._condition.notify_all()

    def wait_for_samples(self, count: int, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        with self._condition:
            while len(self.samples) < count:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"received {len(self.samples)} joint states, expected at least {count}"
                    )
                self._condition.wait(remaining)


def run(args: argparse.Namespace) -> None:
    pipeline = create_retarget_backend(args.retarget_backend, args.side)
    print(
        f"Retarget backend initialized: {pipeline.name} {args.side} "
        f"{pipeline.initialization}"
    )
    collector = StateCollector(args.side)
    client = WujiClient(args.host, args.port)
    client.connect(on_message=collector.on_message)
    try:
        hands = client.hello_ack.get("hands", {})
        if args.side not in hands:
            raise RuntimeError(f"backend does not provide {args.side}; hello_ack={hands}")
        collector.wait_for_samples(2, timeout=5.0)
        baseline = collector.samples[-1].copy()

        commands: list[np.ndarray] = []
        retarget_times_ms: list[float] = []
        frame_count = max(12, int(args.duration * args.hz))
        for frame in range(frame_count):
            phase = frame / max(frame_count - 1, 1)
            curl = 0.1 + 0.8 * (0.5 - 0.5 * math.cos(2.0 * math.pi * phase))
            keypoints = synthetic_mediapipe_hand(curl, args.side)
            started = time.monotonic()
            command = pipeline.retarget(keypoints)
            retarget_times_ms.append((time.monotonic() - started) * 1000.0)
            if command.shape != (20,) or not np.isfinite(command).all():
                raise RuntimeError("IK produced an invalid command")
            commands.append(command)
            if frame == 0:
                # Cold-start IK can exceed the 200 ms command deadman.  Do not
                # arm until the first valid command is ready to send.
                client.set_enable(True, side=args.side)
            client.send_joint_cmd(args.side, command.tolist(), enable=True)
            time.sleep(1.0 / args.hz)

        collector.wait_for_samples(5, timeout=5.0)
        time.sleep(0.5)
        states = np.stack(collector.samples)
        command_array = np.stack(commands)
        state_change = float(np.max(np.abs(states - baseline)))
        command_span = float(np.max(np.ptp(command_array, axis=0)))
        if command_span <= 1e-3:
            raise RuntimeError(f"synthetic trajectory did not change IK output: {command_span}")
        if state_change <= 1e-3:
            raise RuntimeError(f"MuJoCo joint_state did not change: {state_change}")

        print(
            "IK_TCP_MUJOCO_OK "
            f"backend={pipeline.name} side={args.side} "
            f"frames={frame_count} states={len(states)} "
            f"command_span={command_span:.6f}rad "
            f"joint_state_change={state_change:.6f}rad "
            f"finite={bool(np.isfinite(states).all())} "
            f"retarget_avg_ms={float(np.mean(retarget_times_ms)):.3f} "
            f"retarget_max_ms={float(np.max(retarget_times_ms)):.3f}"
        )
    finally:
        try:
            client.set_enable(False, side=args.side)
        finally:
            client.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9500)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    add_retarget_backend_argument(parser)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=30.0)
    args = parser.parse_args()
    if args.duration <= 0.0 or args.hz <= 0.0:
        parser.error("duration and hz must be positive")
    return args


if __name__ == "__main__":
    run(parse_args())
