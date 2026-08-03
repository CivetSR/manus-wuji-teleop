#!/usr/bin/env python3
"""MANUS ROS2 input -> selectable retarget backend -> localhost TCP."""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node

from haptics import finger_forces_to_manus_vibration, peaks_from_tactile
from manus_keypoints import InvalidManusFrame, raw_nodes_to_mediapipe
from wuji_client import WujiClient
from wuji_retargeting_adapter import (
    add_retarget_backend_argument,
    config_path_for_side,
    create_retarget_backend,
)

try:
    from manus_ros2_msgs.msg import ManusGlove, ManusVibrationCommand
except ImportError as exc:
    raise SystemExit(
        "manus_ros2_msgs not found. Run: source /opt/ros/humble/setup.bash && "
        "source ${ROS2_WS:-~/ros2_ws}/install/setup.bash"
    ) from exc


class ManusWujiBridge(Node):
    """Latest-only, bimanual MANUS retargeting client."""

    def __init__(
        self,
        backend_host: str,
        backend_port: int,
        retarget_backend: str,
        left_config: Path,
        right_config: Path,
        haptic_scale_n: float,
        auto_enable: bool,
    ) -> None:
        super().__init__("manus_wuji_bridge")
        self.haptic_scale_n = haptic_scale_n
        self.auto_enable = auto_enable

        # Both backends are stateful, so each side owns one independent instance.
        self.retarget_backend_name = retarget_backend
        self.retargeters = {
            "left": create_retarget_backend(retarget_backend, "left", left_config),
            "right": create_retarget_backend(retarget_backend, "right", right_config),
        }
        self.get_logger().info(f"Retarget backend: {retarget_backend}")
        for side, backend in self.retargeters.items():
            self.get_logger().info(
                f"{side} backend initialized independently: "
                f"{backend.name} {backend.initialization}"
            )

        self.glove_side: dict[int, str] = {}
        self.vib_publishers: dict[int, Any] = {}
        self._vib_lock = threading.Lock()
        self._pending_vib: dict[str, list[float]] = {}

        self._frame_lock = threading.Lock()
        self._latest_frames: dict[str, tuple[np.ndarray, float]] = {}
        self._new_frame = threading.Event()
        self._stop = threading.Event()
        self._last_invalid_reason: dict[str, str] = {}
        self._last_backend_error = ""
        self._backend_control_state: tuple[bool, bool, tuple[str, ...]] | None = None
        self._stats: dict[str, dict[str, float]] = {
            side: {
                "received": 0,
                "invalid": 0,
                "unavailable": 0,
                "overwritten": 0,
                "solved": 0,
                "failed": 0,
                "sent": 0,
                "ik_ms_sum": 0.0,
                "ik_ms_max": 0.0,
                "last_input": 0.0,
                "last_success": 0.0,
                "last_peak": 0.0,
            }
            for side in ("left", "right")
        }

        self.wuji = WujiClient(backend_host, backend_port)
        self.wuji.connect(
            on_tactile=self._on_tactile,
            on_message=self._on_backend_message,
        )
        hands = self.wuji.hello_ack.get("hands", {})
        self._server_sides = {
            side
            for side, info in hands.items()
            if side in ("left", "right")
            and isinstance(info, dict)
            and info.get("connected") is True
        }
        if not self._server_sides:
            self.wuji.close()
            raise RuntimeError(
                f"backend reports no connected Hand2/MuJoCo side: {hands}"
            )
        self.get_logger().info(
            f"Connected to backend {backend_host}:{backend_port}; "
            f"server_sides={sorted(self._server_sides)}"
        )
        self._worker = threading.Thread(
            target=self._retarget_loop,
            name="manus-retarget",
            daemon=True,
        )
        self._worker.start()

        # Must match manus_data_publisher's reliable QoS.
        self._glove_qos = 10
        self._subscribed_topics: set[str] = set()
        self._glove_subscriptions: dict[str, Any] = {}
        self._discover_glove_topics()
        self.create_timer(2.0, self._discover_glove_topics)
        self.create_timer(3.0, self._status_tick)
        self.create_timer(0.02, self._haptic_tick)

    def _discover_glove_topics(self) -> None:
        names = [
            name
            for name, types in self.get_topic_names_and_types()
            if name.startswith("/manus_glove_")
            and not name.endswith("/vibration_cmd")
            and "manus_ros2_msgs/msg/ManusGlove" in types
        ]
        for topic in sorted(names):
            if topic in self._subscribed_topics:
                continue
            subscription = self.create_subscription(
                ManusGlove,
                topic,
                lambda msg, source_topic=topic: self._on_glove_msg(msg, source_topic),
                self._glove_qos,
            )
            # Keep an explicit reference. rclpy's Node currently owns created
            # subscriptions too, but relying on that implementation detail made
            # dynamic discovery unnecessarily fragile.
            self._glove_subscriptions[topic] = subscription
            self._subscribed_topics.add(topic)
            self.get_logger().info(f"Subscribed to {topic}")

    def _on_glove_msg(self, msg: ManusGlove, source_topic: str) -> None:
        """Convert and cache only; IK and TCP never run on the ROS callback."""

        side = (msg.side or "").lower()
        if side not in ("left", "right"):
            side = self.glove_side.get(msg.glove_id, "")
        if side not in ("left", "right"):
            self.get_logger().warning(
                f"Dropping glove_id=0x{msg.glove_id:X}: invalid side {msg.side!r}",
                throttle_duration_sec=2.0,
            )
            return

        self.glove_side[msg.glove_id] = side
        self._ensure_vib_pub(msg.glove_id, source_topic)
        try:
            keypoints = raw_nodes_to_mediapipe(msg.raw_nodes)
        except (InvalidManusFrame, TypeError, ValueError) as exc:
            node_ids = [
                int(node.node_id)
                for node in msg.raw_nodes
                if hasattr(node, "node_id")
            ]
            reason = (
                f"{exc}; topic={source_topic} raw_node_count={msg.raw_node_count} "
                f"len(raw_nodes)={len(msg.raw_nodes)} node_ids={node_ids}"
            )
            with self._frame_lock:
                self._stats[side]["invalid"] += 1
                self._last_invalid_reason[side] = reason
            self.get_logger().warning(
                f"Dropping invalid {side} MANUS frame: {reason}",
                throttle_duration_sec=2.0,
            )
            return

        now = time.monotonic()
        with self._frame_lock:
            stats = self._stats[side]
            stats["received"] += 1
            stats["last_input"] = now
            if side not in self._server_sides:
                stats["unavailable"] += 1
                unavailable = True
            else:
                unavailable = False
                if side in self._latest_frames:
                    stats["overwritten"] += 1
                self._latest_frames[side] = (keypoints, now)
        if unavailable:
            self.get_logger().warning(
                f"Dropping valid {side} MANUS frame: backend provides only "
                f"{sorted(self._server_sides)}. Restart MuJoCo with "
                f"HEADLESS=0 SIDES={side} ./scripts/start_sim.sh; "
                "the bridge will not mirror commands to the other hand.",
                throttle_duration_sec=3.0,
            )
            return
        self._new_frame.set()

    def _retarget_one(self, side: str, keypoints: np.ndarray) -> tuple[np.ndarray, float]:
        started = time.monotonic()
        command = self.retargeters[side].retarget(keypoints)
        elapsed_ms = (time.monotonic() - started) * 1000.0
        return command, elapsed_ms

    def _retarget_loop(self) -> None:
        pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="ik")
        try:
            while not self._stop.is_set():
                if not self._new_frame.wait(timeout=0.1):
                    continue
                self._new_frame.clear()
                if self._stop.is_set():
                    break

                with self._frame_lock:
                    pending = self._latest_frames
                    self._latest_frames = {}
                if not pending:
                    continue

                futures: dict[Future[tuple[np.ndarray, float]], str] = {}
                for side, (keypoints, _timestamp) in pending.items():
                    if side not in self._server_sides:
                        continue
                    futures[pool.submit(self._retarget_one, side, keypoints)] = side

                for future in as_completed(futures):
                    side = futures[future]
                    try:
                        command, elapsed_ms = future.result()
                        if self._stop.is_set():
                            continue
                        if not self.auto_enable:
                            continue
                        # Arm is sent only after a valid IK result exists. Repeating
                        # this idempotent lease request lets a fresh valid frame
                        # recover after the server-side deadman without ever
                        # enabling a hand merely because TCP connected.
                        self.wuji.set_enable(True, side=side)
                        self.wuji.send_joint_cmd(side, command.tolist(), enable=True)
                    except Exception as exc:  # noqa: BLE001 - frame-local failure
                        with self._frame_lock:
                            self._stats[side]["failed"] += 1
                        self.get_logger().error(
                            f"{side} IK/TCP frame dropped: {exc}",
                            throttle_duration_sec=2.0,
                        )
                        continue

                    now = time.monotonic()
                    with self._frame_lock:
                        stats = self._stats[side]
                        stats["solved"] += 1
                        stats["sent"] += 1
                        stats["ik_ms_sum"] += elapsed_ms
                        stats["ik_ms_max"] = max(stats["ik_ms_max"], elapsed_ms)
                        stats["last_success"] = now
                        stats["last_peak"] = float(np.max(np.abs(command)))
        finally:
            pool.shutdown(wait=True, cancel_futures=True)

    def _status_tick(self) -> None:
        try:
            self.wuji.send({"type": "get_status"})
        except (ConnectionError, OSError) as exc:
            detail = f"backend status request failed: {exc}"
            with self._frame_lock:
                self._last_backend_error = detail
            self.get_logger().error(detail, throttle_duration_sec=3.0)

        now = time.monotonic()
        with self._frame_lock:
            snapshot = {side: dict(stats) for side, stats in self._stats.items()}
            invalid_reasons = dict(self._last_invalid_reason)
            backend_error = self._last_backend_error
            control_state = self._backend_control_state
        for side, stats in snapshot.items():
            received = int(stats["received"])
            solved = int(stats["solved"])
            if received == 0:
                last_drop = invalid_reasons.get(side, "none observed")
                self.get_logger().warning(
                    f"{side}: waiting for valid MANUS raw_nodes; "
                    f"invalid={int(stats['invalid'])}; last_drop={last_drop}"
                )
                continue
            if side not in self._server_sides:
                self.get_logger().warning(
                    f"{side}: valid MANUS input cannot control backend sides "
                    f"{sorted(self._server_sides)}; rx={received} "
                    f"dropped_side_mismatch={int(stats['unavailable'])}. "
                    f"Restart MuJoCo with HEADLESS=0 SIDES={side} "
                    "./scripts/start_sim.sh"
                )
                continue
            input_age = now - stats["last_input"]
            success_age = (
                now - stats["last_success"] if stats["last_success"] > 0.0 else math.inf
            )
            state = "valid" if success_age < 2.0 else "stale/error"
            avg_ms = stats["ik_ms_sum"] / max(solved, 1)
            self.get_logger().info(
                f"{side}: backend={self.retarget_backend_name} state={state} "
                f"input_age={input_age:.2f}s "
                f"rx={received} invalid={int(stats['invalid'])} "
                f"latest_dropped={int(stats['overwritten'])} sent={int(stats['sent'])} "
                f"failed={int(stats['failed'])} IK={avg_ms:.2f}ms avg/"
                f"{stats['ik_ms_max']:.2f}ms max peak={stats['last_peak']:.3f}rad "
                f"control={control_state} backend_error={backend_error or 'none'}"
            )

    def _on_backend_message(self, msg: dict[str, Any]) -> None:
        message_type = msg.get("type")
        if message_type == "error":
            detail = (
                f"backend rejected command: code={msg.get('code')!r} "
                f"message={msg.get('message')!r}"
            )
            with self._frame_lock:
                self._last_backend_error = detail
            self.get_logger().error(detail, throttle_duration_sec=2.0)
            return
        if message_type != "status":
            return

        armed_raw = msg.get("armed_sides")
        armed = (
            tuple(sorted(str(side) for side in armed_raw))
            if isinstance(armed_raw, list)
            else ()
        )
        state = (
            msg.get("controller") is True,
            msg.get("control_lease_held") is True,
            armed,
        )
        with self._frame_lock:
            changed = state != self._backend_control_state
            self._backend_control_state = state
            if msg.get("ok") is True:
                self._last_backend_error = ""
        if changed:
            self.get_logger().info(
                "Backend control state: "
                f"controller={state[0]} lease={state[1]} armed={list(state[2])}"
            )

    def _ensure_vib_pub(self, glove_id: int, source_topic: str) -> None:
        if glove_id in self.vib_publishers:
            return
        match = re.fullmatch(r"/manus_glove_(\d+)", source_topic)
        if match is None:
            raise ValueError(f"invalid MANUS glove topic: {source_topic!r}")
        index = int(match.group(1))
        topic = f"/manus_glove_{index}/vibration_cmd"
        self.vib_publishers[glove_id] = self.create_publisher(
            ManusVibrationCommand, topic, 10
        )
        self.get_logger().info(f"Vibration glove_id=0x{glove_id:X} -> {topic}")

    def _on_tactile(self, msg: dict[str, Any]) -> None:
        side = str(msg.get("side", "")).lower()
        if side not in ("left", "right"):
            return
        powers = msg.get("haptic_powers")
        if not isinstance(powers, list) or len(powers) < 5:
            powers = finger_forces_to_manus_vibration(
                peaks_from_tactile(msg, side),
                force_full_scale_n=self.haptic_scale_n,
            )
        safe = []
        for power in powers[:5]:
            value = float(power)
            safe.append(max(0.0, min(1.0, value)) if math.isfinite(value) else 0.0)
        with self._vib_lock:
            self._pending_vib[side] = safe

    def _haptic_tick(self) -> None:
        with self._vib_lock:
            pending = self._pending_vib
            self._pending_vib = {}
        for side, powers in pending.items():
            glove_id = next(
                (glove_id for glove_id, mapped_side in self.glove_side.items() if mapped_side == side),
                None,
            )
            if glove_id is None or glove_id not in self.vib_publishers:
                continue
            msg = ManusVibrationCommand()
            msg.intensities = (powers + [0.0] * 5)[:5]
            self.vib_publishers[glove_id].publish(msg)

    def shutdown(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        self._new_frame.set()
        self._worker.join(timeout=3.0)
        if rclpy.ok():
            with self._vib_lock:
                for side in set(self.glove_side.values()):
                    self._pending_vib[side] = [0.0] * 5
            try:
                self._haptic_tick()
            except Exception as exc:  # noqa: BLE001 - TCP disable must still run
                if rclpy.ok():
                    self.get_logger().error(f"Failed to clear MANUS vibration: {exc}")
        try:
            self.wuji.set_enable(False)
        finally:
            self.wuji.close()


def _default_backend_host() -> str:
    return os.environ.get(
        "WUJI_BACKEND_HOST",
        os.environ.get("ROBOT_HOST", "127.0.0.1"),
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="MANUS raw nodes -> selectable retargeter -> local Hand2/MuJoCo backend"
    )
    parser.add_argument("--host", default=_default_backend_host())
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("WUJI_BACKEND_PORT", os.environ.get("ROBOT_PORT", "9500"))),
    )
    parser.add_argument("--left-config", type=Path, default=config_path_for_side("left"))
    parser.add_argument("--right-config", type=Path, default=config_path_for_side("right"))
    add_retarget_backend_argument(parser)
    parser.add_argument("--haptic-scale-n", type=float, default=2.0)
    parser.add_argument("--gil-switch-ms", type=float, default=0.2)
    parser.add_argument(
        "--no-auto-enable",
        action="store_true",
        help="Connect without enabling the backend hands",
    )
    args = parser.parse_args()
    sys.setswitchinterval(max(1e-5, args.gil_switch_ms / 1000.0))

    rclpy.init()
    node = ManusWujiBridge(
        backend_host=args.host,
        backend_port=args.port,
        retarget_backend=args.retarget_backend,
        left_config=args.left_config,
        right_config=args.right_config,
        haptic_scale_n=args.haptic_scale_n,
        auto_enable=not args.no_auto_enable,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # ROS Humble may invalidate the context before spin returns on SIGINT,
        # raising RCLError rather than KeyboardInterrupt. Unexpected runtime
        # errors while the context is still healthy must remain visible.
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.get_logger().info("Shutting down...")
        try:
            node.shutdown()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
