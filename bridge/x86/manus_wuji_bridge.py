#!/usr/bin/env python3
"""x86 bridge: Manus gloves (ROS2) <-> Wuji Hand 2 on Jetson (TCP).

Prerequisites:
  1. Jetson: wuji_manus_bridge server listening on :9500
  2. x86: ros2 run manus_ros2 manus_data_publisher   (owns Manus SDK)
  3. This script (does NOT open Manus SDK itself)

Example:
  source /home/omen/srworkspace/manus-hand-viz/scripts/env.sh
  export ROBOT_HOST=6.6.7.100
  python3 manus_wuji_bridge.py --config ../examples/retarget_manus_to_wuji.yaml
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node

from joint_map import (
    ergonomics_list_to_dict,
    finger_forces_to_manus_vibration,
    load_retarget_config,
    manus_ergonomics_to_wuji20,
    peaks_from_tactile,
)
from wuji_client import WujiClient

try:
    from manus_ros2_msgs.msg import ManusGlove, ManusVibrationCommand
except ImportError as exc:
    raise SystemExit(
        "manus_ros2_msgs not found. Run: source /opt/ros/humble/setup.bash && "
        "source ~/ros2_ws/install/setup.bash"
    ) from exc


class ManusWujiBridge(Node):
    def __init__(
        self,
        robot_host: str,
        robot_port: int,
        config: dict[str, Any],
        cmd_hz: float,
        haptic_scale_n: float,
        auto_enable: bool,
    ) -> None:
        super().__init__("manus_wuji_bridge")
        self.config = config
        self.cmd_period = 1.0 / max(cmd_hz, 1.0)
        self.haptic_scale_n = haptic_scale_n
        self.auto_enable = auto_enable

        self.latest_joints: dict[str, list[float]] = {"left": [0.0] * 20, "right": [0.0] * 20}
        self.glove_side: dict[int, str] = {}
        self.vib_publishers: dict[int, Any] = {}
        self._joint_lock = threading.Lock()
        self._vib_lock = threading.Lock()
        self._pending_vib: dict[str, list[float]] = {}
        self._glove_msg_count = 0
        self._last_glove_wall_time = 0.0
        self._subscribed_topics: set[str] = set()

        # Must match manus_data_publisher (default reliable QoS). BEST_EFFORT drops all msgs.
        self._glove_qos = 10
        self._discover_glove_topics()
        self.create_timer(2.0, self._discover_glove_topics)
        self.create_timer(3.0, self._status_tick)

        self.wuji = WujiClient(robot_host, robot_port)
        self.wuji.connect(on_tactile=self._on_tactile)
        self.get_logger().info(
            f"Connected to Wuji server {robot_host}:{robot_port}, hands={self.wuji.hello_ack.get('hands', {})}"
        )

        if self.auto_enable:
            self.wuji.set_enable(True, side="both")
            self.get_logger().info("Sent enable:true for both hands")

        self.create_timer(self.cmd_period, self._cmd_tick)
        self.create_timer(0.02, self._haptic_tick)

    def _side_cfg(self, side: str) -> dict[str, Any]:
        return self.config.get(side, self.config.get(side.lower(), {}))

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
            self.create_subscription(
                ManusGlove,
                topic,
                self._on_glove_msg,
                self._glove_qos,
            )
            self._subscribed_topics.add(topic)
            self.get_logger().info(f"Subscribed to {topic}")

    def _on_glove_msg(self, msg: ManusGlove) -> None:
        side = (msg.side or "").lower()
        if side not in ("left", "right"):
            side = self.glove_side.get(msg.glove_id, "left")

        self.glove_side[msg.glove_id] = side
        self._ensure_vib_pub(msg.glove_id)

        ergo = ergonomics_list_to_dict(msg.ergonomics)
        joints = manus_ergonomics_to_wuji20(ergo, self._side_cfg(side))
        with self._joint_lock:
            self.latest_joints[side] = joints
        self._glove_msg_count += 1
        self._last_glove_wall_time = time.time()

    def _status_tick(self) -> None:
        with self._joint_lock:
            joints = dict(self.latest_joints)
        now = time.time()
        if self._glove_msg_count == 0 or now - self._last_glove_wall_time > 2.0:
            self.get_logger().warning(
                "No ManusGlove data received — check manus_data_publisher and glove pairing"
            )
            return
        for side, pos in joints.items():
            peak = max((abs(v) for v in pos), default=0.0)
            if peak > 0.01:
                self.get_logger().info(
                    f"Teleop active: {side} peak={peak:.3f} rad, "
                    f"glove_msgs={self._glove_msg_count}, wuji_enabled={self.wuji.enabled}"
                )
                return
        self.get_logger().warning(
            f"Receiving glove data ({self._glove_msg_count} msgs) but all joint cmds ~0 — check mapping"
        )

    def _ensure_vib_pub(self, glove_id: int) -> None:
        if glove_id in self.vib_publishers:
            return
        idx = len(self.vib_publishers)
        topic = f"/manus_glove_{idx}/vibration_cmd"
        pub = self.create_publisher(ManusVibrationCommand, topic, 10)
        self.vib_publishers[glove_id] = pub
        self.get_logger().info(f"Vibration publisher glove_id=0x{glove_id:X} -> {topic}")

    def _on_tactile(self, msg: dict[str, Any]) -> None:
        side = str(msg.get("side", "")).lower()
        if side not in ("left", "right"):
            return

        powers = msg.get("haptic_powers")
        if not isinstance(powers, list) or len(powers) < 5:
            peaks = peaks_from_tactile(msg, side)
            powers = finger_forces_to_manus_vibration(
                peaks, force_full_scale_n=self.haptic_scale_n
            )

        with self._vib_lock:
            self._pending_vib[side] = [float(max(0.0, min(1.0, p))) for p in powers[:5]]

    def _cmd_tick(self) -> None:
        with self._joint_lock:
            joints = dict(self.latest_joints)
        for side, pos in joints.items():
            self.wuji.send_joint_cmd(side, pos, enable=self.wuji.enabled)

    def _haptic_tick(self) -> None:
        with self._vib_lock:
            pending = dict(self._pending_vib)
            self._pending_vib.clear()

        for side, powers in pending.items():
            glove_id = next(
                (gid for gid, s in self.glove_side.items() if s == side),
                None,
            )
            if glove_id is None or glove_id not in self.vib_publishers:
                continue
            msg = ManusVibrationCommand()
            msg.intensities = [float(p) for p in powers] + [0.0] * max(0, 5 - len(powers))
            msg.intensities = msg.intensities[:5]
            self.vib_publishers[glove_id].publish(msg)

    def shutdown(self) -> None:
        with self._vib_lock:
            for side in list(self.glove_side.values()):
                self._pending_vib[side] = [0.0] * 5
        self._haptic_tick()
        self.wuji.set_enable(False)
        self.wuji.close()


def main() -> int:
    ap = argparse.ArgumentParser(description="Manus x86 -> Wuji teleop bridge")
    ap.add_argument("--host", default=os.environ.get("ROBOT_HOST", "6.6.8.100"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("ROBOT_PORT", "9500")))
    ap.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "examples" / "retarget_manus_to_wuji.yaml",
    )
    ap.add_argument("--cmd-hz", type=float, default=60.0)
    ap.add_argument("--haptic-scale-n", type=float, default=2.0)
    ap.add_argument(
        "--no-auto-enable",
        action="store_true",
        help="Do not send enable:true on connect (hands stay idle until manual enable)",
    )
    args = ap.parse_args()

    config = load_retarget_config(args.config)

    rclpy.init()
    node = ManusWujiBridge(
        robot_host=args.host,
        robot_port=args.port,
        config=config,
        cmd_hz=args.cmd_hz,
        haptic_scale_n=args.haptic_scale_n,
        auto_enable=not args.no_auto_enable,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
