#!/usr/bin/env python3
"""Safely test one Manus -> Wuji joint mapping at a time."""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node

from joint_map import (
    FINGERS,
    ergonomics_list_to_dict,
    load_retarget_config,
    manus_ergonomics_to_wuji20,
)
from wuji_client import WujiClient

try:
    from manus_ros2_msgs.msg import ManusGlove
except ImportError as exc:
    raise SystemExit(
        "manus_ros2_msgs not found. Source ROS2 and ~/ros2_ws/install/setup.bash first."
    ) from exc


class SingleJointTester(Node):
    def __init__(self, side: str) -> None:
        super().__init__("manus_wuji_single_joint_test")
        self.side = side
        self.latest_ergo: dict[str, float] | None = None
        self._lock = threading.Lock()
        self._subscribed: set[str] = set()
        self._discover_topics()
        self.create_timer(1.0, self._discover_topics)

    def _discover_topics(self) -> None:
        for name, types in self.get_topic_names_and_types():
            if (
                name.startswith("/manus_glove_")
                and not name.endswith("/vibration_cmd")
                and "manus_ros2_msgs/msg/ManusGlove" in types
                and name not in self._subscribed
            ):
                self.create_subscription(ManusGlove, name, self._on_glove, 10)
                self._subscribed.add(name)
                self.get_logger().info(f"Subscribed to {name}")

    def _on_glove(self, msg: ManusGlove) -> None:
        if (msg.side or "").lower() != self.side:
            return
        ergo = ergonomics_list_to_dict(msg.ergonomics)
        if ergo:
            with self._lock:
                self.latest_ergo = ergo

    def get_ergo(self) -> dict[str, float] | None:
        with self._lock:
            return dict(self.latest_ergo) if self.latest_ergo else None


class RobotState:
    def __init__(self, side: str) -> None:
        self.side = side
        self.position: list[float] | None = None
        self._lock = threading.Lock()

    def on_message(self, msg: dict[str, Any]) -> None:
        if msg.get("type") != "joint_state" or msg.get("side") != self.side:
            return
        raw = msg.get("position")
        if not isinstance(raw, list) or len(raw) < 20:
            return
        position = [float(value) for value in raw[:20]]
        if all(math.isfinite(value) for value in position):
            with self._lock:
                self.position = position

    def get(self) -> list[float] | None:
        with self._lock:
            return list(self.position) if self.position else None


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="Manus → Wuji 单关节安全测试")
    parser.add_argument("--side", required=True, choices=("left", "right"))
    parser.add_argument("--finger", required=True, choices=FINGERS)
    parser.add_argument("--joint", required=True, choices=("J1", "J2", "J3", "J4"))
    parser.add_argument("--host", default=os.environ.get("ROBOT_HOST", "6.6.8.100"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("ROBOT_PORT", "9500")))
    parser.add_argument(
        "--config",
        type=Path,
        default=root / "examples" / "retarget_manus_to_wuji.calibrated.yaml",
    )
    parser.add_argument("--duration", type=float, default=15.0, help="最长使能时间（秒）")
    parser.add_argument("--cmd-hz", type=float, default=50.0)
    parser.add_argument(
        "--max-initial-delta",
        type=float,
        default=0.12,
        help="使能前命令与实际位置允许的最大差值（rad）",
    )
    return parser.parse_args()


def wait_until(description: str, getter: Any, timeout: float) -> Any:
    print(f"等待{description}", end="", flush=True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = getter()
        if value is not None:
            print(" 就绪")
            return value
        print(".", end="", flush=True)
        time.sleep(0.25)
    print()
    raise RuntimeError(f"等待{description}超时")


def main() -> int:
    args = parse_args()
    if args.duration <= 0 or args.cmd_hz <= 0 or args.max_initial_delta <= 0:
        raise SystemExit("duration、cmd-hz、max-initial-delta 必须大于 0")

    config = load_retarget_config(args.config)
    side_config = config.get(args.side)
    if not isinstance(side_config, dict):
        raise SystemExit(f"配置中没有已标定的 {args.side} 数据：{args.config}")
    joint_config = side_config.get(args.finger.capitalize(), {}).get(args.joint, {})
    if "min" not in joint_config or "max" not in joint_config:
        raise SystemExit(
            f"{args.side}/{args.finger}/{args.joint} 没有标定限位；禁止使用默认配置测试"
        )

    finger_index = FINGERS.index(args.finger)
    joint_number = int(args.joint[1:]) - 1
    test_index = finger_index * 4 + joint_number
    label = f"{args.side}/{args.finger}/{args.joint} (index {test_index})"

    rclpy.init()
    node = SingleJointTester(args.side)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    client: WujiClient | None = None
    enabled = False
    try:
        ergo = wait_until("对应 Manus 手套数据", node.get_ergo, 30.0)

        robot_state = RobotState(args.side)
        client = WujiClient(args.host, args.port)
        client.connect(on_message=robot_state.on_message)
        hand_info = client.hello_ack.get("hands", {}).get(args.side, {})
        if not hand_info.get("connected", False):
            raise RuntimeError(f"Wuji {args.side} 未连接：{hand_info}")
        actual = wait_until("Wuji 关节状态", robot_state.get, 5.0)

        mapped = manus_ergonomics_to_wuji20(ergo, side_config)
        initial_delta = abs(mapped[test_index] - actual[test_index])
        print(f"\n测试关节：{label}")
        print(f"实际位置：{actual[test_index]:+.4f} rad")
        print(f"手套目标：{mapped[test_index]:+.4f} rad")
        print(f"首次位置差：{initial_delta:.4f} rad")
        print("只有这个关节会跟随手套；其余 19 个关节保持当前反馈位置。")

        if initial_delta > args.max_initial_delta:
            raise RuntimeError(
                f"首次位置差超过 {args.max_initial_delta:.3f} rad。"
                "请调整手套姿态，使目标接近灵巧手当前位置后重试"
            )

        confirmation = f"ENABLE {args.side.upper()} {args.finger.upper()} {args.joint}"
        print("\n确认急停可用、人员远离夹点。")
        typed = input(f"输入 {confirmation} 后使能，直接回车取消：").strip()
        if typed != confirmation:
            print("未使能，测试取消")
            return 0

        stop_event = threading.Event()

        def wait_for_enter() -> None:
            try:
                input("\n已使能。移动手套测试；按 Enter 立即停止。\n")
            finally:
                stop_event.set()

        input_thread = threading.Thread(target=wait_for_enter, daemon=True)
        input_thread.start()
        baseline = actual
        client.set_enable(True, side=args.side)
        enabled = True
        deadline = time.monotonic() + args.duration
        period = 1.0 / args.cmd_hz
        next_status = 0.0

        while not stop_event.is_set() and time.monotonic() < deadline:
            loop_start = time.monotonic()
            latest_ergo = node.get_ergo()
            latest_actual = robot_state.get()
            if latest_ergo is None or latest_actual is None:
                raise RuntimeError("测试期间数据中断")

            mapped = manus_ergonomics_to_wuji20(latest_ergo, side_config)
            command = list(baseline)
            command[test_index] = mapped[test_index]
            client.send_joint_cmd(args.side, command, enable=True)

            if loop_start >= next_status:
                print(
                    f"\r{label} 目标={command[test_index]:+.3f} "
                    f"实际={latest_actual[test_index]:+.3f} rad   ",
                    end="",
                    flush=True,
                )
                next_status = loop_start + 0.25
            time.sleep(max(0.0, period - (time.monotonic() - loop_start)))

        print("\n达到停止条件，正在禁用该手...")
        return 0
    except (RuntimeError, OSError, ConnectionError) as exc:
        print(f"\n错误：{exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在停止...")
        return 130
    finally:
        if client is not None:
            if enabled:
                try:
                    client.set_enable(False, side=args.side)
                except Exception:
                    pass
            client.close()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    sys.exit(main())
