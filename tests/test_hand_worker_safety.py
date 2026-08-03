from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from typing import Any

import pytest

import wuji_manus_bridge.hand_worker as hand_worker


class FakeValue:
    def __init__(self, value: Any) -> None:
        self.value = value

    def get(self) -> Any:
        return self.value


class FakeSubscription:
    def close(self) -> None:
        pass


class FakePublisher:
    def __init__(self) -> None:
        self.created_thread = threading.get_ident()
        self.send_threads: list[int] = []
        self.close_thread: int | None = None
        self.sent_count = 0

    def send(self, _commands: list[Any]) -> None:
        self.send_threads.append(threading.get_ident())
        self.sent_count += 1

    def close(self) -> None:
        self.close_thread = threading.get_ident()


class FakeJointCommandResource:
    def __init__(self, hand: "FakeHand") -> None:
        self.hand = hand

    def publish(self) -> FakePublisher:
        self.hand.publisher = FakePublisher()
        return self.hand.publisher


class FakeJointStatesResource:
    def subscribe_with_callback(self, callback: Any) -> FakeSubscription:
        joints = [
            SimpleNamespace(nid=index, position=0.01 * index, velocity=0.0, effort=0.0)
            for index in range(20)
        ]
        callback(SimpleNamespace(joints=joints))
        return FakeSubscription()


class FakeHand:
    def __init__(self) -> None:
        self.is_connected = True
        self.serial_number = "WH-FAKE"
        self.publisher: FakePublisher | None = None
        self.enable_threads: list[int] = []
        self.disable_threads: list[int] = []
        self.disconnected = False
        self.fail_enable = False

    def handedness(self) -> FakeValue:
        return FakeValue("left")

    def online_joints_count(self) -> FakeValue:
        return FakeValue(20)

    def joint_command(self) -> FakeJointCommandResource:
        return FakeJointCommandResource(self)

    def joint_states(self) -> FakeJointStatesResource:
        return FakeJointStatesResource()

    def enable(self) -> None:
        self.enable_threads.append(threading.get_ident())
        if self.fail_enable:
            raise RuntimeError("enable acknowledgement failed")

    def disable(self) -> None:
        self.disable_threads.append(threading.get_ident())

    def disconnect(self) -> None:
        self.disconnected = True


class FakeManager:
    def __init__(self, hand: FakeHand) -> None:
        self.hand = hand

    def connect(self, **_kwargs: Any) -> FakeHand:
        return self.hand


def wait_until(predicate: Any, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise TimeoutError("condition was not met")
        time.sleep(0.005)


def test_real_worker_requires_arm_and_owns_publisher_on_one_thread(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake_hand = FakeHand()
    manager = FakeManager(fake_hand)
    fake_sdk = SimpleNamespace(
        SdkManager=SimpleNamespace(instance=lambda: manager),
        ConnectOptions=type("ConnectOptions", (), {}),
    )
    monkeypatch.setattr(hand_worker, "wuji_sdk", fake_sdk)
    monkeypatch.setattr(
        hand_worker,
        "JointCommand",
        lambda position, velocity, effort: (position, velocity, effort),
    )

    worker = hand_worker.HandWorker("left", "fake:5000")
    worker.connect()
    assert fake_hand.enable_threads == []

    worker.start_loop()
    assert fake_hand.publisher is not None
    publisher = fake_hand.publisher
    assert fake_hand.enable_threads == []

    worker.set_enabled(True)
    assert len(fake_hand.enable_threads) == 1
    worker.set_joint_target([0.2] * 20)
    wait_until(lambda: publisher.sent_count > 0)

    sent_before_disable = publisher.sent_count
    worker.set_enabled(False)
    assert len(fake_hand.disable_threads) == 1
    time.sleep(0.03)
    assert publisher.sent_count == sent_before_disable

    worker.disconnect()
    assert fake_hand.disconnected
    assert publisher.send_threads
    assert publisher.created_thread == publisher.close_thread
    assert set(publisher.send_threads) == {publisher.created_thread}
    assert fake_hand.enable_threads == [publisher.created_thread]
    assert fake_hand.disable_threads == [publisher.created_thread]


def test_real_worker_disables_after_partial_enable_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake_hand = FakeHand()
    fake_hand.fail_enable = True
    manager = FakeManager(fake_hand)
    fake_sdk = SimpleNamespace(
        SdkManager=SimpleNamespace(instance=lambda: manager),
        ConnectOptions=type("ConnectOptions", (), {}),
    )
    monkeypatch.setattr(hand_worker, "wuji_sdk", fake_sdk)

    worker = hand_worker.HandWorker("left", "fake:5000")
    worker.connect()
    worker.start_loop()
    publisher = fake_hand.publisher
    assert publisher is not None

    with pytest.raises(RuntimeError, match="publisher failed"):
        worker.set_enabled(True)
    wait_until(lambda: bool(fake_hand.disable_threads))
    worker.disconnect()

    assert fake_hand.enable_threads == [publisher.created_thread]
    assert fake_hand.disable_threads == [publisher.created_thread]
    assert publisher.close_thread == publisher.created_thread
