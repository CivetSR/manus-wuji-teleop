from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

import pytest

import wuji_manus_bridge.hand_worker as hand_worker


@dataclass
class FakeDevice:
    sn: str
    address: str


class FakeValue:
    def __init__(self, value: str) -> None:
        self.value = value

    def get(self) -> str:
        return self.value


class FakeHand:
    def __init__(self, side: str) -> None:
        self.side = side
        self.disconnected = False

    def handedness(self) -> FakeValue:
        return FakeValue(self.side)

    def disconnect(self) -> None:
        self.disconnected = True


class FakeManager:
    def __init__(self) -> None:
        self.devices = [
            FakeDevice("WG-GLOVE", "glove:1234"),
            FakeDevice("WH-LEFT", "192.168.1.10:5000"),
            FakeDevice("WH-RIGHT", "192.168.1.11:5000"),
        ]
        self.sides = {
            "192.168.1.10:5000": "left",
            "192.168.1.11:5000": "right",
        }

    def scan(self) -> list[FakeDevice]:
        return self.devices

    def connect(self, *, address: str, device_name: str) -> FakeHand:
        assert device_name.startswith("wuji_hand_2_probe_")
        return FakeHand(self.sides[address])


def test_network_discovery_filters_gloves_and_uses_reported_handedness(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manager = FakeManager()
    fake_sdk = SimpleNamespace(
        SdkManager=SimpleNamespace(instance=lambda: manager),
    )
    monkeypatch.setattr(hand_worker, "wuji_sdk", fake_sdk)

    discovered = hand_worker.discover_hand2_devices(["left", "right"])
    assert discovered["left"].address == "192.168.1.10:5000"
    assert discovered["right"].address == "192.168.1.11:5000"
    assert discovered["left"].serial_number == "WH-LEFT"


def test_discovery_fails_when_requested_handedness_is_missing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manager = FakeManager()
    manager.devices = [FakeDevice("WH-LEFT", "192.168.1.10:5000")]
    fake_sdk = SimpleNamespace(
        SdkManager=SimpleNamespace(instance=lambda: manager),
    )
    monkeypatch.setattr(hand_worker, "wuji_sdk", fake_sdk)
    with pytest.raises(RuntimeError, match="missing network Hand2 sides"):
        hand_worker.discover_hand2_devices(["right"])
