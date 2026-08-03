"""Wuji fingertip force to MANUS vibration helpers."""

from __future__ import annotations

import math
from typing import Any

FINGERS = ("thumb", "index", "middle", "ring", "pinky")


def finger_forces_to_manus_vibration(
    peak_n_by_finger: list[float],
    *,
    force_full_scale_n: float = 2.0,
    deadband_n: float = 0.05,
) -> list[float]:
    """Map thumb-to-pinky peak force in newtons to MANUS intensities."""

    scale = max(float(force_full_scale_n), 1e-6)
    out: list[float] = []
    for index in range(5):
        raw = peak_n_by_finger[index] if index < len(peak_n_by_finger) else 0.0
        force = abs(float(raw))
        if not math.isfinite(force) or force < deadband_n:
            out.append(0.0)
        else:
            out.append(max(0.0, min(1.0, force / scale)))
    return out


def peaks_from_tactile(msg: dict[str, Any], side: str) -> list[float]:
    """Extract thumb-to-pinky peak force from either tactile wire shape."""

    fingers = msg.get("fingers")
    if isinstance(fingers, list):
        peaks = [
            float(finger.get("peak_n", 0.0)) if isinstance(finger, dict) else 0.0
            for finger in fingers[:5]
        ]
        return peaks + [0.0] * (5 - len(peaks))

    hands = msg.get("hands", {})
    side_obj = hands.get(side, {}) if isinstance(hands, dict) else {}
    finger_map = side_obj.get("fingers", {}) if isinstance(side_obj, dict) else {}
    if isinstance(finger_map, dict):
        return [
            float(finger_map.get(name, {}).get("peak_n", 0.0))
            if isinstance(finger_map.get(name, {}), dict)
            else 0.0
            for name in FINGERS
        ]
    return [0.0] * 5
