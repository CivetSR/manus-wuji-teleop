"""Pure helpers for two-point Manus -> Wuji joint calibration."""

from __future__ import annotations

import statistics
from typing import Iterable

from joint_map import (
    DEG2RAD,
    DEFAULT_JOINT_SRC,
    FINGER_CAP,
    WUJI_JOINTS,
    ergonomics_key,
)

JOINT_LIMITS_DEG = {
    "Thumb": {
        "J1": (-68.0, 74.0),
        "J2": (-85.0, 40.0),
        "J3": (-60.0, 90.0),
        "J4": (-60.0, 90.0),
    },
    "Finger": {
        "J1": (-60.0, 90.0),
        "J2": (-40.0, 40.0),
        "J3": (-60.0, 120.0),
        "J4": (-60.0, 90.0),
    },
}


def median_pose(samples: Iterable[dict[str, float]]) -> dict[str, float]:
    """Return a pose using the median of each sampled ergonomics channel."""
    samples = list(samples)
    keys = {key for sample in samples for key in sample}
    return {
        key: statistics.median(sample[key] for sample in samples if key in sample)
        for key in keys
    }


def build_side_config(
    open_pose: dict[str, float],
    fist_pose: dict[str, float],
    together_pose: dict[str, float],
    spread_pose: dict[str, float],
    *,
    min_input_delta: float = 2.0,
) -> tuple[dict[str, object], list[str]]:
    """Build angle-preserving mapping; calibration only determines sign and zero."""
    config: dict[str, object] = {}
    errors: list[str] = []

    for finger in FINGER_CAP:
        finger_config: dict[str, object] = {}
        for joint in WUJI_JOINTS:
            src = DEFAULT_JOINT_SRC[joint]
            key = ergonomics_key(finger, src)
            is_spread = src == "MCPSpread"
            start_pose = together_pose if is_spread else open_pose
            end_pose = spread_pose if is_spread else fist_pose

            if key not in start_pose or key not in end_pose:
                errors.append(f"{key}: missing from Manus data")
                continue

            start = float(start_pose[key])
            end = float(end_pose[key])
            delta = end - start
            if abs(delta) < min_input_delta:
                errors.append(
                    f"{key}: input moved only {delta:.3f}; "
                    f"expected at least {min_input_delta:.3f}"
                )
                continue

            # Preserve angular magnitude exactly: one Manus degree -> one Wuji degree.
            scale = DEG2RAD if delta > 0.0 else -DEG2RAD
            zero_input = (start + end) * 0.5 if is_spread else start
            bias = -zero_input * scale
            limits = JOINT_LIMITS_DEG["Thumb" if finger == "Thumb" else "Finger"][joint]
            finger_config[joint] = {
                "src": src,
                "scale": round(scale, 9),
                "bias": round(bias, 9),
                "min": round(limits[0] * DEG2RAD, 9),
                "max": round(limits[1] * DEG2RAD, 9),
            }
        config[finger] = finger_config

    return config, errors
