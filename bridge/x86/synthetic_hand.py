"""Deterministic, glove-free MediaPipe hand trajectories for tests and demos."""

from __future__ import annotations

import math

import numpy as np


def synthetic_mediapipe_hand(curl: float, side: str = "left") -> np.ndarray:
    """Return a plausible 21-point hand in metres with controllable finger curl."""

    if side not in ("left", "right"):
        raise ValueError(f"side must be left or right, got {side!r}")
    curl = max(0.0, min(1.0, float(curl)))
    mirror = 1.0 if side == "left" else -1.0
    points = np.zeros((21, 3), dtype=np.float64)

    # Thumb CMC -> MCP -> IP -> tip.
    thumb = np.array([0.018 * mirror, 0.018, 0.0], dtype=np.float64)
    points[1] = thumb
    for index, length in enumerate((0.025, 0.022, 0.019), start=2):
        angle = curl * (0.35 + 0.35 * (index - 1))
        thumb = thumb + np.array(
            [0.55 * mirror * length, length * math.cos(angle), -length * math.sin(angle)]
        )
        points[index] = thumb

    finger_specs = (
        (5, 0.030, 0.040, (0.033, 0.024, 0.020)),
        (9, 0.010, 0.044, (0.036, 0.026, 0.021)),
        (13, -0.012, 0.041, (0.034, 0.024, 0.020)),
        (17, -0.030, 0.035, (0.029, 0.021, 0.018)),
    )
    for start, x, base_y, lengths in finger_specs:
        current = np.array([x * mirror, base_y, 0.0], dtype=np.float64)
        points[start] = current
        for segment, length in enumerate(lengths, start=1):
            angle = curl * (0.35 + 0.55 * segment)
            current = current + np.array(
                [0.0, length * math.cos(angle), -length * math.sin(angle)]
            )
            points[start + segment] = current
    return points
