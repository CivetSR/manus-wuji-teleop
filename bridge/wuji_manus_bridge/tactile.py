"""Decode Wuji Hand 2 fingertip tactile binary frames."""

from __future__ import annotations

import struct
from typing import Any, Dict, List

POINT_COUNT = 34
POINT_STRIDE = 6
AGGREGATE_STRIDE = 8

# Default haptic mapping: map |agg_fz| and peak into [0, 1].
# Tunable; Manus side may ignore haptic_01 and recompute from peak_n/agg_fz.
DEFAULT_HAPTIC_SCALE_N = 2.0  # Newtons -> full vibration


def _to_bytes(data: Any) -> bytes:
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    if hasattr(data, "tobytes"):
        return data.tobytes()
    return bytes(data)


def decode_fingertip_frame(data: Any, haptic_scale_n: float = DEFAULT_HAPTIC_SCALE_N) -> Dict[str, float]:
    raw = _to_bytes(data)
    fzs: List[float] = []
    for i in range(POINT_COUNT):
        off = i * POINT_STRIDE
        if off + POINT_STRIDE > len(raw):
            break
        _fx, _fy, fz = struct.unpack_from("<hhh", raw, off)
        fzs.append(fz * 0.01)

    agg = {"fx": 0.0, "fy": 0.0, "fz": 0.0, "temp_c": 0.0}
    agg_off = POINT_COUNT * POINT_STRIDE
    if len(raw) >= agg_off + AGGREGATE_STRIDE:
        fx, fy, fz, temp = struct.unpack_from("<hhhh", raw, agg_off)
        agg = {
            "fx": fx * 0.01,
            "fy": fy * 0.01,
            "fz": fz * 0.01,
            "temp_c": temp * 0.1,
        }

    active = [v for v in fzs if abs(v) > 0.005]
    peak = max((abs(v) for v in fzs), default=0.0)
    mean = (sum(active) / len(active)) if active else 0.0
    scale = max(haptic_scale_n, 1e-6)
    haptic = min(1.0, max(peak, abs(agg["fz"])) / scale)

    return {
        "peak_n": float(peak),
        "mean_n": float(mean),
        "agg_fx": float(agg["fx"]),
        "agg_fy": float(agg["fy"]),
        "agg_fz": float(agg["fz"]),
        "temp_c": float(agg["temp_c"]),
        "active_points": float(len(active)),
        "haptic_01": float(haptic),
    }


EMPTY_FINGER = {
    "peak_n": 0.0,
    "mean_n": 0.0,
    "agg_fx": 0.0,
    "agg_fy": 0.0,
    "agg_fz": 0.0,
    "temp_c": 0.0,
    "active_points": 0.0,
    "haptic_01": 0.0,
}
