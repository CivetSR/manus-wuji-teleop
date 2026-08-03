"""Manus ergonomics -> Wuji Hand 2 joint mapping."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

NUM_JOINTS = 20
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
FINGER_CAP = ("Thumb", "Index", "Middle", "Ring", "Pinky")
DEG2RAD = math.pi / 180.0

# Manus ROS ergonomics type suffixes map to Wuji J1..J4 within each finger.
WUJI_JOINTS = ("J1", "J2", "J3", "J4")
DEFAULT_JOINT_SRC = {
    "J1": "MCPStretch",
    "J2": "MCPSpread",
    "J3": "PIPStretch",
    "J4": "DIPStretch",
}


def ergonomics_key(finger: str, src: str) -> str:
    """Build Manus ergonomics type name (matches manus_data_publisher strings)."""
    if src == "MCPSpread":
        if finger == "Thumb":
            return "ThumbMCPSpread"
        return f"{finger}Spread"
    return f"{finger}{src}"


def load_retarget_config(path: Path | None) -> dict[str, Any]:
    if path is None or not path.is_file():
        return {}
    try:
        import yaml  # type: ignore
    except ImportError as exc:
        raise RuntimeError("PyYAML required: pip install pyyaml") from exc
    with path.open(encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    return data


def ergonomics_list_to_dict(ergonomics: list[Any]) -> dict[str, float]:
    out: dict[str, float] = {}
    for item in ergonomics:
        key = getattr(item, "type", None) or item.get("type")
        val = getattr(item, "value", None) if hasattr(item, "value") else item.get("value")
        if key is not None:
            out[str(key)] = float(val or 0.0)
    return out


def manus_ergonomics_to_wuji20(
    ergo: dict[str, float],
    side_cfg: dict[str, Any] | None = None,
    *,
    clamp_rad: tuple[float, float] = (-1.8, 1.8),
) -> list[float]:
    """Map Manus ergonomics dict (ROS type names) -> 20 Wuji radians."""
    side_cfg = side_cfg or {}
    out = [0.0] * NUM_JOINTS
    for fi, finger in enumerate(FINGER_CAP):
        finger_cfg = side_cfg.get(finger, {})
        base = fi * 4
        for ji, jname in enumerate(WUJI_JOINTS):
            joint_cfg = finger_cfg.get(jname, {})
            src = joint_cfg.get("src", DEFAULT_JOINT_SRC[jname])
            scale = float(joint_cfg.get("scale", DEG2RAD))
            bias = float(joint_cfg.get("bias", 0.0))
            key = ergonomics_key(finger, src)
            val = float(ergo.get(key, 0.0))
            q = val * scale + bias
            joint_min = float(joint_cfg.get("min", clamp_rad[0] if clamp_rad else -math.inf))
            joint_max = float(joint_cfg.get("max", clamp_rad[1] if clamp_rad else math.inf))
            if joint_min > joint_max:
                joint_min, joint_max = joint_max, joint_min
            q = max(joint_min, min(joint_max, q))
            out[base + ji] = q
    return out


def finger_forces_to_manus_vibration(
    peak_n_by_finger: list[float],
    *,
    force_full_scale_n: float = 2.0,
    deadband_n: float = 0.05,
) -> list[float]:
    out: list[float] = []
    for i in range(5):
        f = abs(float(peak_n_by_finger[i])) if i < len(peak_n_by_finger) else 0.0
        if f < deadband_n:
            out.append(0.0)
        else:
            out.append(max(0.0, min(1.0, f / force_full_scale_n)))
    return out


def peaks_from_tactile(msg: dict[str, Any], side: str) -> list[float]:
    """Extract thumb..pinky peak force (N) from server tactile message."""
    fingers = msg.get("fingers")
    if isinstance(fingers, list):
        return [float(f.get("peak_n", 0.0)) for f in fingers[:5]] + [0.0] * max(
            0, 5 - len(fingers)
        )
    hands = msg.get("hands", {})
    side_obj = hands.get(side, {})
    finger_map = side_obj.get("fingers", {})
    if isinstance(finger_map, dict):
        return [float(finger_map.get(name, {}).get("peak_n", 0.0)) for name in FINGERS]
    return [0.0] * 5
