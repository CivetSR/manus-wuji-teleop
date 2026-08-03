"""Wire protocol: newline-delimited JSON (JSONL) over TCP.

All angles are radians unless noted. Finger order is always:
  0=thumb, 1=index, 2=middle, 3=ring, 4=pinky

Wuji Hand 2 joint layout (length 20, finger-major):
  joints[finger * 4 + j] for finger in 0..4, j in 0..3 (J1..J4)
"""

from __future__ import annotations

import json
import math
from typing import Any, Dict, List, Optional

PROTOCOL_VERSION = 1
DEFAULT_TCP_PORT = 9500
NUM_JOINTS = 20
NUM_FINGERS = 5
JOINTS_PER_FINGER = 4

FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")


def dumps(msg: Dict[str, Any]) -> bytes:
    return (
        json.dumps(
            msg,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")


def loads_line(line: bytes | str) -> Dict[str, Any]:
    if isinstance(line, bytes):
        line = line.decode("utf-8")

    def reject_nonfinite(token: str) -> None:
        raise ValueError(f"non-standard JSON numeric token is forbidden: {token}")

    value = json.loads(line, parse_constant=reject_nonfinite)
    if not isinstance(value, dict):
        raise ValueError("each JSONL message must be an object")
    return value


def normalize_positions(raw: Optional[List[float]]) -> List[float]:
    if not isinstance(raw, list) or len(raw) != NUM_JOINTS:
        length = len(raw) if isinstance(raw, list) else None
        raise ValueError(
            f"position must be a JSON list of exactly {NUM_JOINTS} values, got {length}"
        )
    if any(type(value) not in (int, float) for value in raw):
        raise ValueError("position values must be JSON numbers (booleans/strings are forbidden)")
    pos = [float(value) for value in raw]
    if not all(math.isfinite(value) for value in pos):
        raise ValueError("position contains NaN or infinity")
    return pos
