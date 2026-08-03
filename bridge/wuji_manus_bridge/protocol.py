"""Wire protocol: newline-delimited JSON (JSONL) over TCP.

All angles are radians unless noted. Finger order is always:
  0=thumb, 1=index, 2=middle, 3=ring, 4=pinky

Wuji Hand 2 joint layout (length 20, finger-major):
  joints[finger * 4 + j] for finger in 0..4, j in 0..3 (J1..J4)
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional

PROTOCOL_VERSION = 1
DEFAULT_TCP_PORT = 9500
NUM_JOINTS = 20
NUM_FINGERS = 5
JOINTS_PER_FINGER = 4

FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")


def dumps(msg: Dict[str, Any]) -> bytes:
    return (json.dumps(msg, separators=(",", ":"), ensure_ascii=False) + "\n").encode(
        "utf-8"
    )


def loads_line(line: bytes | str) -> Dict[str, Any]:
    if isinstance(line, bytes):
        line = line.decode("utf-8")
    return json.loads(line)


@dataclass
class Hello:
    type: str = "hello"
    protocol_version: int = PROTOCOL_VERSION
    client: str = "manus_x86"
    features: List[str] = field(default_factory=lambda: ["joint_cmd", "tactile", "haptic"])


@dataclass
class HelloAck:
    type: str = "hello_ack"
    protocol_version: int = PROTOCOL_VERSION
    server: str = "wuji_manus_bridge"
    hands: Dict[str, Any] = field(default_factory=dict)
    control_hz: float = 100.0
    cutoff_hz: float = 5.0
    max_joint_speed_rad_s: float = 2.0


@dataclass
class JointCmd:
    """Manus -> Wuji: target joint positions for one hand."""

    type: str = "joint_cmd"
    side: str = "left"  # left | right
    seq: int = 0
    t_ms: int = 0
    position: List[float] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    velocity: Optional[List[float]] = None
    effort: Optional[List[float]] = None
    # If false, command is ignored (safety). Default True once enabled.
    enable: bool = True


@dataclass
class EnableCmd:
    type: str = "enable"
    side: str = "both"  # left | right | both
    enabled: bool = True


@dataclass
class JointStateMsg:
    type: str = "joint_state"
    side: str = "left"
    seq: int = 0
    t_ms: int = 0
    position: List[float] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    velocity: List[float] = field(default_factory=list)
    effort: List[float] = field(default_factory=list)


@dataclass
class TactileFinger:
    """Per-finger tactile summary for haptic mapping on Manus side."""

    peak_n: float = 0.0
    mean_n: float = 0.0
    agg_fx: float = 0.0
    agg_fy: float = 0.0
    agg_fz: float = 0.0
    temp_c: float = 0.0
    active_points: int = 0
    # Optional normalized 0..1 intensity hint (server-side). Manus may remapping.
    haptic_01: float = 0.0


@dataclass
class TactileMsg:
    """Wuji -> Manus: fingertip pressure for vibration feedback."""

    type: str = "tactile"
    side: str = "left"
    seq: int = 0
    t_ms: int = 0
    # length 5: thumb..pinky
    fingers: List[Dict[str, float]] = field(default_factory=list)
    # Convenience array aligned with Manus CoreSdk_VibrateFingersForGlove:
    # float powers[5] = {thumb, index, middle, ring, pinky}, each in [0, 1]
    haptic_powers: List[float] = field(default_factory=lambda: [0.0] * NUM_FINGERS)


@dataclass
class StatusMsg:
    type: str = "status"
    ok: bool = True
    message: str = ""
    hands: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ErrorMsg:
    type: str = "error"
    code: str = ""
    message: str = ""


def normalize_positions(raw: Optional[List[float]]) -> List[float]:
    pos = list(raw or [])[:NUM_JOINTS]
    if len(pos) < NUM_JOINTS:
        pos.extend([0.0] * (NUM_JOINTS - len(pos)))
    return [float(x) for x in pos]


def msg_to_dict(obj: Any) -> Dict[str, Any]:
    if hasattr(obj, "__dataclass_fields__"):
        return asdict(obj)
    if isinstance(obj, dict):
        return obj
    raise TypeError(type(obj))
