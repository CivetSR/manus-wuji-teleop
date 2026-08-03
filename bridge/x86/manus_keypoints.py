"""Strict MANUS raw-node conversion to MediaPipe hand keypoints."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from typing import Any

import numpy as np

# MediaPipe: wrist, thumb 1..4, index 5..8, middle 9..12, ring 13..16,
# pinky 17..20.  These are the IDs emitted by the bundled MANUS 3.1.1
# publisher.  Conversion below uses the semantic chain/joint fields because
# older MANUS Core releases used a different numeric ID layout.
MEDIAPIPE_TO_MANUS = (
    0,
    1,
    2,
    3,
    4,
    6,
    7,
    8,
    9,
    11,
    12,
    13,
    14,
    16,
    17,
    18,
    19,
    21,
    22,
    23,
    24,
)

MEDIAPIPE_NODE_KEYS = (
    ("hand", "invalid"),
    ("thumb", "mcp"),
    ("thumb", "pip"),
    ("thumb", "dip"),
    ("thumb", "tip"),
    ("index", "pip"),
    ("index", "ip"),
    ("index", "dip"),
    ("index", "tip"),
    ("middle", "pip"),
    ("middle", "ip"),
    ("middle", "dip"),
    ("middle", "tip"),
    ("ring", "pip"),
    ("ring", "ip"),
    ("ring", "dip"),
    ("ring", "tip"),
    ("pinky", "pip"),
    ("pinky", "ip"),
    ("pinky", "dip"),
    ("pinky", "tip"),
)

MEDIAPIPE_BONES = (
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),
    (0, 5),
    (5, 6),
    (6, 7),
    (7, 8),
    (0, 9),
    (9, 10),
    (10, 11),
    (11, 12),
    (0, 13),
    (13, 14),
    (14, 15),
    (15, 16),
    (0, 17),
    (17, 18),
    (18, 19),
    (19, 20),
)
MIN_BONE_LENGTH_M = 1e-5
MIN_PALM_AREA_M2 = 1e-7


class InvalidManusFrame(ValueError):
    """Raised when a MANUS frame cannot safely be sent to IK."""


def validate_mediapipe_keypoints(keypoints: Any) -> np.ndarray:
    """Validate shape, finiteness, bone lengths, and a non-degenerate palm."""

    points = np.asarray(keypoints, dtype=np.float64)
    if points.shape != (21, 3):
        raise InvalidManusFrame(
            f"MediaPipe keypoints must have shape (21, 3), got {points.shape}"
        )
    if not np.isfinite(points).all():
        raise InvalidManusFrame("MediaPipe keypoints contain NaN or infinity")

    collapsed = [
        (start, end)
        for start, end in MEDIAPIPE_BONES
        if np.linalg.norm(points[end] - points[start]) < MIN_BONE_LENGTH_M
    ]
    if collapsed:
        raise InvalidManusFrame(f"degenerate MediaPipe frame has collapsed bones: {collapsed}")

    index_ray = points[5] - points[0]
    pinky_ray = points[17] - points[0]
    palm_area = float(np.linalg.norm(np.cross(index_ray, pinky_ray)))
    if palm_area < MIN_PALM_AREA_M2:
        raise InvalidManusFrame(
            f"degenerate MediaPipe palm area {palm_area:.3e} m^2"
        )
    return points


def _field(value: Any, name: str) -> Any:
    if isinstance(value, Mapping):
        if name not in value:
            raise InvalidManusFrame(f"MANUS node is missing {name!r}")
        return value[name]
    try:
        return getattr(value, name)
    except AttributeError as exc:
        raise InvalidManusFrame(f"MANUS node is missing {name!r}") from exc


def _position_xyz(node: Any) -> np.ndarray:
    pose = _field(node, "pose")
    position = _field(pose, "position")
    xyz = np.asarray(
        [
            _field(position, "x"),
            _field(position, "y"),
            _field(position, "z"),
        ],
        dtype=np.float64,
    )
    if xyz.shape != (3,):
        raise InvalidManusFrame(f"MANUS node position must have shape (3,), got {xyz.shape}")
    if not np.isfinite(xyz).all():
        raise InvalidManusFrame("MANUS node position contains NaN or infinity")
    return xyz


def raw_nodes_to_mediapipe(raw_nodes: Iterable[Any]) -> np.ndarray:
    """Convert MANUS raw nodes to wrist-relative MediaPipe ``(21, 3)`` metres.

    Extra MANUS metacarpal anchors are ignored. Every required semantic
    chain/joint pair must occur exactly once. Numeric IDs are deliberately not
    used for selection because MANUS 3.1.1 emits IDs 0..24 while an older
    production pipeline emitted a different layout. The MANUS Y coordinate is
    negated to preserve the established retargeting frame.
    """

    required = set(MEDIAPIPE_NODE_KEYS)
    positions: dict[tuple[str, str], np.ndarray] = {}
    seen_node_ids: set[int] = set()
    node_ids: list[int] = []

    try:
        iterator = iter(raw_nodes)
    except TypeError as exc:
        raise InvalidManusFrame("raw_nodes must be iterable") from exc

    for node in iterator:
        try:
            node_id = int(_field(node, "node_id"))
        except (TypeError, ValueError) as exc:
            raise InvalidManusFrame("MANUS node_id must be an integer") from exc
        if node_id in seen_node_ids:
            raise InvalidManusFrame(f"duplicate MANUS node_id: {node_id}")
        seen_node_ids.add(node_id)
        node_ids.append(node_id)

        chain = str(_field(node, "chain_type")).strip().lower()
        joint = str(_field(node, "joint_type")).strip().lower()
        key = (chain, joint)
        if key not in required:
            continue
        if key in positions:
            raise InvalidManusFrame(
                f"duplicate required MANUS semantic node: chain={chain!r}, joint={joint!r}"
            )
        xyz = _position_xyz(node)
        xyz[1] = -xyz[1]
        positions[key] = xyz

    missing = [key for key in MEDIAPIPE_NODE_KEYS if key not in positions]
    if missing:
        raise InvalidManusFrame(
            "MANUS frame is missing required semantic nodes: "
            f"{missing}; available node_ids={sorted(node_ids)}"
        )

    keypoints = np.stack([positions[key] for key in MEDIAPIPE_NODE_KEYS])
    keypoints -= keypoints[0]
    return validate_mediapipe_keypoints(keypoints)
