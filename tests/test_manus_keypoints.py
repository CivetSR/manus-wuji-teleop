from __future__ import annotations

import numpy as np
import pytest

from manus_keypoints import (
    MEDIAPIPE_NODE_KEYS,
    MEDIAPIPE_TO_MANUS,
    InvalidManusFrame,
    raw_nodes_to_mediapipe,
)
from synthetic_hand import synthetic_mediapipe_hand


def make_node(
    node_id: int,
    xyz: tuple[float, float, float] | None = None,
    *,
    key: tuple[str, str] | None = None,
) -> dict:
    if key is None:
        key = dict(zip(MEDIAPIPE_TO_MANUS, MEDIAPIPE_NODE_KEYS))[node_id]
    if xyz is None:
        xyz = (node_id * 0.001, node_id * 0.002, node_id * 0.003)
    return {
        "node_id": node_id,
        "chain_type": key[0].capitalize(),
        "joint_type": key[1].upper(),
        "pose": {
            "position": {
                "x": xyz[0],
                "y": xyz[1],
                "z": xyz[2],
            }
        },
    }


def valid_nodes() -> list[dict]:
    expected = synthetic_mediapipe_hand(0.35, "left")
    nodes = [
        make_node(
            node_id,
            (
                float(expected[index, 0]),
                float(-expected[index, 1]),
                float(expected[index, 2]),
            ),
            key=key,
        )
        for index, (node_id, key) in enumerate(
            zip(MEDIAPIPE_TO_MANUS, MEDIAPIPE_NODE_KEYS)
        )
    ]
    # MANUS 3.1.1 also emits one metacarpal anchor for every non-thumb
    # finger. These are intentionally not part of MediaPipe-21.
    for node_id, finger, point_index in (
        (5, "index", 5),
        (10, "middle", 9),
        (15, "ring", 13),
        (20, "pinky", 17),
    ):
        anchor = expected[point_index] * 0.5
        nodes.append(
            make_node(
                node_id,
                (float(anchor[0]), float(-anchor[1]), float(anchor[2])),
                key=(finger, "mcp"),
            )
        )
    return list(reversed(nodes))


def test_raw_nodes_follow_verified_mapping_and_flip_y() -> None:
    keypoints = raw_nodes_to_mediapipe(valid_nodes())
    assert keypoints.shape == (21, 3)
    np.testing.assert_allclose(keypoints, synthetic_mediapipe_hand(0.35, "left"))


def test_bundled_manus_31_schema_uses_ids_zero_through_twenty_four() -> None:
    nodes = valid_nodes()
    assert sorted(node["node_id"] for node in nodes) == list(range(25))
    assert MEDIAPIPE_TO_MANUS == (
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


def test_semantic_mapping_accepts_legacy_numeric_ids() -> None:
    legacy_ids = (
        1,
        22,
        23,
        24,
        25,
        3,
        4,
        5,
        6,
        8,
        9,
        10,
        11,
        13,
        14,
        15,
        16,
        18,
        19,
        20,
        21,
    )
    expected = synthetic_mediapipe_hand(0.35, "left")
    nodes = [
        make_node(
            node_id,
            (
                float(expected[index, 0]),
                float(-expected[index, 1]),
                float(expected[index, 2]),
            ),
            key=key,
        )
        for index, (node_id, key) in enumerate(zip(legacy_ids, MEDIAPIPE_NODE_KEYS))
    ]
    np.testing.assert_allclose(raw_nodes_to_mediapipe(nodes), expected)


def test_world_space_wrist_is_normalized_and_zero_wrist_is_valid() -> None:
    nodes = valid_nodes()
    translation = np.asarray([1.5, -0.75, 2.0])
    for node in nodes:
        position = node["pose"]["position"]
        position["x"] += translation[0]
        position["y"] += translation[1]
        position["z"] += translation[2]

    keypoints = raw_nodes_to_mediapipe(nodes)
    np.testing.assert_allclose(keypoints[0], np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(keypoints, synthetic_mediapipe_hand(0.35, "left"))


def test_missing_required_node_rejects_entire_frame() -> None:
    nodes = valid_nodes()
    nodes = [node for node in nodes if node["node_id"] != 24]
    with pytest.raises(InvalidManusFrame, match="pinky.*tip"):
        raw_nodes_to_mediapipe(nodes)


def test_nonfinite_position_rejects_entire_frame() -> None:
    nodes = valid_nodes()
    next(node for node in nodes if node["node_id"] == 24)["pose"]["position"]["y"] = np.nan
    with pytest.raises(InvalidManusFrame, match="NaN"):
        raw_nodes_to_mediapipe(nodes)


def test_duplicate_required_node_is_rejected() -> None:
    nodes = valid_nodes()
    nodes.append(make_node(MEDIAPIPE_TO_MANUS[0]))
    with pytest.raises(InvalidManusFrame, match="duplicate"):
        raw_nodes_to_mediapipe(nodes)


def test_collapsed_bone_rejects_entire_frame() -> None:
    nodes = valid_nodes()
    by_id = {node["node_id"]: node for node in nodes}
    by_id[23]["pose"]["position"] = dict(by_id[22]["pose"]["position"])
    with pytest.raises(InvalidManusFrame, match="collapsed bones"):
        raw_nodes_to_mediapipe(nodes)


def test_collinear_palm_rejects_entire_frame() -> None:
    points = synthetic_mediapipe_hand(0.35, "left")
    points[5] = [0.01, 0.02, 0.0]
    points[17] = [0.02, 0.04, 0.0]
    nodes = [
        make_node(
            node_id,
            (float(points[index, 0]), float(-points[index, 1]), float(points[index, 2])),
        )
        for index, node_id in enumerate(MEDIAPIPE_TO_MANUS)
    ]
    with pytest.raises(InvalidManusFrame, match="palm area"):
        raw_nodes_to_mediapipe(nodes)
