"""Fail-fast Wuji Hand 2 joint-name ordering helpers."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np


def hand2_protocol_joint_names(side: str) -> tuple[str, ...]:
    """Return the TCP/device order: thumb..pinky, four joints per finger."""

    side = side.lower()
    if side not in ("left", "right"):
        raise ValueError(f"side must be left or right, got {side!r}")
    prefix = "l_" if side == "left" else "r_"
    names: list[str] = []
    names.extend(
        prefix + name
        for name in ("thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip")
    )
    for finger in ("index_finger", "middle_finger", "ring_finger"):
        names.extend(
            prefix + finger + suffix
            for suffix in ("_mcp_flex", "_mcp_abd", "_pip", "_dip")
        )
    names.extend(
        prefix + name
        for name in ("pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip")
    )
    return tuple(names)


def strict_joint_name_permutation(
    source_joint_names: Sequence[str],
    destination_joint_names: Sequence[str],
) -> np.ndarray:
    """Return indices that reorder source values into destination name order.

    Unlike the permissive helper in the official examples, this function never
    returns ``None`` and never substitutes an identity mapping after a mismatch.
    An identity permutation is returned only when all names were verified.
    """

    source = tuple(source_joint_names)
    destination = tuple(destination_joint_names)
    if not source or not destination:
        raise ValueError("joint-name lists must be non-empty")
    if any(not isinstance(name, str) or not name for name in source + destination):
        raise ValueError("every joint name must be a non-empty string")
    if len(set(source)) != len(source):
        raise ValueError(f"source joint names contain duplicates: {source}")
    if len(set(destination)) != len(destination):
        raise ValueError(f"destination joint names contain duplicates: {destination}")
    if len(source) != len(destination) or set(source) != set(destination):
        missing = sorted(set(destination) - set(source))
        unexpected = sorted(set(source) - set(destination))
        raise ValueError(
            "joint names do not match; "
            f"source_count={len(source)} destination_count={len(destination)} "
            f"missing_from_source={missing} unexpected_in_source={unexpected}"
        )

    source_index = {name: index for index, name in enumerate(source)}
    return np.asarray([source_index[name] for name in destination], dtype=np.int64)


def mujoco_actuator_joint_names(model: object) -> tuple[str, ...]:
    """Return scalar joint names in MuJoCo actuator order."""

    import mujoco

    names: list[str] = []
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id < 0:
            raise ValueError(f"actuator {actuator_id} is not attached to a joint")
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if not name:
            raise ValueError(f"actuator {actuator_id} targets an unnamed joint")
        names.append(name)
    return tuple(names)
