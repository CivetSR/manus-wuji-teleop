from __future__ import annotations

import numpy as np
import pytest

from joint_order import hand2_protocol_joint_names, strict_joint_name_permutation


@pytest.mark.parametrize("side", ["left", "right"])
def test_official_hand2_qpos_permutation(side: str) -> None:
    protocol = hand2_protocol_joint_names(side)
    official_pinocchio_order = (
        protocol[4:8]
        + protocol[8:12]
        + protocol[16:20]
        + protocol[12:16]
        + protocol[0:4]
    )
    permutation = strict_joint_name_permutation(official_pinocchio_order, protocol)
    np.testing.assert_array_equal(
        permutation,
        [16, 17, 18, 19, 0, 1, 2, 3, 4, 5, 6, 7, 12, 13, 14, 15, 8, 9, 10, 11],
    )


def test_verified_identity_is_allowed() -> None:
    names = ("a", "b", "c")
    np.testing.assert_array_equal(strict_joint_name_permutation(names, names), [0, 1, 2])


def test_name_mismatch_never_falls_back_to_identity() -> None:
    with pytest.raises(ValueError, match="missing_from_source"):
        strict_joint_name_permutation(("a", "b"), ("a", "c"))


def test_duplicate_names_are_rejected() -> None:
    with pytest.raises(ValueError, match="duplicates"):
        strict_joint_name_permutation(("a", "a"), ("a", "b"))
