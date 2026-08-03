from __future__ import annotations

import pytest

from wuji_manus_bridge.smoother import JointSmoother


@pytest.mark.parametrize(
    "kwargs",
    [
        {"control_hz": float("nan")},
        {"control_hz": 0.0},
        {"cutoff_hz": float("inf")},
        {"max_joint_speed_rad_s": -1.0},
    ],
)
def test_smoother_rejects_unsafe_numeric_configuration(kwargs: dict) -> None:
    with pytest.raises(ValueError):
        JointSmoother(**kwargs)


@pytest.mark.parametrize(
    "bad",
    [
        [0.0] * 19,
        [0.0] * 21,
        [0.0] * 19 + [float("nan")],
        [0.0] * 19 + [True],
    ],
)
def test_smoother_never_pads_truncates_or_coerces_targets(bad: list) -> None:
    smoother = JointSmoother()
    with pytest.raises(ValueError):
        smoother.set_target(bad)
