from __future__ import annotations

import argparse

import numpy as np
import pytest

from joint_order import hand2_protocol_joint_names
from synthetic_hand import synthetic_mediapipe_hand
from wuji_retargeting_adapter import (
    DESCRIPTION_ROOT_TOKEN,
    PINNED_DESCRIPTION_COMMIT,
    RETARGETER_INCOMPATIBLE_GENERATIONS,
    RetargeterBackend,
    SdkRetargetBackend,
    add_retarget_backend_argument,
    config_path_for_side,
    create_retarget_backend,
    default_retarget_backend,
    find_wuji_description_root,
    materialize_config,
    validate_backend_name,
)


@pytest.mark.parametrize(
    ("side", "rotation_x", "rotation_z"),
    [("left", 10.0, 15.0), ("right", -10.0, -15.0)],
)
def test_manus_config_uses_pinned_beta1_model(
    side: str, rotation_x: float, rotation_z: float
) -> None:
    root = find_wuji_description_root()
    config_path = config_path_for_side(side)
    raw_text = config_path.read_text(encoding="utf-8")
    assert DESCRIPTION_ROOT_TOKEN in raw_text
    assert "hand2/hand2_beta1/body" in raw_text

    config, model = materialize_config(config_path, side, root)
    assert config["model"]["generation"] == "wuji-description/hand2/hand2_beta1/body"
    assert config["model"]["description_commit"] == PINNED_DESCRIPTION_COMMIT
    assert config["model"]["input"] == "manus_raw_nodes"
    assert "thumb_skip_pip" not in config["retarget"]
    assert config["retarget"]["mediapipe_rotation"] == {
        "x": rotation_x,
        "y": 0.0,
        "z": rotation_z,
    }
    assert config["retarget"]["w_dir"] == 10.0
    assert config["retarget"]["lp_alpha"] == 0.72
    assert model.urdf.parent.parent == model.mjcf.parent.parent
    assert model.root.name == "body"
    assert model.root.parent.name == "hand2_beta1"


def test_backend_parameter_and_environment_default(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    assert validate_backend_name("RETARGETER") == "retargeter"
    assert validate_backend_name("sdk") == "sdk"
    with pytest.raises(ValueError, match="retarget backend"):
        validate_backend_name("unknown")
    monkeypatch.delenv("RETARGET_BACKEND", raising=False)
    assert default_retarget_backend() == "sdk"
    monkeypatch.setenv("RETARGET_BACKEND", "retargeter")
    assert default_retarget_backend() == "retargeter"
    monkeypatch.setenv("RETARGET_BACKEND", "sdk")
    parser = argparse.ArgumentParser()
    add_retarget_backend_argument(parser)
    assert parser.parse_args([]).retarget_backend == "sdk"
    assert parser.parse_args(["--retarget-backend", "retargeter"]).retarget_backend == (
        "retargeter"
    )


def test_left_and_right_backends_have_independent_state() -> None:
    left = create_retarget_backend("sdk", "left")
    right = create_retarget_backend("sdk", "right")
    assert left is not right
    assert left.session is not right.session


@pytest.mark.parametrize("side", ["left", "right"])
def test_retargeter_refuses_the_pinned_beta1_model(side: str) -> None:
    """hand2_beta1 inverts MCP flexion, so the Python IK must not run silently."""

    assert "hand2_beta1" in RETARGETER_INCOMPATIBLE_GENERATIONS
    with pytest.raises(RuntimeError, match="does not support the hand2_beta1"):
        RetargeterBackend(side)
    with pytest.raises(RuntimeError, match="does not support the hand2_beta1"):
        create_retarget_backend("retargeter", side)


@pytest.mark.parametrize("side", ["left", "right"])
def test_firmware_joint_names_match_protocol_order(side: str) -> None:
    assert hand2_protocol_joint_names(side)[0] == f"{side[0]}_thumb_cmc_flex"
    assert len(hand2_protocol_joint_names(side)) == 20


@pytest.mark.parametrize("side", ["left", "right"])
def test_sdk_offline_smoke_is_finite_firmware_order(side: str) -> None:
    backend = SdkRetargetBackend(side)
    command = backend.retarget(synthetic_mediapipe_hand(0.55, side))
    assert command.shape == (20,)
    assert np.isfinite(command).all()


class FakeSession:
    def __init__(self, output: np.ndarray) -> None:
        self.output = output
        self.reset_count = 0

    def step(self, _keypoints: np.ndarray) -> np.ndarray:
        return self.output

    def reset(self) -> None:
        self.reset_count += 1


def test_sdk_output_is_not_reordered() -> None:
    expected = np.arange(20, dtype=np.float32)
    session = FakeSession(expected)
    backend = SdkRetargetBackend("left", session=session)
    actual = backend.retarget(synthetic_mediapipe_hand(0.5, "left"))
    np.testing.assert_array_equal(actual, expected)
    backend.reset()
    assert session.reset_count == 1


@pytest.mark.parametrize(
    "bad_output",
    [np.zeros(19), np.full(20, np.nan)],
)
def test_sdk_invalid_output_is_rejected(bad_output: np.ndarray) -> None:
    backend = SdkRetargetBackend("left", session=FakeSession(bad_output))
    with pytest.raises(ValueError):
        backend.retarget(synthetic_mediapipe_hand(0.5, "left"))


def test_degenerate_frame_is_rejected_before_sdk_can_emit_a_zero_command() -> None:
    session = FakeSession(np.zeros(20))
    backend = SdkRetargetBackend("left", session=session)
    with pytest.raises(ValueError, match="degenerate"):
        backend.retarget(np.zeros((21, 3)))
