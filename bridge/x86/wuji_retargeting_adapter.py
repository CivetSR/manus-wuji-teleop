"""Unified per-hand retargeting backends for Wuji Hand 2."""

from __future__ import annotations

import argparse
import importlib.metadata
import os
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np
import yaml

from joint_order import (
    hand2_protocol_joint_names,
    mujoco_actuator_joint_names,
    strict_joint_name_permutation,
)
from manus_keypoints import InvalidManusFrame, validate_mediapipe_keypoints

TELEOP_ROOT = Path(__file__).resolve().parents[2]
MINIMUM_RETARGETING_VERSION = (2026, 8, 3)
PINNED_SDK_VERSION = (2026, 8, 3)
PINNED_DESCRIPTION_COMMIT = "8271644a78d69ed9a4adcf9165d882c64ad33dfa"
DESCRIPTION_ROOT_TOKEN = "${WUJI_DESCRIPTION_ROOT}"
RETARGET_BACKENDS = ("retargeter", "sdk")

# AdaptiveOptimizerAnalytical's biomechanical priors (soft_min/w_hyper forbid
# negative joint angles) assume the hand2/body joint frames. hand2_beta1 rotates
# every *_mcp_flex origin by ~pi, which inverts the flexion direction, so those
# priors drive all four MCP joints onto their +pi/2 limit and hold them there for
# any input. Measured: 7/20 joints frozen and zero MCP travel across a full
# open-to-close sweep, versus smooth tracking on hand2/body.
RETARGETER_INCOMPATIBLE_GENERATIONS = ("hand2_beta1",)


class PerHandRetargetBackend(Protocol):
    """Stateful interface shared by both per-hand implementations."""

    side: str
    name: str
    initialization: str

    def retarget(self, keypoints: np.ndarray) -> np.ndarray:
        """Return a finite 20-vector in Hand2 firmware/TCP order."""

    def reset(self) -> None:
        """Reset warm-start and filter state."""


@dataclass(frozen=True)
class Hand2ModelPaths:
    root: Path
    urdf: Path
    mjcf: Path


def _version_tuple(version: str) -> tuple[int, ...]:
    try:
        return tuple(int(part) for part in version.split("."))
    except ValueError as exc:
        raise RuntimeError(f"unrecognized package version: {version!r}") from exc


def _validate_side(side: str) -> str:
    side = side.lower()
    if side not in ("left", "right"):
        raise ValueError(f"side must be left or right, got {side!r}")
    return side


def validate_backend_name(value: str) -> str:
    value = str(value).lower()
    if value not in RETARGET_BACKENDS:
        raise ValueError(
            f"retarget backend must be one of {RETARGET_BACKENDS}, got {value!r}"
        )
    return value


def default_retarget_backend() -> str:
    return validate_backend_name(os.environ.get("RETARGET_BACKEND", "sdk"))


def add_retarget_backend_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--retarget-backend",
        choices=RETARGET_BACKENDS,
        default=default_retarget_backend(),
        help=(
            "sdk=RetargetSession, firmware-matched and the only backend that "
            "supports the pinned Hand2 Beta1 model; retargeter=Python IK, "
            "requires a hand2/body-generation model"
        ),
    )


def find_wuji_retargeting_root() -> Path:
    """Find official algorithm code from the environment or adjacent checkout."""

    configured = os.environ.get("WUJI_RETARGETING_ROOT")
    candidate = (
        Path(configured).expanduser()
        if configured
        else TELEOP_ROOT.parent / "wuji-retargeting"
    ).resolve()
    if (candidate / "pyproject.toml").is_file() and (
        candidate / "wuji_retargeting" / "retarget.py"
    ).is_file():
        return candidate
    raise FileNotFoundError(
        "official wuji-retargeting checkout not found; set WUJI_RETARGETING_ROOT "
        f"or clone it adjacent to this repository (checked {candidate})"
    )


def find_wuji_description_root() -> Path:
    """Find the separately pinned v2026.8.3 model checkout."""

    configured = os.environ.get("WUJI_DESCRIPTION_ROOT")
    candidate = (
        Path(configured).expanduser()
        if configured
        else TELEOP_ROOT / "deps" / "wuji-description"
    )
    if candidate.is_symlink() and not candidate.exists():
        raise RuntimeError(f"WUJI_DESCRIPTION_ROOT is a broken symlink: {candidate}")
    candidate = candidate.resolve()
    if not (candidate / "hand2" / "hand2_beta1" / "body").is_dir():
        raise FileNotFoundError(
            "pinned wuji-description checkout not found; set WUJI_DESCRIPTION_ROOT "
            f"or run setup.sh (checked {candidate})"
        )
    return candidate


def validate_description_checkout(root: Path) -> str:
    """Require the exact model revision selected for IK and MuJoCo."""

    try:
        result = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError(f"WUJI_DESCRIPTION_ROOT is not a readable git checkout: {root}") from exc
    revision = result.stdout.strip()
    if revision != PINNED_DESCRIPTION_COMMIT:
        raise RuntimeError(
            "wuji-description model revision mismatch; "
            f"expected {PINNED_DESCRIPTION_COMMIT}, got {revision} at {root}"
        )
    checkout_top = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "--show-toplevel"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if Path(checkout_top).resolve() != root.resolve():
        raise RuntimeError(
            f"WUJI_DESCRIPTION_ROOT must be the checkout root: selected={root}, top={checkout_top}"
        )
    model_relative = "hand2/hand2_beta1/body"
    model_diff = subprocess.run(
        ["git", "-C", str(root), "diff", "--quiet", "HEAD", "--", model_relative],
        check=False,
    )
    if model_diff.returncode == 1:
        raise RuntimeError(
            f"pinned Hand2 Beta1 model has tracked modifications at {root / model_relative}"
        )
    if model_diff.returncode != 0:
        raise RuntimeError(f"failed to verify model cleanliness at {root}")
    if (root / model_relative).is_symlink():
        raise RuntimeError("pinned Hand2 Beta1 body must not be a symbolic link")
    return revision


def validate_official_install(root: Path) -> str:
    """Require imported wuji-retargeting code to be the selected editable checkout."""

    import wuji_retargeting

    imported_root = Path(wuji_retargeting.__file__).resolve().parent.parent
    if imported_root != root.resolve():
        raise RuntimeError(
            "wuji_retargeting import does not match WUJI_RETARGETING_ROOT; "
            f"imported={imported_root} selected={root}. "
            f"Run: python3 -m pip install -e {root}"
        )
    version = importlib.metadata.version("wuji-retargeting")
    if _version_tuple(version) < MINIMUM_RETARGETING_VERSION:
        minimum = ".".join(str(part) for part in MINIMUM_RETARGETING_VERSION)
        raise RuntimeError(
            f"wuji-retargeting {version} is too old; version >= {minimum} is required"
        )
    return version


def official_hand2_model_paths(
    side: str,
    description_root: Path | None = None,
) -> Hand2ModelPaths:
    side = _validate_side(side)
    description_root = (description_root or find_wuji_description_root()).resolve()
    validate_description_checkout(description_root)
    model_root = description_root / "hand2" / "hand2_beta1" / "body"
    paths = Hand2ModelPaths(
        root=model_root,
        urdf=model_root / "urdf" / f"{side}.urdf",
        mjcf=model_root / "mjcf" / f"{side}.xml",
    )
    missing = [path for path in (paths.urdf, paths.mjcf) if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"pinned Hand2 Beta1 assets are incomplete: {missing}")
    return paths


def config_path_for_side(side: str) -> Path:
    side = _validate_side(side)
    return TELEOP_ROOT / "bridge" / "config" / f"retarget_manus_hand2_{side}.yaml"


def materialize_config(
    config_path: Path,
    side: str,
    description_root: Path | None = None,
) -> tuple[dict[str, Any], Hand2ModelPaths]:
    """Resolve model tokens and reject mixed URDF/MJCF generations."""

    description_root = (description_root or find_wuji_description_root()).resolve()
    model = official_hand2_model_paths(side, description_root)
    with Path(config_path).open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if not isinstance(config, dict):
        raise ValueError(f"retarget config root must be a mapping: {config_path}")
    metadata = config.get("model") or {}
    if metadata.get("description_commit") != PINNED_DESCRIPTION_COMMIT:
        raise ValueError(
            f"config must pin wuji-description commit {PINNED_DESCRIPTION_COMMIT}"
        )

    optimizer = config.get("optimizer")
    if not isinstance(optimizer, dict):
        raise ValueError(f"retarget config is missing optimizer mapping: {config_path}")
    for key in ("urdf_path", "mjcf_path"):
        raw = optimizer.get(key)
        if not isinstance(raw, str) or DESCRIPTION_ROOT_TOKEN not in raw:
            raise ValueError(
                f"optimizer.{key} must use {DESCRIPTION_ROOT_TOKEN} "
                "so model provenance is explicit"
            )
        optimizer[key] = raw.replace(DESCRIPTION_ROOT_TOKEN, str(description_root))

    configured_urdf = Path(optimizer["urdf_path"]).resolve()
    configured_mjcf = Path(optimizer["mjcf_path"]).resolve()
    if configured_urdf != model.urdf or configured_mjcf != model.mjcf:
        raise ValueError(
            "Retargeter IK and MuJoCo must use the same pinned Hand2 Beta1 checkout; "
            f"configured urdf={configured_urdf}, mjcf={configured_mjcf}, "
            f"expected urdf={model.urdf}, mjcf={model.mjcf}"
        )
    return config, model


def _retargeter_from_materialized_yaml(config: dict[str, Any], side: str) -> Any:
    """Call the official ``Retargeter.from_yaml`` with absolute pinned paths."""

    from wuji_retargeting import Retargeter

    with tempfile.TemporaryDirectory(prefix="manus-wuji-retarget-") as directory:
        generated = Path(directory) / f"retarget_manus_hand2_{side}.yaml"
        with generated.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(config, stream, sort_keys=False)
        return Retargeter.from_yaml(str(generated), hand_side=side)


def _validated_keypoints(keypoints: np.ndarray) -> np.ndarray:
    try:
        return validate_mediapipe_keypoints(keypoints)
    except InvalidManusFrame as exc:
        raise ValueError(str(exc)) from exc


def _validated_firmware_output(output: Any, backend_name: str) -> np.ndarray:
    command = np.asarray(output, dtype=np.float64)
    if command.shape != (20,):
        raise ValueError(
            f"{backend_name} returned shape {command.shape}; expected firmware shape (20,)"
        )
    if not np.isfinite(command).all():
        raise ValueError(f"{backend_name} returned NaN or infinity")
    return command.copy()


class RetargeterBackend:
    """Official Python Retargeter with exactly one name-based reorder."""

    name = "retargeter"

    def __init__(self, side: str, config_path: Path | None = None) -> None:
        import mujoco

        self.side = _validate_side(side)
        self.checkout_root = find_wuji_retargeting_root()
        self.version = validate_official_install(self.checkout_root)
        self.description_root = find_wuji_description_root()
        self.description_revision = validate_description_checkout(self.description_root)
        self.config_path = Path(config_path or config_path_for_side(self.side)).resolve()
        self.config, self.model_paths = materialize_config(
            self.config_path, self.side, self.description_root
        )
        self.generation = self.model_paths.root.parent.name
        if self.generation in RETARGETER_INCOMPATIBLE_GENERATIONS:
            raise RuntimeError(
                f"retarget backend 'retargeter' does not support the {self.generation} "
                "model generation: its *_mcp_flex joint frames are rotated ~pi relative "
                "to hand2/body, so the official optimizer pins every MCP joint at its "
                "+pi/2 limit and the fingers never follow the glove. "
                "Use the 'sdk' backend (default), which is matched to Hand 2 firmware."
            )
        self.retargeter = _retargeter_from_materialized_yaml(self.config, self.side)

        self.source_joint_names = tuple(self.retargeter.optimizer.robot.dof_joint_names)
        self.firmware_joint_names = hand2_protocol_joint_names(self.side)
        self.qpos_to_firmware = strict_joint_name_permutation(
            self.source_joint_names, self.firmware_joint_names
        )

        model = mujoco.MjModel.from_xml_path(str(self.model_paths.mjcf))
        actuator_names = mujoco_actuator_joint_names(model)
        strict_joint_name_permutation(actuator_names, self.firmware_joint_names)
        if actuator_names != self.firmware_joint_names:
            raise ValueError(
                "pinned Hand2 MJCF actuator order no longer matches firmware order"
            )
        self.initialization = (
            f"wuji-retargeting={self.version} model={self.description_revision[:12]} "
            f"perm={self.qpos_to_firmware.tolist()}"
        )

    def retarget(self, keypoints: np.ndarray) -> np.ndarray:
        keypoints = _validated_keypoints(keypoints)
        qpos = np.asarray(self.retargeter.retarget(keypoints), dtype=np.float64)
        expected = len(self.source_joint_names)
        if qpos.shape != (expected,):
            raise ValueError(f"Retargeter returned shape {qpos.shape}, expected ({expected},)")
        if not np.isfinite(qpos).all():
            raise ValueError("Retargeter returned NaN or infinity")
        # This is the sole reorder in this backend.
        return _validated_firmware_output(
            qpos[self.qpos_to_firmware], self.name
        )

    def reset(self) -> None:
        self.retargeter.reset()


class SdkRetargetBackend:
    """wuji-sdk RetargetSession; output is already firmware order."""

    name = "sdk"

    def __init__(self, side: str, *, session: Any | None = None) -> None:
        self.side = _validate_side(side)
        if session is None:
            try:
                import wuji_sdk
            except ImportError as exc:
                raise RuntimeError(
                    "RETARGET_BACKEND=sdk requires wuji-sdk == 2026.8.3"
                ) from exc
            version = importlib.metadata.version("wuji-sdk")
            if _version_tuple(version) != PINNED_SDK_VERSION:
                raise RuntimeError(
                    f"wuji-sdk {version} is unsupported; exactly 2026.8.3 is required"
                )
            handedness = (
                wuji_sdk.Handedness.Left
                if self.side == "left"
                else wuji_sdk.Handedness.Right
            )
            try:
                session = wuji_sdk.RetargetSession.for_hand(
                    wuji_sdk.HandModel.WujiHand2,
                    handedness,
                )
            except Exception as exc:
                raise RuntimeError(
                    f"failed to initialize wuji-sdk RetargetSession for {self.side}: {exc}"
                ) from exc
            self.version = version
        else:
            self.version = "injected"
        self.session = session
        self.initialization = (
            f"wuji-sdk={self.version} HandModel.WujiHand2 "
            f"Handedness.{self.side.capitalize()} output=firmware-order(no-reorder)"
        )

    def retarget(self, keypoints: np.ndarray) -> np.ndarray:
        keypoints = _validated_keypoints(keypoints)
        # RetargetSession.step() explicitly returns firmware order. Do not reorder.
        return _validated_firmware_output(self.session.step(keypoints), self.name)

    def reset(self) -> None:
        self.session.reset()


def create_retarget_backend(
    backend: str,
    side: str,
    config_path: Path | None = None,
) -> PerHandRetargetBackend:
    backend = validate_backend_name(backend)
    if backend == "retargeter":
        return RetargeterBackend(side, config_path)
    return SdkRetargetBackend(side)
