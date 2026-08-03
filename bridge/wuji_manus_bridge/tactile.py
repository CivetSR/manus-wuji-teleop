"""Decode Wuji Hand 2 fingertip tactile binary frames."""

from __future__ import annotations

import json
import math
import struct
from dataclasses import dataclass
from typing import Any, Dict, List

OFFICIAL_POINT_COUNTS = (40, 34, 34, 34, 34)
DEFAULT_POINT_STRIDE = 6
DEFAULT_AGGREGATE_STRIDE = 8

# Default haptic mapping: map |agg_fz| and peak into [0, 1].
# Tunable; Manus side may ignore haptic_01 and recompute from peak_n/agg_fz.
DEFAULT_HAPTIC_SCALE_N = 2.0  # Newtons -> full vibration

FIELD_FORMATS = {
    "i8": "<b",
    "u8": "<B",
    "i16": "<h",
    "u16": "<H",
    "i32": "<i",
    "u32": "<I",
    "f32": "<f",
}


@dataclass(frozen=True)
class FingertipField:
    """One field from the official v1 point-array format contract."""

    name: str
    offset: int
    field_type: str
    scale: float = 1.0

    def read(self, data: bytes, base: int) -> float:
        return float(
            struct.unpack_from(FIELD_FORMATS[self.field_type], data, base + self.offset)[0]
        ) * self.scale


FALLBACK_POINT_FIELDS = (
    FingertipField("fx", 0, "i16", 0.01),
    FingertipField("fy", 2, "i16", 0.01),
    FingertipField("fz", 4, "i16", 0.01),
)
FALLBACK_AGGREGATE_FIELDS = (
    FingertipField("fx", 0, "i16", 0.01),
    FingertipField("fy", 2, "i16", 0.01),
    FingertipField("fz", 4, "i16", 0.01),
    FingertipField("temperature", 6, "i16", 0.1),
)


@dataclass(frozen=True)
class FingertipFormat:
    """Validated SDK v1 ``point_array`` layout or official fixed fallback."""

    point_count: int
    point_stride: int = DEFAULT_POINT_STRIDE
    aggregate_stride: int = DEFAULT_AGGREGATE_STRIDE
    point_fields: tuple[FingertipField, ...] = FALLBACK_POINT_FIELDS
    aggregate_fields: tuple[FingertipField, ...] = FALLBACK_AGGREGATE_FIELDS
    digest: int | None = None


def format_for_finger(finger_index: int, info: Any | None = None) -> FingertipFormat:
    """Prefer SDK metadata and otherwise use the official Hand 2 sensor counts."""

    if not 0 <= finger_index < len(OFFICIAL_POINT_COUNTS):
        raise ValueError(f"finger_index must be 0..4, got {finger_index}")
    defaults = FingertipFormat(point_count=OFFICIAL_POINT_COUNTS[finger_index])
    if info is None:
        return defaults

    raw_format = getattr(info, "format", None)
    if not isinstance(raw_format, str) or not raw_format.strip():
        return FingertipFormat(
            point_count=defaults.point_count,
            digest=_optional_int(getattr(info, "digest", None)),
        )
    try:
        metadata = json.loads(
            raw_format,
            parse_constant=lambda token: (_raise_nonfinite_json(token)),
        )
    except (json.JSONDecodeError, ValueError) as exc:
        raise ValueError(f"invalid fingertip info.format JSON: {exc}") from exc
    if not isinstance(metadata, dict):
        raise ValueError("fingertip info.format must decode to a JSON object")

    if metadata.get("v") != 1:
        raise ValueError(f"unsupported fingertip format version: {metadata.get('v')!r}")
    if metadata.get("encoding") != "point_array":
        raise ValueError(
            f"unsupported fingertip format encoding: {metadata.get('encoding')!r}"
        )
    point_count = _positive_int(metadata.get("point_count"), "point_count")
    point_stride = _positive_int(metadata.get("point_stride"), "point_stride")
    aggregate_stride = _positive_int(
        metadata.get("aggregate_stride"), "aggregate_stride"
    )
    point_fields = _parse_fields(
        metadata.get("point_fields"),
        "point_fields",
        point_stride,
        required={"fx", "fy", "fz"},
    )
    aggregate_fields = _parse_fields(
        metadata.get("aggregate_fields"),
        "aggregate_fields",
        aggregate_stride,
        required={"fx", "fy", "fz", "temperature"},
    )
    return FingertipFormat(
        point_count=point_count,
        point_stride=point_stride,
        aggregate_stride=aggregate_stride,
        point_fields=point_fields,
        aggregate_fields=aggregate_fields,
        digest=_optional_int(getattr(info, "digest", None)),
    )


def _raise_nonfinite_json(token: str) -> None:
    raise ValueError(f"non-standard JSON numeric token is forbidden: {token}")


def _optional_int(value: Any) -> int | None:
    if value is None:
        return None
    if type(value) is not int or value < 0:
        raise ValueError(
            f"fingertip metadata digest must be a non-negative integer, got {value!r}"
        )
    return value


def _positive_int(value: Any, field: str) -> int:
    if type(value) is not int or value <= 0:
        raise ValueError(f"fingertip format {field} must be a positive integer")
    return value


def _positive_float(value: Any, field: str) -> float:
    if (
        type(value) not in (int, float)
        or not math.isfinite(float(value))
        or float(value) <= 0.0
    ):
        raise ValueError(f"fingertip format {field} must be a positive number")
    return float(value)


def _parse_fields(
    raw_fields: Any,
    field_group: str,
    stride: int,
    *,
    required: set[str],
) -> tuple[FingertipField, ...]:
    if not isinstance(raw_fields, list) or not raw_fields:
        raise ValueError(f"fingertip format {field_group} must be a non-empty list")
    parsed: list[FingertipField] = []
    seen: set[str] = set()
    for index, raw in enumerate(raw_fields):
        if not isinstance(raw, dict):
            raise ValueError(f"fingertip format {field_group}[{index}] must be an object")
        name = raw.get("name")
        if not isinstance(name, str) or not name or name in seen:
            raise ValueError(
                f"fingertip format {field_group}[{index}] has invalid/duplicate name"
            )
        offset = raw.get("offset")
        if type(offset) is not int or offset < 0:
            raise ValueError(
                f"fingertip format {field_group}[{index}].offset must be non-negative"
            )
        field_type = raw.get("type")
        if field_type not in FIELD_FORMATS:
            raise ValueError(
                f"fingertip format {field_group}[{index}] has unsupported type "
                f"{field_type!r}"
            )
        width = struct.calcsize(FIELD_FORMATS[field_type])
        if offset + width > stride:
            raise ValueError(
                f"fingertip format {field_group}[{index}] exceeds stride {stride}"
            )
        parsed.append(
            FingertipField(
                name=name,
                offset=offset,
                field_type=field_type,
                scale=_positive_float(raw.get("scale", 1.0), f"{field_group}.scale"),
            )
        )
        seen.add(name)
    missing = sorted(required - seen)
    if missing:
        raise ValueError(f"fingertip format {field_group} is missing fields: {missing}")
    return tuple(parsed)


def _to_bytes(data: Any) -> bytes:
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    if hasattr(data, "tobytes"):
        return data.tobytes()
    return bytes(data)


def decode_fingertip_frame(
    data: Any,
    *,
    sensor_format: FingertipFormat,
    haptic_scale_n: float = DEFAULT_HAPTIC_SCALE_N,
) -> Dict[str, float | int]:
    raw = _to_bytes(data)
    point_bytes = sensor_format.point_count * sensor_format.point_stride
    expected_bytes = point_bytes + sensor_format.aggregate_stride
    if len(raw) != expected_bytes:
        raise ValueError(
            "invalid fingertip frame length: "
            f"expected {expected_bytes} bytes for {sensor_format.point_count} points, "
            f"got {len(raw)} bytes"
        )
    fzs: List[float] = []
    for i in range(sensor_format.point_count):
        off = i * sensor_format.point_stride
        point = {
            field.name: field.read(raw, off) for field in sensor_format.point_fields
        }
        if not all(math.isfinite(value) for value in point.values()):
            raise ValueError("fingertip point fields contain NaN or infinity")
        fzs.append(point["fz"])

    agg = {
        field.name: field.read(raw, point_bytes)
        for field in sensor_format.aggregate_fields
    }
    if not all(math.isfinite(value) for value in agg.values()):
        raise ValueError("fingertip aggregate fields contain NaN or infinity")

    active = [v for v in fzs if abs(v) > 0.005]
    peak = max((abs(v) for v in fzs), default=0.0)
    mean = (sum(active) / len(active)) if active else 0.0
    scale = _positive_float(haptic_scale_n, "haptic_scale_n")
    haptic = min(1.0, max(peak, abs(agg["fz"])) / scale)

    return {
        "peak_n": float(peak),
        "mean_n": float(mean),
        "agg_fx": float(agg["fx"]),
        "agg_fy": float(agg["fy"]),
        "agg_fz": float(agg["fz"]),
        "temp_c": float(agg["temperature"]),
        "active_points": len(active),
        "point_count": sensor_format.point_count,
        "haptic_01": float(haptic),
    }


EMPTY_FINGER = {
    "peak_n": 0.0,
    "mean_n": 0.0,
    "agg_fx": 0.0,
    "agg_fy": 0.0,
    "agg_fz": 0.0,
    "temp_c": 0.0,
    "active_points": 0.0,
    "point_count": 0.0,
    "haptic_01": 0.0,
}
