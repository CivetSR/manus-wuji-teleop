from __future__ import annotations

import json
import struct
from types import SimpleNamespace

import pytest

from wuji_manus_bridge.tactile import (
    FingertipField,
    FingertipFormat,
    OFFICIAL_POINT_COUNTS,
    decode_fingertip_frame,
    format_for_finger,
)


def make_frame(point_count: int) -> bytes:
    points = b"".join(
        struct.pack("<hhh", index, -index, 100 + index)
        for index in range(point_count)
    )
    aggregate = struct.pack("<hhhh", 10, 20, 300, 250)
    return points + aggregate


def test_official_fallback_uses_40_thumb_points_and_34_elsewhere() -> None:
    assert tuple(format_for_finger(index).point_count for index in range(5)) == (
        40,
        34,
        34,
        34,
        34,
    )
    assert OFFICIAL_POINT_COUNTS == (40, 34, 34, 34, 34)


def test_thumb_aggregate_is_decoded_after_all_40_points() -> None:
    sensor_format = format_for_finger(0)
    decoded = decode_fingertip_frame(
        make_frame(40),
        sensor_format=sensor_format,
        haptic_scale_n=10.0,
    )

    assert decoded["point_count"] == 40
    assert decoded["active_points"] == 40
    assert decoded["peak_n"] == pytest.approx(1.39)
    assert decoded["agg_fz"] == pytest.approx(3.0)
    assert decoded["temp_c"] == pytest.approx(25.0)


def test_sdk_format_metadata_overrides_the_fallback_layout() -> None:
    info = SimpleNamespace(
        digest=123,
        format=json.dumps(
            {
                "v": 1,
                "encoding": "point_array",
                "point_count": 3,
                "point_stride": 8,
                "aggregate_stride": 12,
                "point_fields": [
                    {"name": "fz", "offset": 0, "type": "i16", "scale": 0.02},
                    {"name": "fy", "offset": 2, "type": "u8", "scale": 1.0},
                    {"name": "fx", "offset": 4, "type": "f32", "scale": 1.0},
                ],
                "aggregate_fields": [
                    {
                        "name": "temperature",
                        "offset": 0,
                        "type": "i16",
                        "scale": 0.5,
                    },
                    {"name": "fx", "offset": 2, "type": "i16", "scale": 0.02},
                    {"name": "fy", "offset": 4, "type": "i16", "scale": 0.02},
                    {"name": "fz", "offset": 8, "type": "f32", "scale": 1.0},
                ],
            }
        ),
    )
    sensor_format = format_for_finger(0, info)
    point = struct.pack("<hBBf", 50, 0, 0, 0.0)
    aggregate = struct.pack("<hhhhf", 20, 0, 0, 0, 2.0)
    decoded = decode_fingertip_frame(
        point * 3 + aggregate,
        sensor_format=sensor_format,
    )

    assert sensor_format.digest == 123
    assert decoded["point_count"] == 3
    assert decoded["peak_n"] == pytest.approx(1.0)
    assert decoded["agg_fz"] == pytest.approx(2.0)
    assert decoded["temp_c"] == pytest.approx(10.0)


def test_truncated_payload_is_dropped_instead_of_partially_decoded() -> None:
    with pytest.raises(ValueError, match="frame length"):
        decode_fingertip_frame(
            make_frame(34),
            sensor_format=format_for_finger(0),
        )


def test_invalid_self_describing_format_fails_fast() -> None:
    info = SimpleNamespace(
        digest=1,
        format=json.dumps(
            {
                "v": 1,
                "encoding": "point_array",
                "point_count": "40",
                "point_stride": 6,
                "aggregate_stride": 8,
                "point_fields": [],
                "aggregate_fields": [],
            }
        ),
    )
    with pytest.raises(ValueError, match="positive integer"):
        format_for_finger(0, info)


def test_nonfinite_float_field_is_dropped() -> None:
    sensor_format = FingertipFormat(
        point_count=1,
        point_stride=12,
        point_fields=(
            FingertipField("fx", 0, "f32"),
            FingertipField("fy", 4, "f32"),
            FingertipField("fz", 8, "f32"),
        ),
    )
    payload = struct.pack("<fff", 0.0, 0.0, float("nan")) + struct.pack(
        "<hhhh", 0, 0, 0, 250
    )
    with pytest.raises(ValueError, match="NaN"):
        decode_fingertip_frame(payload, sensor_format=sensor_format)
