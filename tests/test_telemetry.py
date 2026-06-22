import math
from types import SimpleNamespace

import pytest

from field_coordinate.telemetry import LatestTelemetryCache, normalize_attitude, normalize_global_position
from field_coordinate.state import AttitudeChangeFilter


def test_normalize_attitude_converts_radians_to_degrees() -> None:
    msg = SimpleNamespace(
        roll=math.radians(10.0),
        pitch=math.radians(-5.0),
        yaw=math.radians(90.0),
    )

    attitude = normalize_attitude(msg)

    assert attitude.roll_deg == pytest.approx(10.0)
    assert attitude.pitch_deg == pytest.approx(-5.0)
    assert attitude.yaw_deg == pytest.approx(90.0)


def test_normalize_global_position_converts_mavlink_units() -> None:
    msg = SimpleNamespace(
        lat=410000000,
        lon=290000000,
        relative_alt=12345,
    )

    position = normalize_global_position(msg)

    assert position.lat_deg == pytest.approx(41.0)
    assert position.lon_deg == pytest.approx(29.0)
    assert position.relative_alt_m == pytest.approx(12.345)


def test_normalize_global_position_rejects_missing_fields() -> None:
    with pytest.raises(ValueError):
        normalize_global_position(SimpleNamespace(lat=410000000))


def test_normalize_attitude_rejects_missing_fields() -> None:
    with pytest.raises(ValueError):
        normalize_attitude(SimpleNamespace(roll=0.0, pitch=0.0))


def test_latest_telemetry_cache_keeps_recent_position_and_attitude() -> None:
    cache = LatestTelemetryCache(max_age_s=1.0)
    cache.update_position(
        SimpleNamespace(
            lat=410000000,
            lon=290000000,
            relative_alt=12345,
        ),
        now_s=10.0,
    )
    cache.update_attitude(
        SimpleNamespace(
            roll=0.0,
            pitch=0.0,
            yaw=math.radians(45.0),
        ),
        now_s=10.5,
    )

    position, attitude = cache.snapshot(now_s=10.8)

    assert position is not None
    assert attitude is not None
    assert position.lat_deg == pytest.approx(41.0)
    assert attitude.yaw_deg == pytest.approx(45.0)


def test_latest_telemetry_cache_tracks_attitude_changes() -> None:
    change_filter = AttitudeChangeFilter(max_history=3)
    cache = LatestTelemetryCache(max_age_s=1.0, attitude_filter=change_filter)

    cache.update_attitude(
        SimpleNamespace(roll=0.0, pitch=0.0, yaw=math.radians(45.0)),
        now_s=10.0,
    )
    assert cache.last_attitude_change_is_new is True

    cache.update_attitude(
        SimpleNamespace(roll=0.0, pitch=0.0, yaw=math.radians(45.0)),
        now_s=10.5,
    )

    _, attitude = cache.snapshot(now_s=10.8)
    assert cache.last_attitude_change_is_new is False
    assert attitude is not None
    assert attitude.yaw_deg == pytest.approx(45.0)
    assert len(change_filter.history) == 2


def test_latest_telemetry_cache_expires_stale_values() -> None:
    cache = LatestTelemetryCache(max_age_s=1.0)
    cache.update_position(
        SimpleNamespace(
            lat=410000000,
            lon=290000000,
            relative_alt=12345,
        ),
        now_s=10.0,
    )

    position, attitude = cache.snapshot(now_s=11.1)

    assert position is None
    assert attitude is None
