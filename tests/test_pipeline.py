import json

import pytest

from field_coordinate.config import FieldConfig, GeometryMode, ResolutionPreset
from field_coordinate.models import Attitude, Detection, GlobalPosition, PixelPoint
from field_coordinate.pipeline import append_log, estimate_target, serialize_estimate


def test_estimate_target_chains_detection_telemetry_and_user_geometry() -> None:
    config = FieldConfig(
        resolution_preset=ResolutionPreset.HD_720,
        geometry_mode=GeometryMode.LEGACY,
        horizontal_fov_deg=62.2,
        vertical_fov_deg=48.8,
        target_real_size_m=0.07,
        focal_px=980.0,
    )
    detection = Detection(
        center=PixelPoint(x=740.0, y=360.0),
        observed_width_px=70.0,
        observed_height_px=70.0,
        area_px=5000.0,
        confidence=0.8,
    )
    position = GlobalPosition(lat_deg=41.0, lon_deg=29.0, relative_alt_m=20.0)
    attitude = Attitude(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0)

    estimate = estimate_target(detection, position, attitude, config)

    assert estimate.status == "OK"
    assert estimate.distance_m == pytest.approx(0.98)
    assert estimate.local_offset.east_m > 0.0
    assert estimate.global_position.lon_deg > 29.0


def test_estimate_target_uses_roll_pitch_projected_aircraft_point() -> None:
    config = FieldConfig(
        resolution_preset=ResolutionPreset.HD_720,
        geometry_mode=GeometryMode.LEGACY,
        focal_px=980.0,
    )
    detection = Detection(
        center=PixelPoint(x=640.0, y=360.0),
        observed_width_px=70.0,
        observed_height_px=70.0,
        area_px=5000.0,
        confidence=0.8,
    )

    level = estimate_target(
        detection,
        GlobalPosition(lat_deg=41.0, lon_deg=29.0, relative_alt_m=20.0),
        Attitude(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0),
        config,
    )
    tilted = estimate_target(
        detection,
        GlobalPosition(lat_deg=41.0, lon_deg=29.0, relative_alt_m=20.0),
        Attitude(roll_deg=10.0, pitch_deg=-5.0, yaw_deg=0.0),
        config,
    )

    assert level.local_offset is not None
    assert tilted.local_offset is not None
    assert level.local_offset.east_m == pytest.approx(0.0)
    assert level.local_offset.north_m == pytest.approx(0.0)
    assert abs(tilted.local_offset.east_m) > 0.0
    assert abs(tilted.local_offset.north_m) > 0.0
    assert level.aircraft_point == PixelPoint(x=640.0, y=360.0)
    assert tilted.aircraft_point != level.aircraft_point


def test_estimate_target_refuses_uncalibrated_config() -> None:
    config = FieldConfig(
        resolution_preset=ResolutionPreset.HD_720,
        geometry_mode=GeometryMode.LEGACY,
        focal_px=None,
    )
    detection = Detection(
        center=PixelPoint(x=640.0, y=360.0),
        observed_width_px=70.0,
        observed_height_px=70.0,
        area_px=5000.0,
        confidence=0.8,
    )
    position = GlobalPosition(lat_deg=41.0, lon_deg=29.0, relative_alt_m=20.0)
    attitude = Attitude(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0)

    estimate = estimate_target(detection, position, attitude, config)

    assert estimate.status == "UNCALIBRATED"


def test_estimate_target_statuses_for_missing_inputs() -> None:
    config = FieldConfig(focal_px=980.0)
    detection = Detection(
        center=PixelPoint(x=640.0, y=360.0),
        observed_width_px=70.0,
        observed_height_px=70.0,
        area_px=5000.0,
        confidence=0.8,
    )
    position = GlobalPosition(lat_deg=41.0, lon_deg=29.0, relative_alt_m=20.0)
    attitude = Attitude(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0)

    assert estimate_target(None, position, attitude, config).status == "NO_TARGET"
    assert estimate_target(detection, None, attitude, config).status == "NO_GPS"
    assert estimate_target(detection, position, None, config).status == "NO_ATTITUDE"
    assert (
        estimate_target(
            detection,
            GlobalPosition(lat_deg=41.0, lon_deg=29.0, relative_alt_m=0.0),
            attitude,
            config,
        ).status
        == "BAD_ALTITUDE"
    )


def test_serialize_estimate_outputs_jsonl_safe_payload() -> None:
    config = FieldConfig(focal_px=980.0)
    detection = Detection(
        center=PixelPoint(x=640.0, y=360.0),
        observed_width_px=70.0,
        observed_height_px=70.0,
        area_px=5000.0,
        confidence=0.8,
    )
    estimate = estimate_target(
        detection,
        GlobalPosition(lat_deg=41.0, lon_deg=29.0, relative_alt_m=20.0),
        Attitude(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0),
        config,
    )

    payload = json.loads(serialize_estimate(estimate))

    assert payload["status"] == "OK"
    assert "global_position" in payload
    assert "local_offset" in payload


def test_append_log_writes_one_json_line(tmp_path) -> None:
    estimate = estimate_target(
        Detection(
            center=PixelPoint(x=640.0, y=360.0),
            observed_width_px=70.0,
            observed_height_px=70.0,
            area_px=5000.0,
            confidence=0.8,
        ),
        GlobalPosition(lat_deg=41.0, lon_deg=29.0, relative_alt_m=20.0),
        Attitude(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0),
        FieldConfig(focal_px=980.0),
    )
    log_path = tmp_path / "nested" / "session.jsonl"

    append_log(log_path, estimate)

    lines = log_path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1
    assert json.loads(lines[0])["status"] == "OK"
