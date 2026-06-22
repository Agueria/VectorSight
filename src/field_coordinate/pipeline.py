from __future__ import annotations

import json
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from field_coordinate.camera_geometry import (
    calculate_distance_m,
    pixel_scale_m,
    projected_aircraft_point,
    relative_target_offset_m,
)
from field_coordinate.config import FieldConfig
from field_coordinate.coordinates import local_offset_to_global, rotate_body_offset_to_north_east
from field_coordinate.models import Attitude, Detection, GlobalPosition, TargetEstimate
from field_coordinate.vision import is_reachable


def estimate_target(
    detection: Detection | None,
    position: GlobalPosition | None,
    attitude: Attitude | None,
    config: FieldConfig,
) -> TargetEstimate:
    if detection is None:
        return TargetEstimate(status="NO_TARGET", detection=None, message="No target detected")
    if config.focal_px is None:
        return TargetEstimate(
            status="UNCALIBRATED",
            detection=detection,
            message="Config does not contain focal_px",
        )
    if position is None:
        return TargetEstimate(
            status="NO_GPS",
            detection=detection,
            message="No global position telemetry",
        )
    if attitude is None:
        return TargetEstimate(
            status="NO_ATTITUDE",
            detection=detection,
            message="No attitude telemetry",
        )
    if position.relative_alt_m <= 0:
        return TargetEstimate(
            status="BAD_ALTITUDE",
            detection=detection,
            message="Relative altitude must be positive",
        )

    distance_m = calculate_distance_m(
        focal_px=config.focal_px,
        real_size_m=config.target_real_size_m,
        observed_size_px=detection.observed_diameter_px,
    )
    scale = pixel_scale_m(
        altitude_m=position.relative_alt_m,
        horizontal_fov_deg=config.horizontal_fov_deg,
        vertical_fov_deg=config.vertical_fov_deg,
        resolution=config.resolution,
        mode=config.geometry,
    )
    aircraft_point = projected_aircraft_point(
        roll_deg=attitude.roll_deg,
        pitch_deg=attitude.pitch_deg,
        resolution=config.resolution,
        horizontal_fov_deg=config.horizontal_fov_deg,
        vertical_fov_deg=config.vertical_fov_deg,
    )
    reachable = is_reachable(
        target=detection.center,
        aircraft=aircraft_point,
        resolution=config.resolution,
    )
    if not reachable:
        return TargetEstimate(
            status="NOT_REACHABLE",
            detection=detection,
            distance_m=distance_m,
            aircraft_point=aircraft_point,
            message="Target or projected aircraft point is outside the shooting zone",
            reachable=False,
        )

    body_offset = relative_target_offset_m(
        target=detection.center,
        origin=aircraft_point,
        scale=scale,
    )
    local_offset = rotate_body_offset_to_north_east(body_offset, yaw_deg=attitude.yaw_deg)
    target_position = local_offset_to_global(position.lat_deg, position.lon_deg, local_offset)

    return TargetEstimate(
        status="OK",
        detection=detection,
        local_offset=local_offset,
        global_position=target_position,
        distance_m=distance_m,
        aircraft_point=aircraft_point,
        reachable=True,
    )


def _json_default(value: Any) -> Any:
    if is_dataclass(value):
        return asdict(value)
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


def serialize_estimate(estimate: TargetEstimate) -> str:
    payload = asdict(estimate)
    payload["timestamp_utc"] = datetime.now(timezone.utc).isoformat()
    return json.dumps(payload, default=_json_default, sort_keys=True)


def append_log(log_path: str | Path, estimate: TargetEstimate) -> None:
    path = Path(log_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(serialize_estimate(estimate))
        handle.write("\n")
