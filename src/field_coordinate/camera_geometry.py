from __future__ import annotations

import math

from field_coordinate.config import GeometryMode
from field_coordinate.models import LocalOffset, PixelPoint, PixelScale, Resolution


def _require_positive(name: str, value: float) -> None:
    if value <= 0:
        raise ValueError(f"{name} must be positive")


def calculate_focal_px(
    *,
    known_distance_m: float,
    real_size_m: float,
    observed_size_px: float,
) -> float:
    _require_positive("known_distance_m", known_distance_m)
    _require_positive("real_size_m", real_size_m)
    _require_positive("observed_size_px", observed_size_px)
    return observed_size_px * known_distance_m / real_size_m


def calculate_distance_m(
    *,
    focal_px: float,
    real_size_m: float,
    observed_size_px: float,
) -> float:
    _require_positive("focal_px", focal_px)
    _require_positive("real_size_m", real_size_m)
    _require_positive("observed_size_px", observed_size_px)
    return real_size_m * focal_px / observed_size_px


def pixel_scale_m(
    *,
    altitude_m: float,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
    resolution: Resolution,
    mode: GeometryMode,
) -> PixelScale:
    _require_positive("altitude_m", altitude_m)
    _require_positive("horizontal_fov_deg", horizontal_fov_deg)
    _require_positive("vertical_fov_deg", vertical_fov_deg)

    h_angle = math.radians(horizontal_fov_deg)
    v_angle = math.radians(vertical_fov_deg)

    if mode == GeometryMode.LEGACY:
        x_m_per_px = (math.tan(h_angle / 4.0) * altitude_m) / (resolution.width / 4.0)
        y_m_per_px = (math.tan(v_angle / 4.0) * altitude_m) / (resolution.height / 4.0)
    else:
        x_m_per_px = (2.0 * math.tan(h_angle / 2.0) * altitude_m) / resolution.width
        y_m_per_px = (2.0 * math.tan(v_angle / 2.0) * altitude_m) / resolution.height

    return PixelScale(x_m_per_px=x_m_per_px, y_m_per_px=y_m_per_px)


def relative_target_offset_m(
    *,
    target: PixelPoint,
    origin: PixelPoint,
    scale: PixelScale,
) -> LocalOffset:
    rel_x_px = target.x - origin.x
    rel_y_px = target.y - origin.y
    east_m = rel_x_px * scale.x_m_per_px
    north_m = -rel_y_px * scale.y_m_per_px
    return LocalOffset(north_m=north_m, east_m=east_m)


def projected_aircraft_point(
    *,
    roll_deg: float,
    pitch_deg: float,
    resolution: Resolution,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
) -> PixelPoint:
    x_pixel_coef = resolution.width / horizontal_fov_deg
    y_pixel_coef = resolution.height / vertical_fov_deg
    origin = PixelPoint(x=resolution.width / 2.0, y=resolution.height / 2.0)
    return PixelPoint(
        x=origin.x + int(roll_deg * x_pixel_coef),
        y=origin.y + int(pitch_deg * y_pixel_coef),
    )
