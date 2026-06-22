from __future__ import annotations

import math

from field_coordinate.models import GlobalPosition, LocalOffset


EARTH_RADIUS_M = 6_378_137.0


def rotate_body_offset_to_north_east(offset: LocalOffset, *, yaw_deg: float) -> LocalOffset:
    yaw = math.radians(yaw_deg)
    north_m = offset.north_m * math.cos(yaw) - offset.east_m * math.sin(yaw)
    east_m = offset.north_m * math.sin(yaw) + offset.east_m * math.cos(yaw)
    return LocalOffset(north_m=north_m, east_m=east_m)


def local_offset_to_global(
    lat_deg: float,
    lon_deg: float,
    offset: LocalOffset,
) -> GlobalPosition:
    lat_rad = math.radians(lat_deg)
    delta_lat = offset.north_m / EARTH_RADIUS_M
    cos_lat = math.cos(lat_rad)
    if abs(cos_lat) < 1e-12:
        raise ValueError("Longitude is unstable near the poles")

    delta_lon = offset.east_m / (EARTH_RADIUS_M * cos_lat)
    return GlobalPosition(
        lat_deg=lat_deg + math.degrees(delta_lat),
        lon_deg=lon_deg + math.degrees(delta_lon),
    )


def offset_distance_m(offset: LocalOffset) -> float:
    return math.hypot(offset.north_m, offset.east_m)
