from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Resolution:
    width: int
    height: int


@dataclass(frozen=True)
class PixelPoint:
    x: float
    y: float


@dataclass(frozen=True)
class PixelScale:
    x_m_per_px: float
    y_m_per_px: float


@dataclass(frozen=True)
class PixelZone:
    top_left: PixelPoint
    bottom_right: PixelPoint


@dataclass(frozen=True)
class LocalOffset:
    north_m: float
    east_m: float


@dataclass(frozen=True)
class GlobalPosition:
    lat_deg: float
    lon_deg: float
    relative_alt_m: float = 0.0


@dataclass(frozen=True)
class Attitude:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class Detection:
    center: PixelPoint
    observed_width_px: float
    observed_height_px: float
    area_px: float
    confidence: float
    radius_px: float | None = None

    @property
    def observed_diameter_px(self) -> float:
        return max(self.observed_width_px, self.observed_height_px)


@dataclass(frozen=True)
class TargetEstimate:
    status: str
    detection: Detection | None
    local_offset: LocalOffset | None = None
    global_position: GlobalPosition | None = None
    distance_m: float | None = None
    aircraft_point: PixelPoint | None = None
    message: str = ""
    reachable: bool = False
