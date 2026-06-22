from __future__ import annotations

import json
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any

from field_coordinate.models import PixelPoint, Resolution


class ResolutionPreset(Enum):
    HD_720 = "HD_720"
    FHD_1080 = "FHD_1080"
    LEGACY_1440_1080 = "LEGACY_1440_1080"
    CUSTOM = "CUSTOM"


class GeometryMode(Enum):
    LEGACY = "LEGACY"
    REFINED = "REFINED"


@dataclass(frozen=True)
class HsvRange:
    lower: tuple[int, int, int]
    upper: tuple[int, int, int]


def _enum_value(enum_type: type[Enum], value: Enum | str) -> Enum:
    if isinstance(value, enum_type):
        return value
    try:
        return enum_type[str(value)]
    except KeyError:
        return enum_type(str(value))


def resolve_resolution(
    preset: ResolutionPreset | str,
    *,
    custom_width: int | None = None,
    custom_height: int | None = None,
) -> Resolution:
    selected = _enum_value(ResolutionPreset, preset)

    if selected == ResolutionPreset.HD_720:
        return Resolution(width=1280, height=720)
    if selected == ResolutionPreset.FHD_1080:
        return Resolution(width=1920, height=1080)
    if selected == ResolutionPreset.LEGACY_1440_1080:
        return Resolution(width=1440, height=1080)
    if custom_width is None or custom_height is None:
        raise ValueError("CUSTOM resolution requires custom_width and custom_height")
    if custom_width <= 0 or custom_height <= 0:
        raise ValueError("Resolution width and height must be positive")
    return Resolution(width=custom_width, height=custom_height)


@dataclass(frozen=True)
class FieldConfig:
    resolution_preset: ResolutionPreset | str = ResolutionPreset.HD_720
    geometry_mode: GeometryMode | str = GeometryMode.REFINED
    custom_width: int | None = None
    custom_height: int | None = None
    camera_index: int = 0
    camera_fps: int = 30
    camera_codec: str = "MJPG"
    horizontal_fov_deg: float = 62.2
    vertical_fov_deg: float = 48.8
    target_real_size_m: float = 0.07
    focal_px: float | None = None
    hsv_ranges: tuple[HsvRange, ...] = field(
        default_factory=lambda: (
            HsvRange(lower=(0, 122, 161), upper=(53, 255, 255)),
        )
    )
    threshold_value: int = 70
    blur_kernel: int = 5
    min_area_px: float = 5000.0
    max_area_px: float = 1500000.0
    mavlink_ports: tuple[str, ...] = (
        "/dev/tty.usbmodem1201",
        "/dev/ttyACM0",
        "/dev/ttyACM1",
    )
    mavlink_baud: int = 115200
    mavlink_stream_rate_hz: int = 10
    log_path: str = "logs/session.jsonl"
    preview: bool = True
    gpio_enabled: bool = False
    reachable_led_pin: int = 21
    found_gpio_pin: int = 3
    i2c_enabled: bool = False
    i2c_bus: int = 0
    i2c_write_address: int = 80
    i2c_register: int = 0

    def __post_init__(self) -> None:
        if self.camera_fps <= 0:
            raise ValueError("camera_fps must be positive")
        if len(self.camera_codec) not in (0, 4):
            raise ValueError("camera_codec must be empty or four characters")
        if self.horizontal_fov_deg <= 0 or self.vertical_fov_deg <= 0:
            raise ValueError("FOV values must be positive")
        if self.target_real_size_m <= 0:
            raise ValueError("target_real_size_m must be positive")
        if self.focal_px is not None and self.focal_px <= 0:
            raise ValueError("focal_px must be positive when provided")
        if not self.hsv_ranges:
            raise ValueError("At least one HSV range is required")
        for hsv_range in self.hsv_ranges:
            for value in (*hsv_range.lower, *hsv_range.upper):
                if value < 0 or value > 255:
                    raise ValueError("HSV channel values must be between 0 and 255")
        if self.threshold_value < 0 or self.threshold_value > 255:
            raise ValueError("threshold_value must be between 0 and 255")
        if self.blur_kernel <= 0:
            raise ValueError("blur_kernel must be positive")
        if self.min_area_px <= 0 or self.max_area_px <= 0:
            raise ValueError("Contour area bounds must be positive")
        if self.min_area_px > self.max_area_px:
            raise ValueError("min_area_px cannot be greater than max_area_px")
        if not self.mavlink_ports:
            raise ValueError("At least one MAVLink port is required")
        if self.mavlink_baud <= 0 or self.mavlink_stream_rate_hz <= 0:
            raise ValueError("MAVLink baud and stream rate must be positive")
        if self.reachable_led_pin < 0 or self.found_gpio_pin < 0:
            raise ValueError("GPIO pins must be nonnegative")
        if self.i2c_bus < 0 or self.i2c_write_address < 0 or self.i2c_register < 0:
            raise ValueError("I2C settings must be nonnegative")
        if self.i2c_write_address > 0x7F:
            raise ValueError("i2c_write_address must be between 0 and 127")
        if self.i2c_register > 0xFF:
            raise ValueError("i2c_register must be between 0 and 255")

    @property
    def resolution(self) -> Resolution:
        return resolve_resolution(
            self.resolution_preset,
            custom_width=self.custom_width,
            custom_height=self.custom_height,
        )

    @property
    def geometry(self) -> GeometryMode:
        return _enum_value(GeometryMode, self.geometry_mode)

    @property
    def origin(self) -> PixelPoint:
        resolution = self.resolution
        return PixelPoint(x=resolution.width / 2.0, y=resolution.height / 2.0)


def _hsv_range_from_raw(raw: dict[str, Any]) -> HsvRange:
    lower = tuple(int(value) for value in raw["lower"])
    upper = tuple(int(value) for value in raw["upper"])
    if len(lower) != 3 or len(upper) != 3:
        raise ValueError("HSV ranges must have three channels")
    return HsvRange(lower=lower, upper=upper)


def load_config(path: str | Path) -> FieldConfig:
    config_path = Path(path)
    data = json.loads(config_path.read_text(encoding="utf-8"))

    if "hsv_ranges" in data:
        data["hsv_ranges"] = tuple(_hsv_range_from_raw(item) for item in data["hsv_ranges"])
    if "mavlink_ports" in data:
        data["mavlink_ports"] = tuple(str(port) for port in data["mavlink_ports"])

    return FieldConfig(**data)


def config_to_dict(config: FieldConfig) -> dict[str, Any]:
    data = {
        "resolution_preset": _enum_value(ResolutionPreset, config.resolution_preset).value,
        "geometry_mode": config.geometry.value,
        "custom_width": config.custom_width,
        "custom_height": config.custom_height,
        "camera_index": config.camera_index,
        "camera_fps": config.camera_fps,
        "camera_codec": config.camera_codec,
        "horizontal_fov_deg": config.horizontal_fov_deg,
        "vertical_fov_deg": config.vertical_fov_deg,
        "target_real_size_m": config.target_real_size_m,
        "focal_px": config.focal_px,
        "hsv_ranges": [
            {"lower": list(item.lower), "upper": list(item.upper)}
            for item in config.hsv_ranges
        ],
        "threshold_value": config.threshold_value,
        "blur_kernel": config.blur_kernel,
        "min_area_px": config.min_area_px,
        "max_area_px": config.max_area_px,
        "mavlink_ports": list(config.mavlink_ports),
        "mavlink_baud": config.mavlink_baud,
        "mavlink_stream_rate_hz": config.mavlink_stream_rate_hz,
        "log_path": config.log_path,
        "preview": config.preview,
        "gpio_enabled": config.gpio_enabled,
        "reachable_led_pin": config.reachable_led_pin,
        "found_gpio_pin": config.found_gpio_pin,
        "i2c_enabled": config.i2c_enabled,
        "i2c_bus": config.i2c_bus,
        "i2c_write_address": config.i2c_write_address,
        "i2c_register": config.i2c_register,
    }
    return {key: value for key, value in data.items() if value is not None}
