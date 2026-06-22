import math

import pytest

from field_coordinate.camera_geometry import (
    calculate_distance_m,
    calculate_focal_px,
    pixel_scale_m,
    projected_aircraft_point,
    relative_target_offset_m,
)
from field_coordinate.config import GeometryMode, ResolutionPreset, resolve_resolution
from field_coordinate.models import PixelPoint


def test_focal_length_matches_original_formula() -> None:
    assert calculate_focal_px(
        known_distance_m=1.05,
        real_size_m=0.07,
        observed_size_px=70.0,
    ) == pytest.approx(1050.0)


def test_distance_matches_original_formula() -> None:
    assert calculate_distance_m(
        focal_px=980.0,
        real_size_m=0.07,
        observed_size_px=70.0,
    ) == pytest.approx(0.98)


def test_geometry_rejects_zero_or_negative_values() -> None:
    with pytest.raises(ValueError):
        calculate_focal_px(
            known_distance_m=1.05,
            real_size_m=0.0,
            observed_size_px=70.0,
        )

    with pytest.raises(ValueError):
        calculate_distance_m(
            focal_px=980.0,
            real_size_m=0.07,
            observed_size_px=0.0,
        )


def test_resolution_presets_include_legacy_and_hd_modes() -> None:
    assert resolve_resolution(ResolutionPreset.HD_720).width == 1280
    assert resolve_resolution(ResolutionPreset.HD_720).height == 720
    assert resolve_resolution(ResolutionPreset.LEGACY_1440_1080).width == 1440
    assert resolve_resolution(ResolutionPreset.LEGACY_1440_1080).height == 1080
    assert resolve_resolution(ResolutionPreset.FHD_1080).width == 1920
    assert resolve_resolution(ResolutionPreset.FHD_1080).height == 1080


def test_custom_resolution_requires_width_and_height() -> None:
    custom = resolve_resolution(
        ResolutionPreset.CUSTOM,
        custom_width=1024,
        custom_height=768,
    )

    assert custom.width == 1024
    assert custom.height == 768

    with pytest.raises(ValueError):
        resolve_resolution(ResolutionPreset.CUSTOM)


def test_legacy_pixel_scale_preserves_original_fov_divided_by_four_formula() -> None:
    scale = pixel_scale_m(
        altitude_m=20.0,
        horizontal_fov_deg=62.2,
        vertical_fov_deg=48.8,
        resolution=resolve_resolution(ResolutionPreset.HD_720),
        mode=GeometryMode.LEGACY,
    )

    expected_x_m = (math.tan(math.radians(62.2) / 4.0) * 20.0) / (1280 / 4.0)
    expected_y_m = (math.tan(math.radians(48.8) / 4.0) * 20.0) / (720 / 4.0)
    assert scale.x_m_per_px == pytest.approx(expected_x_m)
    assert scale.y_m_per_px == pytest.approx(expected_y_m)


def test_refined_pixel_scale_keeps_user_fov_idea_but_uses_full_frame_span() -> None:
    refined = pixel_scale_m(
        altitude_m=20.0,
        horizontal_fov_deg=62.2,
        vertical_fov_deg=48.8,
        resolution=resolve_resolution(ResolutionPreset.HD_720),
        mode=GeometryMode.REFINED,
    )
    legacy = pixel_scale_m(
        altitude_m=20.0,
        horizontal_fov_deg=62.2,
        vertical_fov_deg=48.8,
        resolution=resolve_resolution(ResolutionPreset.HD_720),
        mode=GeometryMode.LEGACY,
    )

    assert refined.x_m_per_px > legacy.x_m_per_px
    assert refined.y_m_per_px > legacy.y_m_per_px


def test_relative_target_offset_uses_selected_resolution_center() -> None:
    resolution = resolve_resolution(ResolutionPreset.HD_720)
    scale = pixel_scale_m(
        altitude_m=20.0,
        horizontal_fov_deg=62.2,
        vertical_fov_deg=48.8,
        resolution=resolution,
        mode=GeometryMode.LEGACY,
    )

    offset = relative_target_offset_m(
        target=PixelPoint(x=740.0, y=320.0),
        origin=PixelPoint(x=640.0, y=360.0),
        scale=scale,
    )

    assert offset.east_m == pytest.approx(100.0 * scale.x_m_per_px)
    assert offset.north_m == pytest.approx(40.0 * scale.y_m_per_px)


def test_projected_aircraft_point_preserves_nerdeyim_formula() -> None:
    resolution = resolve_resolution(ResolutionPreset.HD_720)

    point = projected_aircraft_point(
        roll_deg=10.0,
        pitch_deg=-5.0,
        resolution=resolution,
        horizontal_fov_deg=62.2,
        vertical_fov_deg=48.8,
    )

    assert point.x == pytest.approx(640.0 + int(10.0 * (1280 / 62.2)))
    assert point.y == pytest.approx(360.0 + int(-5.0 * (720 / 48.8)))
