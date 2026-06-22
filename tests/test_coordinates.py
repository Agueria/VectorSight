import pytest

from field_coordinate.coordinates import local_offset_to_global, rotate_body_offset_to_north_east
from field_coordinate.models import LocalOffset


def test_local_offset_to_global_moves_north_and_east_in_expected_directions() -> None:
    start_lat = 41.0
    start_lon = 29.0

    north = local_offset_to_global(start_lat, start_lon, LocalOffset(north_m=10.0, east_m=0.0))
    east = local_offset_to_global(start_lat, start_lon, LocalOffset(north_m=0.0, east_m=10.0))

    assert north.lat_deg > start_lat
    assert north.lon_deg == pytest.approx(start_lon)
    assert east.lat_deg == pytest.approx(start_lat)
    assert east.lon_deg > start_lon


def test_local_offset_to_global_moves_south_and_west_in_expected_directions() -> None:
    start_lat = 41.0
    start_lon = 29.0

    south_west = local_offset_to_global(
        start_lat,
        start_lon,
        LocalOffset(north_m=-10.0, east_m=-10.0),
    )

    assert south_west.lat_deg < start_lat
    assert south_west.lon_deg < start_lon


def test_rotate_body_offset_preserves_user_yaw_idea_without_quadrant_loss() -> None:
    forward = LocalOffset(north_m=5.0, east_m=0.0)

    yaw_0 = rotate_body_offset_to_north_east(forward, yaw_deg=0.0)
    yaw_90 = rotate_body_offset_to_north_east(forward, yaw_deg=90.0)
    yaw_180 = rotate_body_offset_to_north_east(forward, yaw_deg=180.0)
    yaw_270 = rotate_body_offset_to_north_east(forward, yaw_deg=270.0)

    assert yaw_0.north_m == pytest.approx(5.0)
    assert yaw_0.east_m == pytest.approx(0.0)
    assert yaw_90.north_m == pytest.approx(0.0, abs=1e-9)
    assert yaw_90.east_m == pytest.approx(5.0)
    assert yaw_180.north_m == pytest.approx(-5.0)
    assert yaw_180.east_m == pytest.approx(0.0, abs=1e-9)
    assert yaw_270.north_m == pytest.approx(0.0, abs=1e-9)
    assert yaw_270.east_m == pytest.approx(-5.0)
