from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from field_coordinate.config import FieldConfig
from field_coordinate.debug import dump_cached_messages, request_all_streams, take_message
from field_coordinate.hardware import GpioLed, I2CTargetReporter, NullLed, NullTargetReporter
from field_coordinate.models import (
    Attitude,
    Detection,
    GlobalPosition,
    PixelPoint,
    TargetEstimate,
)
from field_coordinate.pipeline import estimate_target
from field_coordinate.state import AttitudeChangeFilter
from field_coordinate.vision import draw_overlay, is_reachable, shooting_zone


def _detection(x: float, y: float) -> Detection:
    return Detection(
        center=PixelPoint(x=x, y=y),
        observed_width_px=70.0,
        observed_height_px=70.0,
        area_px=5000.0,
        confidence=0.8,
    )


def test_reachability_preserves_center_half_window_for_target_and_aircraft_point() -> None:
    config = FieldConfig(focal_px=980.0)
    zone = shooting_zone(config.resolution)

    assert zone.top_left == PixelPoint(x=320.0, y=180.0)
    assert zone.bottom_right == PixelPoint(x=960.0, y=540.0)
    assert is_reachable(
        target=PixelPoint(x=640.0, y=360.0),
        aircraft=PixelPoint(x=640.0, y=360.0),
        resolution=config.resolution,
    )
    assert not is_reachable(
        target=PixelPoint(x=100.0, y=360.0),
        aircraft=PixelPoint(x=640.0, y=360.0),
        resolution=config.resolution,
    )
    assert not is_reachable(
        target=PixelPoint(x=640.0, y=360.0),
        aircraft=PixelPoint(x=100.0, y=360.0),
        resolution=config.resolution,
    )


def test_pipeline_returns_not_reachable_and_skips_coordinates_outside_shooting_zone() -> None:
    config = FieldConfig(focal_px=980.0)

    estimate = estimate_target(
        _detection(100.0, 360.0),
        GlobalPosition(lat_deg=41.0, lon_deg=29.0, relative_alt_m=20.0),
        Attitude(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0),
        config,
    )

    assert estimate.status == "NOT_REACHABLE"
    assert estimate.local_offset is None
    assert estimate.global_position is None


def test_gpio_led_writes_reachable_state_to_pin() -> None:
    calls = []

    class FakeGPIO:
        BCM = "BCM"
        OUT = "OUT"
        HIGH = 1
        LOW = 0

        def setmode(self, mode):
            calls.append(("setmode", mode))

        def setwarnings(self, enabled):
            calls.append(("setwarnings", enabled))

        def setup(self, pin, mode):
            calls.append(("setup", pin, mode))

        def output(self, pin, value):
            calls.append(("output", pin, value))

    led = GpioLed(pin=21, gpio=FakeGPIO())

    led.set_reachable(True)
    led.set_reachable(False)

    assert ("setup", 21, "OUT") in calls
    assert ("output", 21, 1) in calls
    assert ("output", 21, 0) in calls


def test_i2c_target_reporter_writes_scaled_coordinates_and_pin() -> None:
    bus_writes = []
    gpio_calls = []

    class FakeBus:
        def write_i2c_block_data(self, address, register, data):
            bus_writes.append((address, register, data))

    class FakeGPIO:
        BCM = "BCM"
        OUT = "OUT"
        HIGH = 1

        def setmode(self, mode):
            gpio_calls.append(("setmode", mode))

        def setup(self, pin, mode):
            gpio_calls.append(("setup", pin, mode))

        def output(self, pin, value):
            gpio_calls.append(("output", pin, value))

    reporter = I2CTargetReporter(bus=FakeBus(), gpio=FakeGPIO(), gpio_pin=3)

    reporter.report(lat_deg=41.1234567, lon_deg=29.7654321)

    assert bus_writes
    address, register, data = bus_writes[0]
    assert address == 80
    assert register == 0
    assert len(data) == 8
    assert ("output", 3, 1) in gpio_calls


def test_null_hardware_adapters_are_safe_noops() -> None:
    NullLed().set_reachable(True)
    NullTargetReporter().report(lat_deg=41.0, lon_deg=29.0)


def test_overlay_draws_crosshair_shooting_zone_diagonals_line_and_status_text() -> None:
    config = FieldConfig(focal_px=980.0)
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    detection = _detection(740.0, 360.0)
    estimate = TargetEstimate(
        status="OK",
        detection=detection,
        distance_m=0.98,
        reachable=True,
        aircraft_point=PixelPoint(x=640.0, y=360.0),
    )

    output = draw_overlay(
        frame,
        detection,
        estimate,
        config=config,
    )

    assert output.sum() > frame.sum()
    assert output[180, 320].any()
    assert output[360, 640].any()
    assert output[360, 690, 1] > 0


def test_debug_helpers_request_stream_take_message_and_dump_cache() -> None:
    requests = []

    class FakeMav:
        def request_data_stream_send(self, target_system, target_component, stream_id, rate, start):
            requests.append((target_system, target_component, stream_id, rate, start))

    class FakeConnection:
        target_system = 1
        target_component = 2
        mav = FakeMav()
        messages = {"ATTITUDE": SimpleNamespace(roll=1)}

        def recv_match(self, type=None, blocking=True, timeout=None):
            return {"type": type, "blocking": blocking, "timeout": timeout}

    connection = FakeConnection()

    request_all_streams(connection, rate_hz=30)
    message = take_message(connection, msg_type="ATTITUDE", blocking=False, timeout_s=0.5)
    dumped = dump_cached_messages(connection)

    assert requests[0][3] == 30
    assert message["type"] == "ATTITUDE"
    assert "ATTITUDE" in dumped


def test_attitude_change_filter_keeps_last_values_and_reports_changes() -> None:
    change_filter = AttitudeChangeFilter(max_history=3)

    first = change_filter.update(roll_deg=1.0, pitch_deg=2.0, yaw_deg=3.0)
    same = change_filter.update(roll_deg=1.0, pitch_deg=2.0, yaw_deg=3.0)
    changed = change_filter.update(roll_deg=1.0, pitch_deg=2.5, yaw_deg=3.0)

    assert first.is_new
    assert not same.is_new
    assert changed.is_new
    assert len(change_filter.history) <= 3
