from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any

from field_coordinate.models import Attitude, GlobalPosition
from field_coordinate.state import AttitudeChangeFilter


def _require_attr(message: Any, name: str) -> Any:
    if not hasattr(message, name):
        raise ValueError(f"MAVLink message missing field: {name}")
    return getattr(message, name)


def normalize_attitude(message: Any) -> Attitude:
    return Attitude(
        roll_deg=math.degrees(float(_require_attr(message, "roll"))),
        pitch_deg=math.degrees(float(_require_attr(message, "pitch"))),
        yaw_deg=math.degrees(float(_require_attr(message, "yaw"))),
    )


def normalize_global_position(message: Any) -> GlobalPosition:
    return GlobalPosition(
        lat_deg=float(_require_attr(message, "lat")) / 10_000_000.0,
        lon_deg=float(_require_attr(message, "lon")) / 10_000_000.0,
        relative_alt_m=float(_require_attr(message, "relative_alt")) / 1000.0,
    )


@dataclass
class LatestTelemetryCache:
    max_age_s: float = 1.0
    attitude_filter: AttitudeChangeFilter | None = None
    _position: GlobalPosition | None = None
    _position_time_s: float | None = None
    _attitude: Attitude | None = None
    _attitude_time_s: float | None = None
    _last_attitude_change_is_new: bool | None = None

    def update_position(self, message: Any, *, now_s: float | None = None) -> None:
        timestamp = time.monotonic() if now_s is None else now_s
        self._position = normalize_global_position(message)
        self._position_time_s = timestamp

    def update_attitude(self, message: Any, *, now_s: float | None = None) -> None:
        timestamp = time.monotonic() if now_s is None else now_s
        attitude = normalize_attitude(message)
        if self.attitude_filter is not None:
            change = self.attitude_filter.update(
                roll_deg=attitude.roll_deg,
                pitch_deg=attitude.pitch_deg,
                yaw_deg=attitude.yaw_deg,
            )
            attitude = change.attitude
            self._last_attitude_change_is_new = change.is_new
        self._attitude = attitude
        self._attitude_time_s = timestamp

    @property
    def last_attitude_change_is_new(self) -> bool | None:
        return self._last_attitude_change_is_new

    def snapshot(self, *, now_s: float | None = None) -> tuple[GlobalPosition | None, Attitude | None]:
        timestamp = time.monotonic() if now_s is None else now_s
        position = self._position
        attitude = self._attitude

        if self._position_time_s is None or timestamp - self._position_time_s > self.max_age_s:
            position = None
        if self._attitude_time_s is None or timestamp - self._attitude_time_s > self.max_age_s:
            attitude = None

        return position, attitude


class MavlinkTelemetrySource:  # pragma: no cover - requires live MAVLink hardware
    def __init__(self, ports: tuple[str, ...], baud: int, stream_rate_hz: int) -> None:
        self._ports = ports
        self._baud = baud
        self._stream_rate_hz = stream_rate_hz
        self._connection = None
        self._attitude_filter = AttitudeChangeFilter(max_history=3)
        self._cache = LatestTelemetryCache(
            max_age_s=1.0,
            attitude_filter=self._attitude_filter,
        )

    def connect(self) -> None:
        from pymavlink import mavutil

        last_error: Exception | None = None
        for port in self._ports:
            try:
                self._connection = mavutil.mavlink_connection(port, baud=self._baud)
                self._connection.wait_heartbeat(timeout=10)
                self._connection.mav.request_data_stream_send(
                    self._connection.target_system,
                    self._connection.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    self._stream_rate_hz,
                    1,
                )
                return
            except Exception as exc:
                last_error = exc
                self._connection = None
        raise ConnectionError(f"Could not connect to MAVLink ports: {self._ports}") from last_error

    def read(self) -> tuple[GlobalPosition | None, Attitude | None]:
        if self._connection is None:
            raise RuntimeError("MAVLink source is not connected")

        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            message = self._connection.recv_match(blocking=False)
            if message is None:
                continue
            message_type = message.get_type()
            if message_type == "GLOBAL_POSITION_INT":
                self._cache.update_position(message)
            elif message_type == "ATTITUDE":
                self._cache.update_attitude(message)
        return self._cache.snapshot()
