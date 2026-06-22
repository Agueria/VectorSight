from __future__ import annotations

from typing import Any


def _mav_data_stream_all() -> int:
    try:
        from pymavlink import mavutil

        return int(mavutil.mavlink.MAV_DATA_STREAM_ALL)
    except ImportError:
        return 0


def request_all_streams(connection: Any, *, rate_hz: int) -> None:
    connection.mav.request_data_stream_send(
        connection.target_system,
        connection.target_component,
        _mav_data_stream_all(),
        rate_hz,
        1,
    )


def take_message(
    connection: Any,
    *,
    msg_type: str | None = None,
    blocking: bool = True,
    timeout_s: float | None = None,
) -> Any:
    return connection.recv_match(type=msg_type, blocking=blocking, timeout=timeout_s)


def dump_cached_messages(connection: Any) -> dict[str, Any]:
    return dict(getattr(connection, "messages", {}))
