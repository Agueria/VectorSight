from __future__ import annotations

import argparse
import json
from pathlib import Path

from field_coordinate.camera_geometry import calculate_focal_px
from field_coordinate.config import config_to_dict, load_config
from field_coordinate.hardware import create_led_from_config, create_target_reporter_from_config
from field_coordinate.pipeline import append_log, estimate_target
from field_coordinate.telemetry import MavlinkTelemetrySource
from field_coordinate.vision import detect_target, draw_overlay


def ensure_camera_properties(
    capture,
    *,
    expected_width: int,
    expected_height: int,
    tolerance_px: int = 2,
) -> None:
    import cv2

    actual_width = int(round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
    actual_height = int(round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if (
        abs(actual_width - expected_width) > tolerance_px
        or abs(actual_height - expected_height) > tolerance_px
    ):
        raise RuntimeError(
            "Camera resolution mismatch: "
            f"requested {expected_width}x{expected_height}, "
            f"got {actual_width}x{actual_height}"
        )


def calibrate(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    focal_px = calculate_focal_px(
        known_distance_m=args.distance_m,
        real_size_m=args.known_width_m,
        observed_size_px=args.observed_width_px,
    )
    updated = config_to_dict(config)
    updated["focal_px"] = focal_px
    print(json.dumps(updated, indent=2, sort_keys=True))
    return 0


def run(args: argparse.Namespace) -> int:  # pragma: no cover - requires camera and PX4 hardware
    import cv2

    config = load_config(args.config)
    telemetry = MavlinkTelemetrySource(
        ports=config.mavlink_ports,
        baud=config.mavlink_baud,
        stream_rate_hz=config.mavlink_stream_rate_hz,
    )
    telemetry.connect()
    led = create_led_from_config(config)
    reporter = create_target_reporter_from_config(config)

    resolution = config.resolution
    capture = cv2.VideoCapture(config.camera_index)
    if not capture.isOpened():
        raise RuntimeError(f"Could not open camera index {config.camera_index}")

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, resolution.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution.height)
    capture.set(cv2.CAP_PROP_FPS, config.camera_fps)
    if config.camera_codec:
        codec = cv2.VideoWriter_fourcc(*config.camera_codec)
        capture.set(cv2.CAP_PROP_FOURCC, codec)
    ensure_camera_properties(
        capture,
        expected_width=resolution.width,
        expected_height=resolution.height,
    )

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("Camera frame could not be read")

            position, attitude = telemetry.read()
            detection = detect_target(frame, config)
            estimate = estimate_target(detection, position, attitude, config)
            led.set_reachable(estimate.reachable)
            if estimate.status == "OK" and estimate.global_position is not None:
                reporter.report(
                    lat_deg=estimate.global_position.lat_deg,
                    lon_deg=estimate.global_position.lon_deg,
                )
            append_log(config.log_path, estimate)

            if config.preview:
                cv2.imshow("VectorSight", draw_overlay(frame, detection, estimate, config=config))
                if cv2.waitKey(1) == ord("q"):
                    break
    finally:
        capture.release()
        cv2.destroyAllWindows()

    return 0


def replay(args: argparse.Namespace) -> int:
    path = Path(args.log)
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            print(line)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="field_coordinate")
    subparsers = parser.add_subparsers(dest="command", required=True)

    calibrate_parser = subparsers.add_parser("calibrate")
    calibrate_parser.add_argument("--config", required=True)
    calibrate_parser.add_argument("--known-width-m", type=float, required=True)
    calibrate_parser.add_argument("--distance-m", type=float, required=True)
    calibrate_parser.add_argument("--observed-width-px", type=float, required=True)
    calibrate_parser.set_defaults(func=calibrate)

    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("--config", required=True)
    run_parser.set_defaults(func=run)

    replay_parser = subparsers.add_parser("replay")
    replay_parser.add_argument("--log", required=True)
    replay_parser.set_defaults(func=replay)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)
