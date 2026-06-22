import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from field_coordinate.cli import ensure_camera_properties, main
from field_coordinate.config import FieldConfig, GeometryMode, HsvRange, load_config
from field_coordinate.vision import detection_from_contour, filter_frame, select_largest_contour


def test_load_config_accepts_resolution_preset_and_hsv_ranges(tmp_path: Path) -> None:
    config_path = tmp_path / "config.json"
    config_path.write_text(
        json.dumps(
            {
                "resolution_preset": "LEGACY_1440_1080",
                "geometry_mode": "LEGACY",
                "hsv_ranges": [{"lower": [135, 67, 62], "upper": [177, 255, 255]}],
                "focal_px": 980.0,
            }
        ),
        encoding="utf-8",
    )

    config = load_config(config_path)

    assert config.resolution_preset == "LEGACY_1440_1080"
    assert config.geometry == GeometryMode.LEGACY
    assert config.resolution.width == 1440
    assert config.hsv_ranges[0].lower == (135, 67, 62)


def test_load_config_rejects_unsafe_values(tmp_path: Path) -> None:
    config_path = tmp_path / "bad.json"
    config_path.write_text(
        json.dumps(
            {
                "hsv_ranges": [],
                "target_real_size_m": -0.1,
                "camera_codec": "TOOLONG",
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError):
        load_config(config_path)


@pytest.mark.parametrize(
    "bad_data",
    [
        {"i2c_write_address": 128},
        {"i2c_register": 256},
    ],
)
def test_load_config_rejects_i2c_values_above_protocol_bounds(
    tmp_path: Path,
    bad_data: dict[str, int],
) -> None:
    config_path = tmp_path / "bad_i2c.json"
    config_path.write_text(json.dumps(bad_data), encoding="utf-8")

    with pytest.raises(ValueError):
        load_config(config_path)


def test_cli_calibrate_prints_updated_config(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    config_path = tmp_path / "config.json"
    config_path.write_text(
        json.dumps(
            {
                "resolution_preset": "HD_720",
                "geometry_mode": "REFINED",
                "focal_px": None,
            }
        ),
        encoding="utf-8",
    )

    exit_code = main(
        [
            "calibrate",
            "--config",
            str(config_path),
            "--known-width-m",
            "0.07",
            "--distance-m",
            "1.05",
            "--observed-width-px",
            "70",
        ]
    )

    payload = json.loads(capsys.readouterr().out)
    assert exit_code == 0
    assert payload["focal_px"] == pytest.approx(1050.0)


def test_project_metadata_uses_vectorsight_name_and_description() -> None:
    pyproject = Path("pyproject.toml").read_text(encoding="utf-8")
    license_text = Path("LICENSE").read_text(encoding="utf-8")

    assert 'name = "vectorsight"' in pyproject
    assert "VectorSight" in pyproject
    assert "field-ready" in pyproject
    assert 'license = { file = "LICENSE" }' in pyproject
    assert "Apache License" in license_text
    assert "Copyright 2026 Cem Berk Çakır" in license_text


def test_cli_replay_prints_existing_jsonl(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    log_path = tmp_path / "session.jsonl"
    log_path.write_text('{"status":"OK"}\n\n{"status":"NO_TARGET"}\n', encoding="utf-8")

    exit_code = main(["replay", "--log", str(log_path)])

    assert exit_code == 0
    assert capsys.readouterr().out.splitlines() == [
        '{"status":"OK"}',
        '{"status":"NO_TARGET"}',
    ]


def test_vision_preserves_extreme_point_center_behavior() -> None:
    contour = np.array(
        [
            [[10, 20]],
            [[30, 20]],
            [[30, 60]],
            [[10, 60]],
        ],
        dtype=np.int32,
    )

    detection = detection_from_contour(contour, area_px=800.0, min_area_px=500.0)

    assert detection is not None
    assert detection.center.x == pytest.approx(20.0)
    assert detection.center.y == pytest.approx(40.0)
    assert detection.observed_width_px == pytest.approx(20.0)
    assert detection.observed_height_px == pytest.approx(40.0)
    assert detection.radius_px == pytest.approx((10.0**2 + 20.0**2) ** 0.5)


def test_filter_and_largest_contour_detects_configured_hsv_target() -> None:
    frame = np.zeros((120, 160, 3), dtype=np.uint8)
    frame[40:80, 50:90] = (0, 0, 255)
    red_config = FieldConfig(
        hsv_ranges=(
            HsvRange(lower=(0, 100, 100), upper=(10, 255, 255)),
        ),
        min_area_px=100.0,
        max_area_px=10_000.0,
        threshold_value=70,
    )

    binary = filter_frame(
        frame,
        red_config.hsv_ranges,
        red_config.blur_kernel,
        red_config.threshold_value,
    )
    contour, area = select_largest_contour(
        binary,
        min_area_px=red_config.min_area_px,
        max_area_px=red_config.max_area_px,
    )

    assert contour is not None
    assert area > 100.0


def test_select_largest_contour_returns_none_when_area_is_outside_bounds() -> None:
    frame = np.zeros((80, 80), dtype=np.uint8)
    cv2.rectangle(frame, (10, 10), (20, 20), 255, -1)

    contour, area = select_largest_contour(
        frame,
        min_area_px=1000.0,
        max_area_px=2000.0,
    )

    assert contour is None
    assert area == pytest.approx(0.0)


class _FakeCapture:
    def __init__(self, width: float, height: float) -> None:
        self.width = width
        self.height = height

    def get(self, prop):
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return self.width
        if prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return self.height
        return 0.0


def test_ensure_camera_properties_rejects_resolution_mismatch() -> None:
    with pytest.raises(RuntimeError):
        ensure_camera_properties(
            _FakeCapture(width=640.0, height=480.0),
            expected_width=1280,
            expected_height=720,
        )


def test_ensure_camera_properties_accepts_matching_resolution() -> None:
    ensure_camera_properties(
        _FakeCapture(width=1280.0, height=720.0),
        expected_width=1280,
        expected_height=720,
    )
