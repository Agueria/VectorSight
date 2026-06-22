from __future__ import annotations

import math

from field_coordinate.config import FieldConfig, HsvRange
from field_coordinate.models import Detection, PixelPoint, PixelZone, Resolution, TargetEstimate


def _import_cv2_numpy():
    import cv2
    import numpy as np

    return cv2, np


def filter_frame(frame, hsv_ranges: tuple[HsvRange, ...], blur_kernel: int, threshold_value: int):
    cv2, np = _import_cv2_numpy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = None

    for hsv_range in hsv_ranges:
        lower = np.array(hsv_range.lower)
        upper = np.array(hsv_range.upper)
        current = cv2.inRange(hsv, lower, upper)
        mask = current if mask is None else cv2.bitwise_or(mask, current)

    blur_size = max(1, int(blur_kernel))
    blurred = cv2.blur(frame, (blur_size, blur_size))
    selected = cv2.bitwise_and(blurred, blurred, mask=mask)
    _, thresholded = cv2.threshold(selected, threshold_value, 255, cv2.THRESH_BINARY)
    return cv2.cvtColor(thresholded, cv2.COLOR_BGR2GRAY)


def select_largest_contour(binary_image, *, min_area_px: float, max_area_px: float):
    cv2, _ = _import_cv2_numpy()
    contours, _ = cv2.findContours(binary_image, cv2.RETR_TREE, 1)
    selected = None
    selected_area = 0.0

    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area > selected_area and min_area_px <= area <= max_area_px:
            selected = contour
            selected_area = area
    return selected, selected_area


def detection_from_contour(contour, area_px: float, *, min_area_px: float) -> Detection | None:
    if contour is None:
        return None
    points = [item[0] for item in contour]
    if not points:
        return None

    max_y = max(points, key=lambda point: point[1])
    min_y = min(points, key=lambda point: point[1])
    max_x = max(points, key=lambda point: point[0])
    min_x = min(points, key=lambda point: point[0])

    y_center = PixelPoint(
        x=(float(min_y[0]) + float(max_y[0])) / 2.0,
        y=(float(min_y[1]) + float(max_y[1])) / 2.0,
    )
    x_center = PixelPoint(
        x=(float(min_x[0]) + float(max_x[0])) / 2.0,
        y=(float(min_x[1]) + float(max_x[1])) / 2.0,
    )

    y_radius = math.hypot(float(max_y[1]) - y_center.y, float(max_y[0]) - y_center.x)
    x_radius = math.hypot(float(max_x[1]) - x_center.y, float(max_x[0]) - x_center.x)
    center = x_center if x_radius >= y_radius else y_center
    radius = max(x_radius, y_radius)

    observed_width = float(max_x[0]) - float(min_x[0])
    observed_height = float(max_y[1]) - float(min_y[1])
    if observed_width <= 0 or observed_height <= 0:
        return None

    confidence = min(1.0, area_px / max(min_area_px, 1.0))
    return Detection(
        center=center,
        observed_width_px=observed_width,
        observed_height_px=observed_height,
        area_px=area_px,
        confidence=confidence,
        radius_px=radius,
    )


def detect_target(frame, config: FieldConfig) -> Detection | None:
    binary = filter_frame(
        frame,
        config.hsv_ranges,
        config.blur_kernel,
        config.threshold_value,
    )
    contour, area = select_largest_contour(
        binary,
        min_area_px=config.min_area_px,
        max_area_px=config.max_area_px,
    )
    return detection_from_contour(contour, area, min_area_px=config.min_area_px)


def shooting_zone(resolution: Resolution) -> PixelZone:
    return PixelZone(
        top_left=PixelPoint(x=resolution.width * 0.25, y=resolution.height * 0.25),
        bottom_right=PixelPoint(x=resolution.width * 0.75, y=resolution.height * 0.75),
    )


def point_in_zone(point: PixelPoint, zone: PixelZone) -> bool:
    return (
        zone.top_left.x <= point.x <= zone.bottom_right.x
        and zone.top_left.y <= point.y <= zone.bottom_right.y
    )


def is_reachable(*, target: PixelPoint, aircraft: PixelPoint, resolution: Resolution) -> bool:
    zone = shooting_zone(resolution)
    return point_in_zone(target, zone) and point_in_zone(aircraft, zone)


def draw_overlay(
    frame,
    detection: Detection | None,
    estimate: TargetEstimate,
    *,
    config: FieldConfig | None = None,
    aircraft_point: PixelPoint | None = None,
):
    cv2, _ = _import_cv2_numpy()
    output = frame.copy()
    height, width = output.shape[:2]
    resolution = config.resolution if config is not None else Resolution(width=width, height=height)
    zone = shooting_zone(resolution)
    frame_center = (int(resolution.width / 2), int(resolution.height / 2))
    line_origin = aircraft_point if aircraft_point is not None else estimate.aircraft_point

    cv2.line(
        output,
        (frame_center[0], frame_center[1] - 15),
        (frame_center[0], frame_center[1] + 15),
        (0, 0, 0),
        2,
    )
    cv2.line(
        output,
        (frame_center[0] - 15, frame_center[1]),
        (frame_center[0] + 15, frame_center[1]),
        (0, 0, 0),
        2,
    )
    cv2.rectangle(
        output,
        (int(zone.top_left.x), int(zone.top_left.y)),
        (int(zone.bottom_right.x), int(zone.bottom_right.y)),
        (0, 0, 255),
        2,
    )
    cv2.putText(
        output,
        "Shooting Zone",
        (int(zone.top_left.x) + 5, int(zone.bottom_right.y) - 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        1,
        cv2.LINE_AA,
    )

    if detection is not None:
        target_center = (int(detection.center.x), int(detection.center.y))
        cv2.circle(output, center=target_center, radius=6, color=(255, 0, 0), thickness=-1)
        if detection.radius_px is not None:
            cv2.circle(
                output,
                center=target_center,
                radius=int(detection.radius_px),
                color=(255, 0, 0),
                thickness=2,
            )
        left = int(detection.center.x - detection.observed_width_px / 2.0)
        right = int(detection.center.x + detection.observed_width_px / 2.0)
        top = int(detection.center.y - detection.observed_height_px / 2.0)
        bottom = int(detection.center.y + detection.observed_height_px / 2.0)
        cv2.line(output, (left, top), (right, bottom), (255, 0, 0), 2)
        cv2.line(output, (right, top), (left, bottom), (255, 0, 0), 2)
        if line_origin is not None and estimate.reachable:
            cv2.line(
                output,
                (int(line_origin.x), int(line_origin.y)),
                target_center,
                (0, 255, 0),
                2,
            )

    text = f"status={estimate.status}"
    if estimate.distance_m is not None:
        text += f" dist={estimate.distance_m:.2f}m"
    if estimate.global_position is not None:
        text += (
            f" lat={estimate.global_position.lat_deg:.7f}"
            f" lon={estimate.global_position.lon_deg:.7f}"
        )

    cv2.putText(
        output,
        text,
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return output
