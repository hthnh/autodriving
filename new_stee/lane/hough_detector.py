"""Stage 2 lane detection using Canny edges and probabilistic Hough lines."""

import math
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from .base import BaseLaneDetector, LaneResult


Segment = Tuple[int, int, int, int]


class HoughLaneDetector(BaseLaneDetector):
    """Estimate lane center and heading from fitted edge-line segments."""

    stage_id = "stage_2_canny_hough"
    display_name = "Stage 2 — Canny + Hough"
    DEFAULT_SETTINGS = {
        "roi_y_start": 260,
        "roi_y_end": 480,
        "blur_kernel": 5,
        "canny_low": 50,
        "canny_high": 150,
        "hough_rho": 1.0,
        "hough_theta_deg": 1.0,
        "hough_threshold": 30,
        "min_line_length": 30,
        "max_line_gap": 20,
        "slope_min_abs": 0.35,
        "slope_max_abs": 5.0,
        "lane_width_px": 220,
        "center_kp": 0.003,
        "heading_kp": 0.015,
        "steer_sign": 1,
        "base_speed": 0.10,
        "min_confidence": 0.15,
        "lost_lane_timeout": 0.3,
        "max_steer_norm": 0.6,
    }

    def __init__(self, settings: Optional[dict] = None, **overrides) -> None:
        self.settings = dict(self.DEFAULT_SETTINGS)
        initial = dict(settings or {})
        initial.update(overrides)
        self.update_settings(initial)
        self._last_roi_bounds = (
            self.settings["roi_y_start"], self.settings["roi_y_end"]
        )
        self._last_segments: List[Tuple[Segment, str]] = []
        self._last_fits: Dict[str, Tuple[int, int, int, int]] = {}
        self._last_center_points: Optional[Tuple[int, int, int, int]] = None

    def update_settings(self, settings: dict) -> None:
        """Validate and atomically apply Stage 2 settings."""
        unknown = set(settings) - set(self.DEFAULT_SETTINGS)
        if unknown:
            raise ValueError("unknown Stage 2 setting(s): " + ", ".join(sorted(unknown)))
        values = dict(self.settings)
        values.update(settings)
        integer_names = (
            "roi_y_start", "roi_y_end", "blur_kernel", "canny_low",
            "canny_high", "hough_threshold", "min_line_length",
            "max_line_gap", "lane_width_px",
        )
        for name in integer_names:
            value = values[name]
            if isinstance(value, bool) or float(value) != int(float(value)):
                raise ValueError(f"{name} must be an integer")
            values[name] = int(float(value))
        float_names = (
            "hough_rho", "hough_theta_deg", "slope_min_abs",
            "slope_max_abs", "center_kp", "heading_kp", "base_speed",
            "min_confidence", "lost_lane_timeout", "max_steer_norm",
        )
        for name in float_names:
            if isinstance(values[name], bool):
                raise ValueError(f"{name} must be numeric")
            values[name] = float(values[name])
        values["steer_sign"] = int(values["steer_sign"])

        if values["roi_y_start"] < 0 or values["roi_y_end"] <= values["roi_y_start"]:
            raise ValueError("ROI requires 0 <= roi_y_start < roi_y_end")
        if values["blur_kernel"] <= 0 or values["blur_kernel"] % 2 == 0:
            raise ValueError("blur_kernel must be a positive odd integer")
        if not 0 <= values["canny_low"] < values["canny_high"] <= 255:
            raise ValueError("Canny thresholds require 0 <= low < high <= 255")
        for name in (
            "hough_rho", "hough_theta_deg", "min_line_length",
            "lane_width_px", "slope_min_abs", "slope_max_abs",
            "lost_lane_timeout",
        ):
            if values[name] <= 0:
                raise ValueError(f"{name} must be positive")
        if values["hough_threshold"] <= 0 or values["max_line_gap"] < 0:
            raise ValueError("Hough threshold must be positive and line gap nonnegative")
        if values["slope_max_abs"] < values["slope_min_abs"]:
            raise ValueError("slope_max_abs must be >= slope_min_abs")
        if values["center_kp"] < 0 or values["heading_kp"] < 0:
            raise ValueError("steering gains cannot be negative")
        if values["steer_sign"] not in (-1, 1):
            raise ValueError("steer_sign must be -1 or 1")
        for name in ("base_speed", "min_confidence", "max_steer_norm"):
            if not 0 <= values[name] <= 1:
                raise ValueError(f"{name} must be between 0 and 1")
        self.settings = values
        for name, value in values.items():
            setattr(self, name, value)

    def detect(self, frame_rgb: np.ndarray) -> LaneResult:
        """Run grayscale, Canny, Hough, fitting, and steering estimation."""
        if not isinstance(frame_rgb, np.ndarray) or frame_rgb.ndim != 3:
            raise ValueError("frame_rgb must be an HxWxC numpy array")
        if frame_rgb.shape[2] < 3:
            raise ValueError("frame_rgb must contain at least three channels")
        frame_height, frame_width = frame_rgb.shape[:2]
        image_center_x = frame_width // 2
        y_start = min(self.roi_y_start, frame_height)
        y_end = min(self.roi_y_end, frame_height)
        self._last_roi_bounds = (y_start, y_end)
        self._last_segments = []
        self._last_fits = {}
        self._last_center_points = None
        if y_end <= y_start or frame_width == 0:
            return self._lost(image_center_x)
        roi = frame_rgb[y_start:y_end, :, :3]

        # Stage 2 deliberately ignores color and detects geometric edges.
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(
            gray, (self.blur_kernel, self.blur_kernel), 0
        )
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        raw_lines = cv2.HoughLinesP(
            edges,
            self.hough_rho,
            math.radians(self.hough_theta_deg),
            self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap,
        )

        left: List[Segment] = []
        right: List[Segment] = []
        total_length = 0.0
        if raw_lines is not None:
            for raw in raw_lines[:, 0]:
                x1, y1, x2, y2 = (int(value) for value in raw)
                dx, dy = x2 - x1, y2 - y1
                if dx == 0:
                    self._last_segments.append(((x1, y1, x2, y2), "rejected"))
                    continue
                slope = dy / dx
                if not self.slope_min_abs <= abs(slope) <= self.slope_max_abs:
                    self._last_segments.append(((x1, y1, x2, y2), "rejected"))
                    continue
                segment = (x1, y1, x2, y2)
                side = "left" if slope < 0 else "right"
                (left if side == "left" else right).append(segment)
                self._last_segments.append((segment, side))
                total_length += math.hypot(dx, dy)

        left_fit = self._fit_side(left, roi.shape[0])
        right_fit = self._fit_side(right, roi.shape[0])
        if left_fit:
            self._last_fits["left"] = left_fit
        if right_fit:
            self._last_fits["right"] = right_fit
        if not left_fit and not right_fit:
            return self._lost(
                image_center_x,
                left_count=len(left),
                right_count=len(right),
            )

        bottom_y = roi.shape[0] - 1
        mid_y = roi.shape[0] // 2
        if left_fit and right_fit:
            bottom_x = (left_fit[0] + right_fit[0]) / 2.0
            mid_x = (left_fit[2] + right_fit[2]) / 2.0
            base_confidence = 0.8
        elif left_fit:
            bottom_x = left_fit[0] + self.lane_width_px / 2.0
            mid_x = left_fit[2] + self.lane_width_px / 2.0
            base_confidence = 0.45
        else:
            bottom_x = right_fit[0] - self.lane_width_px / 2.0
            mid_x = right_fit[2] - self.lane_width_px / 2.0
            base_confidence = 0.45

        lane_center_x = int(round(bottom_x))
        center_error = float(lane_center_x - image_center_x)
        forward_pixels = max(1, bottom_y - mid_y)
        # Looking from the ROI bottom toward its midpoint, positive horizontal
        # displacement means the lane heads right relative to image vertical.
        heading_error = math.degrees(
            math.atan2(mid_x - bottom_x, forward_pixels)
        )
        steer = self.steer_sign * (
            self.center_kp * center_error
            + self.heading_kp * heading_error
        )
        steer = max(-1.0, min(1.0, steer))

        total_count = len(left) + len(right)
        count_bonus = min(0.15, total_count * 0.015)
        length_bonus = min(
            0.10, total_length / max(1.0, roi.shape[0] * 4.0) * 0.10
        )
        center_penalty = min(
            0.25,
            abs(center_error) / max(1.0, frame_width / 2.0) * 0.25,
        )
        confidence = max(
            0.0,
            min(1.0, base_confidence + count_bonus + length_bonus - center_penalty),
        )
        self._last_center_points = (
            lane_center_x, bottom_y, int(round(mid_x)), mid_y
        )
        debug = {
            "left_line_count": len(left),
            "right_line_count": len(right),
            "total_line_count": total_count,
            "total_line_length": round(total_length, 2),
            "both_sides": bool(left_fit and right_fit),
        }
        return LaneResult(
            ok=True,
            stage_id=self.stage_id,
            lane_center_x=lane_center_x,
            image_center_x=image_center_x,
            error_px=center_error,
            confidence=confidence,
            heading_error_deg=heading_error,
            left_line_count=len(left),
            right_line_count=len(right),
            total_line_count=total_count,
            steer_norm=steer,
            debug=debug,
        )

    @staticmethod
    def _fit_side(
        segments: List[Segment], roi_height: int
    ) -> Optional[Tuple[int, int, int, int]]:
        if not segments:
            return None
        y_values: List[float] = []
        x_values: List[float] = []
        for x1, y1, x2, y2 in segments:
            x_values.extend((x1, x2))
            y_values.extend((y1, y2))
        if len(set(y_values)) < 2:
            return None
        # Fitting x as a function of y is stable for near-vertical lane lines.
        coefficient, intercept = np.polyfit(y_values, x_values, 1)
        bottom_y = roi_height - 1
        mid_y = roi_height // 2
        bottom_x = int(round(coefficient * bottom_y + intercept))
        mid_x = int(round(coefficient * mid_y + intercept))
        return bottom_x, bottom_y, mid_x, mid_y

    def draw_debug(
        self, frame_rgb: np.ndarray, result: LaneResult
    ) -> np.ndarray:
        """Draw Stage 2 segments, fits, center, heading, and telemetry."""
        debug = frame_rgb.copy()
        height, width = debug.shape[:2]
        y_start = min(self._last_roi_bounds[0], height)
        y_end = min(self._last_roi_bounds[1], height)
        bottom = max(y_start, y_end - 1)
        cv2.rectangle(debug, (0, y_start), (max(0, width - 1), bottom), (0, 215, 232), 2)
        cv2.line(debug, (result.image_center_x, y_start), (result.image_center_x, bottom), (220, 90, 255), 2)

        colors = {
            "left": (69, 227, 138),
            "right": (70, 150, 255),
            "rejected": (105, 115, 125),
        }
        if result.debug:
            for (x1, y1, x2, y2), side in self._last_segments:
                cv2.line(debug, (x1, y1 + y_start), (x2, y2 + y_start), colors[side], 1)
        if result.ok:
            for side, (bottom_x, bottom_y, mid_x, mid_y) in self._last_fits.items():
                cv2.line(
                    debug,
                    (bottom_x, bottom_y + y_start),
                    (mid_x, mid_y + y_start),
                    colors[side],
                    4,
                )
        if result.ok and self._last_center_points:
            center_x, bottom_y, mid_x, mid_y = self._last_center_points
            cv2.circle(debug, (center_x, bottom_y + y_start), 7, (255, 93, 115), -1)
            cv2.arrowedLine(
                debug,
                (center_x, bottom_y + y_start),
                (mid_x, mid_y + y_start),
                (255, 200, 70),
                3,
                tipLength=0.15,
            )

        status = "OK" if result.ok else "LOST"
        error = "--" if result.error_px is None else f"{result.error_px:+.1f}px"
        heading = "--" if result.heading_error_deg is None else f"{result.heading_error_deg:+.1f}deg"
        lines = (
            (f"{self.display_name} | {status}", (69, 227, 138) if result.ok else (255, 93, 115)),
            (f"Error: {error} | Heading: {heading}", (231, 242, 247)),
            (f"Lines L/R/T: {result.left_line_count}/{result.right_line_count}/{result.total_line_count}", (231, 242, 247)),
            (f"Confidence: {result.confidence:.3f}", (231, 242, 247)),
            (f"Steer: {result.steer_norm:+.3f}", (231, 242, 247)),
        )
        for index, (text, color) in enumerate(lines):
            cv2.putText(debug, text, (12, 28 + index * 25), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2, cv2.LINE_AA)
        return debug

    def _lost(
        self,
        image_center_x: int,
        left_count: int = 0,
        right_count: int = 0,
    ) -> LaneResult:
        total = left_count + right_count
        return LaneResult(
            ok=False,
            stage_id=self.stage_id,
            lane_center_x=None,
            image_center_x=image_center_x,
            error_px=None,
            confidence=0.0,
            heading_error_deg=None,
            left_line_count=left_count,
            right_line_count=right_count,
            total_line_count=total,
            steer_norm=0.0,
            debug={
                "left_line_count": left_count,
                "right_line_count": right_count,
                "total_line_count": total,
            },
        )
