"""Stage 1 lane detection using color presets and a contour centroid."""

from typing import Optional, Tuple

import cv2
import numpy as np

from .base import BaseLaneDetector, LaneResult


class ColorLaneDetector(BaseLaneDetector):
    """Find the largest tape-colored region in a lower image ROI."""

    stage_id = "stage_1_color_threshold"
    display_name = "Stage 1 — Color Threshold"

    COLOR_PRESETS = {
        "white_on_dark": ((0, 0, 160), (180, 80, 255)),
        "black_on_bright": ((0, 0, 0), (180, 255, 80)),
    }
    DEFAULT_SETTINGS = {
        "preset": "black_on_bright",
        "roi_y_start": 280,
        "roi_y_end": 480,
        "min_area": 300,
        "morphology_kernel": 5,
        "kp": 0.003,
        "steer_sign": 1,
        "base_speed": 0.12,
        "min_confidence": 0.005,
        "lost_lane_timeout": 0.3,
        "max_steer_norm": 0.6,
    }

    def __init__(
        self,
        roi_y_start: int = 280,
        roi_y_end: int = 480,
        hsv_low: Optional[Tuple[int, int, int]] = None,
        hsv_high: Optional[Tuple[int, int, int]] = None,
        min_area: float = 300,
        morphology_kernel: int = 5,
        kp: float = 0.003,
        steer_sign: float = 1,
        preset: str = "black_on_bright",
        settings: Optional[dict] = None,
    ) -> None:
        initial = dict(self.DEFAULT_SETTINGS)
        initial.update({
            "preset": preset,
            "roi_y_start": roi_y_start,
            "roi_y_end": roi_y_end,
            "min_area": min_area,
            "morphology_kernel": morphology_kernel,
            "kp": kp,
            "steer_sign": steer_sign,
        })
        if settings:
            initial.update(settings)
        self.settings = dict(self.DEFAULT_SETTINGS)
        self.update_settings(initial)

        preset_low, preset_high = self.COLOR_PRESETS[self.preset]
        self.hsv_low = np.asarray(
            preset_low if hsv_low is None else hsv_low, dtype=np.uint8
        )
        self.hsv_high = np.asarray(
            preset_high if hsv_high is None else hsv_high, dtype=np.uint8
        )
        # Debug geometry is updated by detect() and consumed by draw_debug().
        self._last_contour: Optional[np.ndarray] = None
        self._last_centroid_y: Optional[int] = None
        self._last_roi_bounds = (self.roi_y_start, self.roi_y_end)

    def update_settings(self, settings: dict) -> None:
        """Validate and atomically apply Stage 1 settings."""
        unknown = set(settings) - set(self.DEFAULT_SETTINGS)
        if unknown:
            raise ValueError("unknown Stage 1 setting(s): " + ", ".join(sorted(unknown)))
        values = dict(getattr(self, "settings", self.DEFAULT_SETTINGS))
        values.update(settings)
        preset = str(values["preset"]).strip().lower()
        if preset not in self.COLOR_PRESETS:
            raise ValueError("preset must be black_on_bright or white_on_dark")
        for name in ("roi_y_start", "roi_y_end", "min_area", "morphology_kernel"):
            values[name] = int(values[name])
        for name in (
            "kp", "base_speed", "min_confidence", "lost_lane_timeout",
            "max_steer_norm",
        ):
            values[name] = float(values[name])
        values["steer_sign"] = int(values["steer_sign"])
        values["preset"] = preset
        if values["roi_y_start"] < 0 or values["roi_y_end"] <= values["roi_y_start"]:
            raise ValueError("ROI requires 0 <= roi_y_start < roi_y_end")
        if values["min_area"] < 0:
            raise ValueError("min_area cannot be negative")
        kernel = values["morphology_kernel"]
        if kernel <= 0 or kernel % 2 == 0:
            raise ValueError("morphology_kernel must be a positive odd integer")
        if values["kp"] < 0 or values["steer_sign"] not in (-1, 1):
            raise ValueError("kp must be nonnegative and steer_sign must be -1 or 1")
        if not 0 <= values["base_speed"] <= 1:
            raise ValueError("base_speed must be between 0 and 1")
        if not 0 <= values["min_confidence"] <= 1:
            raise ValueError("min_confidence must be between 0 and 1")
        if values["lost_lane_timeout"] <= 0:
            raise ValueError("lost_lane_timeout must be positive")
        if not 0 <= values["max_steer_norm"] <= 1:
            raise ValueError("max_steer_norm must be between 0 and 1")

        self.settings = values
        for name, value in values.items():
            setattr(self, name, value)
        low, high = self.COLOR_PRESETS[preset]
        self.hsv_low = np.asarray(low, dtype=np.uint8)
        self.hsv_high = np.asarray(high, dtype=np.uint8)
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (kernel, kernel)
        )

    def detect(self, frame_rgb: np.ndarray) -> LaneResult:
        """Run the Stage 1 pipeline and return dry-run steering telemetry."""
        if not isinstance(frame_rgb, np.ndarray) or frame_rgb.ndim != 3:
            raise ValueError("frame_rgb must be an HxWxC numpy array")
        if frame_rgb.shape[2] < 3:
            raise ValueError("frame_rgb must contain at least three color channels")

        frame_height, frame_width = frame_rgb.shape[:2]
        image_center_x = frame_width // 2

        # 1. Crop the configured lower-image region of interest.
        y_start = min(self.roi_y_start, frame_height)
        y_end = min(self.roi_y_end, frame_height)
        self._last_roi_bounds = (y_start, y_end)
        self._last_contour = None
        self._last_centroid_y = None
        if y_end <= y_start or frame_width == 0:
            return self._lost_result(image_center_x)
        roi = frame_rgb[y_start:y_end, :, :3]

        # 2. Convert the Picamera2 RGB image into HSV color space.
        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)

        # 3. Threshold pixels selected by the active tape/background preset.
        mask = cv2.inRange(hsv, self.hsv_low, self.hsv_high)

        # 4. Remove isolated noise, then close small holes in lane regions.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        # 5. Select the largest external contour and reject small detections.
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return self._lost_result(image_center_x)
        largest = max(contours, key=cv2.contourArea)
        contour_area = float(cv2.contourArea(largest))
        if contour_area < self.min_area:
            return self._lost_result(image_center_x, contour_area)

        # 6. Compute the lane contour centroid from image moments.
        moments = cv2.moments(largest)
        if moments["m00"] == 0:
            return self._lost_result(image_center_x, contour_area)
        lane_center_x = int(moments["m10"] / moments["m00"])
        centroid_y = int(moments["m01"] / moments["m00"])

        # 7. Express horizontal lane displacement in pixels.
        error_px = float(lane_center_x - image_center_x)

        # 8. Convert proportional steering to the low-level normalized range.
        steer_norm = self.steer_sign * self.kp * error_px
        steer_norm = max(-1.0, min(1.0, steer_norm))

        roi_area = float(roi.shape[0] * roi.shape[1])
        confidence = max(0.0, min(1.0, contour_area / roi_area))
        self._last_contour = largest.copy()
        self._last_centroid_y = centroid_y
        return LaneResult(
            ok=True,
            stage_id=self.stage_id,
            lane_center_x=lane_center_x,
            image_center_x=image_center_x,
            error_px=error_px,
            confidence=confidence,
            contour_area=contour_area,
            steer_norm=steer_norm,
            debug={"preset": self.preset, "contour_area": contour_area},
        )

    def draw_debug(self, frame_rgb: np.ndarray, result: LaneResult) -> np.ndarray:
        """Return an RGB copy annotated with the latest detection geometry."""
        debug = frame_rgb.copy()
        frame_height, frame_width = debug.shape[:2]
        y_start = min(self._last_roi_bounds[0], frame_height)
        y_end = min(self._last_roi_bounds[1], frame_height)
        box_bottom = max(y_start, y_end - 1)

        # RGB color tuples are used because this image remains RGB.
        cv2.rectangle(
            debug,
            (0, y_start),
            (max(0, frame_width - 1), box_bottom),
            (0, 215, 232),
            2,
        )
        cv2.line(
            debug,
            (result.image_center_x, y_start),
            (result.image_center_x, box_bottom),
            (220, 90, 255),
            2,
        )

        if result.ok and self._last_contour is not None:
            contour = self._last_contour.copy()
            contour[:, :, 1] += y_start
            cv2.drawContours(debug, [contour], -1, (70, 227, 138), 2)
        if (
            result.ok
            and result.lane_center_x is not None
            and self._last_centroid_y is not None
        ):
            cv2.circle(
                debug,
                (result.lane_center_x, y_start + self._last_centroid_y),
                6,
                (255, 93, 115),
                -1,
            )

        status = "OK" if result.ok else "LOST"
        status_color = (69, 227, 138) if result.ok else (255, 93, 115)
        error_text = "--" if result.error_px is None else f"{result.error_px:+.1f} px"
        lines = (
            (f"{self.display_name} | {status}", status_color),
            (f"Preset: {self.preset}", (231, 242, 247)),
            (f"Error: {error_text}", (231, 242, 247)),
            (f"Confidence: {result.confidence:.3f}", (231, 242, 247)),
            (f"Steer: {result.steer_norm:+.3f}", (231, 242, 247)),
            (f"KP: {self.kp:.4f}", (231, 242, 247)),
        )
        for index, (text, color) in enumerate(lines):
            cv2.putText(
                debug,
                text,
                (12, 28 + index * 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2,
                cv2.LINE_AA,
            )
        return debug

    @staticmethod
    def _lost_result(image_center_x: int, contour_area: float = 0.0) -> LaneResult:
        return LaneResult(
            ok=False,
            stage_id=ColorLaneDetector.stage_id,
            lane_center_x=None,
            image_center_x=image_center_x,
            error_px=None,
            confidence=0.0,
            contour_area=contour_area,
            steer_norm=0.0,
        )
