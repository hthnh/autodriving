#!/usr/bin/env python3
"""Educational, vision-only lane detection using a white HSV threshold."""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass
class LaneResult:
    """Measurements produced for one camera frame."""

    ok: bool
    lane_center_x: Optional[int]
    image_center_x: int
    error: Optional[float]
    confidence: float
    contour_area: float


class LaneDetectorColor:
    """Detect the largest white lane region in the lower part of an image."""

    def __init__(
        self,
        camera_index: int = 0,
        width: int = 640,
        height: int = 480,
        roi_y_start: int = 280,
        roi_y_end: int = 480,
        kernel_size: int = 5,
        min_contour_area: float = 300.0,
        kp: float = 0.08,
        steer_center: float = 90.0,
        steer_sign: float = 1.0,
        steer_min: float = 45.0,
        steer_max: float = 135.0,
    ) -> None:
        self.width = width
        self.height = height
        self.roi_y_start = roi_y_start
        self.roi_y_end = roi_y_end
        self.min_contour_area = min_contour_area
        self.kp = kp
        self.steer_center = steer_center
        self.steer_sign = steer_sign
        self.steer_min = steer_min
        self.steer_max = steer_max

        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (kernel_size, kernel_size)
        )
        self._lower_white = np.array((0, 0, 160), dtype=np.uint8)
        self._upper_white = np.array((180, 80, 255), dtype=np.uint8)
        self._last_roi: Optional[np.ndarray] = None
        self._last_mask: Optional[np.ndarray] = None
        self._last_contour: Optional[np.ndarray] = None
        self._last_roi_bounds = (roi_y_start, roi_y_end)

        self.capture = cv2.VideoCapture(camera_index)
        if self.capture.isOpened():
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def grab_frame(self) -> Optional[np.ndarray]:
        """Read one frame, returning None when the camera cannot provide it."""
        if not self.capture.isOpened():
            return None
        ok, frame = self.capture.read()
        return frame if ok and frame is not None else None

    def detect(self, frame: np.ndarray) -> LaneResult:
        """Run the complete classical vision pipeline on one BGR frame."""
        frame_height, frame_width = frame.shape[:2]

        # Crop the configured lower-image region of interest (ROI).
        y_start = max(0, min(self.roi_y_start, frame_height))
        y_end = max(y_start, min(self.roi_y_end, frame_height))
        roi = frame[y_start:y_end, :]
        self._last_roi = roi.copy()
        self._last_roi_bounds = (y_start, y_end)
        self._last_contour = None

        roi_width = frame_width
        image_center_x = roi_width // 2
        if roi.size == 0:
            self._last_mask = np.zeros((0, roi_width), dtype=np.uint8)
            return LaneResult(False, None, image_center_x, None, 0.0, 0.0)

        # Convert BGR pixels to HSV so brightness and saturation are separable.
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Keep only low-saturation, bright pixels that are likely white paint.
        mask = cv2.inRange(hsv, self._lower_white, self._upper_white)

        # Morphological opening removes isolated white noise.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)

        # Morphological closing fills small holes in the remaining lane region.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        self._last_mask = mask

        # Find every external white region and select the largest contour.
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        largest = max(contours, key=cv2.contourArea) if contours else None
        contour_area = float(cv2.contourArea(largest)) if largest is not None else 0.0

        # Normalize detected contour area by ROI area as a simple confidence.
        roi_area = float(roi.shape[0] * roi.shape[1])
        confidence = max(0.0, min(1.0, contour_area / roi_area))

        # Reject small regions that are unlikely to represent a lane marking.
        if largest is None or contour_area < self.min_contour_area:
            return LaneResult(
                False, None, image_center_x, None, confidence, contour_area
            )

        self._last_contour = largest

        # Calculate the contour centroid from spatial moments.
        moments = cv2.moments(largest)
        if moments["m00"] == 0.0:
            return LaneResult(
                False, None, image_center_x, None, confidence, contour_area
            )
        lane_center_x = int(moments["m10"] / moments["m00"])

        # Lane error is horizontal displacement from the ROI image center.
        error = float(lane_center_x - image_center_x)
        return LaneResult(
            True,
            lane_center_x,
            image_center_x,
            error,
            confidence,
            contour_area,
        )

    def compute_steer(self, result: LaneResult) -> float:
        """Compute a clamped dry-run steering angle without controlling hardware."""
        if not result.ok or result.error is None:
            return self.steer_center
        steer = self.steer_center + self.steer_sign * self.kp * result.error
        return max(self.steer_min, min(self.steer_max, steer))

    def draw_overlay(
        self, frame: np.ndarray, result: LaneResult, fps: float
    ) -> np.ndarray:
        """Return a frame annotated with the current detection measurements."""
        output = frame.copy()
        y_start, y_end = self._last_roi_bounds
        box_bottom = max(y_start, y_end - 1)

        # Mark the ROI used by the detector.
        cv2.rectangle(
            output,
            (0, y_start),
            (max(0, output.shape[1] - 1), box_bottom),
            (255, 180, 0),
            2,
        )

        # Draw the selected contour and its centroid in full-frame coordinates.
        if self._last_contour is not None:
            shifted = self._last_contour.copy()
            shifted[:, :, 1] += y_start
            cv2.drawContours(output, [shifted], -1, (0, 255, 0), 2)
        if result.lane_center_x is not None:
            centroid_y = y_start + max(0, (y_end - y_start) // 2)
            if self._last_contour is not None:
                moments = cv2.moments(self._last_contour)
                if moments["m00"] != 0.0:
                    centroid_y = y_start + int(moments["m01"] / moments["m00"])
            cv2.circle(output, (result.lane_center_x, centroid_y), 6, (0, 0, 255), -1)

        # Draw the desired image-center reference through the ROI.
        cv2.line(
            output,
            (result.image_center_x, y_start),
            (result.image_center_x, box_bottom),
            (255, 0, 255),
            2,
        )

        # Add status and numeric telemetry for visual inspection.
        status = "OK" if result.ok else "LOST"
        status_color = (0, 255, 0) if result.ok else (0, 0, 255)
        error_text = "N/A" if result.error is None else f"{result.error:+.1f} px"
        lines = (
            (status, status_color),
            (f"Error: {error_text}", (255, 255, 255)),
            (f"Confidence: {result.confidence:.3f}", (255, 255, 255)),
            (f"FPS: {fps:.1f}", (255, 255, 255)),
        )
        for index, (text, color) in enumerate(lines):
            cv2.putText(
                output,
                text,
                (12, 28 + index * 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2,
                cv2.LINE_AA,
            )
        return output

    @property
    def roi_image(self) -> Optional[np.ndarray]:
        """Most recently processed ROI, for visualization."""
        return self._last_roi

    @property
    def mask_image(self) -> Optional[np.ndarray]:
        """Most recently processed binary mask, for visualization."""
        return self._last_mask

    def release(self) -> None:
        """Release the camera resource."""
        self.capture.release()


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line interface."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("vision", "dry", "headless"), default="vision")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--roi-y-start", type=int, default=280)
    parser.add_argument("--roi-y-end", type=int, default=480)
    parser.add_argument("--kernel-size", type=int, default=5)
    parser.add_argument("--min-contour-area", type=float, default=300.0)
    return parser


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    """Reject invalid dimensions before opening the camera."""
    if args.width <= 0 or args.height <= 0:
        parser.error("--width and --height must be positive")
    if args.roi_y_start < 0 or args.roi_y_end <= args.roi_y_start:
        parser.error("ROI requires 0 <= --roi-y-start < --roi-y-end")
    if args.kernel_size <= 0:
        parser.error("--kernel-size must be positive")
    if args.min_contour_area < 0:
        parser.error("--min-contour-area cannot be negative")


def format_telemetry(
    result: LaneResult, steer: float, fps: float
) -> str:
    """Format one compact dry-run console update."""
    status = "OK" if result.ok else "LOST"
    error = "N/A" if result.error is None else f"{result.error:+.1f} px"
    return (
        f"{status} | Error: {error} | Confidence: {result.confidence:.3f} | "
        f"Steer: {steer:.1f} | FPS: {fps:.1f}"
    )


def main() -> int:
    """Capture and process frames until Q, Ctrl+C, or camera failure."""
    parser = build_parser()
    args = parser.parse_args()
    validate_args(parser, args)

    detector = LaneDetectorColor(
        camera_index=args.camera_index,
        width=args.width,
        height=args.height,
        roi_y_start=args.roi_y_start,
        roi_y_end=args.roi_y_end,
        kernel_size=args.kernel_size,
        min_contour_area=args.min_contour_area,
    )
    if not detector.capture.isOpened():
        print(f"Error: could not open camera index {args.camera_index}.")
        detector.release()
        return 1

    show_gui = args.mode in ("vision", "dry")
    print_console = args.mode in ("dry", "headless")
    last_frame_time = time.perf_counter()
    last_print_time = last_frame_time - 0.2
    fps = 0.0

    try:
        while True:
            frame = detector.grab_frame()
            if frame is None:
                print("Error: camera frame capture failed.")
                return 1

            # Run detection, dry steering, and frame-rate measurement.
            result = detector.detect(frame)
            steer = detector.compute_steer(result)
            now = time.perf_counter()
            elapsed = now - last_frame_time
            instantaneous_fps = 1.0 / elapsed if elapsed > 0.0 else 0.0
            fps = instantaneous_fps if fps == 0.0 else 0.9 * fps + 0.1 * instantaneous_fps
            last_frame_time = now

            # Dry and headless modes report at most once every 0.2 seconds.
            if print_console and now - last_print_time >= 0.2:
                print(format_telemetry(result, steer, fps), flush=True)
                last_print_time = now

            # GUI modes show all required pipeline/debug views.
            if show_gui:
                overlay = detector.draw_overlay(frame, result, fps)
                cv2.imshow("Original", frame)
                if detector.roi_image is not None:
                    cv2.imshow("ROI", detector.roi_image)
                if detector.mask_image is not None and detector.mask_image.size > 0:
                    cv2.imshow("Mask", detector.mask_image)
                cv2.imshow("Result", overlay)
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        detector.release()
        if show_gui:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
