"""Common result and interface for educational lane detector stages."""

from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import numpy as np


@dataclass
class LaneResult:
    """Common lane measurements returned by every detector stage."""

    ok: bool
    stage_id: str
    lane_center_x: Optional[int]
    image_center_x: int
    error_px: Optional[float]
    confidence: float
    contour_area: float = 0.0
    heading_error_deg: Optional[float] = None
    left_line_count: int = 0
    right_line_count: int = 0
    total_line_count: int = 0
    steer_norm: float = 0.0
    debug: Dict[str, Any] = field(default_factory=dict)


class BaseLaneDetector:
    """Small common interface implemented by each educational stage."""

    stage_id = "base"
    display_name = "Base Lane Detector"

    def update_settings(self, settings: dict) -> None:
        raise NotImplementedError

    def detect(self, frame_rgb: np.ndarray) -> LaneResult:
        raise NotImplementedError

    def draw_debug(
        self, frame_rgb: np.ndarray, result: LaneResult
    ) -> np.ndarray:
        raise NotImplementedError
