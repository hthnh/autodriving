from dataclasses import dataclass
from enum import Enum


class ControlMode(str, Enum):
    MANUAL = "manual"
    LANE_KEEP = "lane_keep"
    FOLLOW = "follow"
    STOP = "stop"


@dataclass
class ControlIntent:
    speed: float = 0.0      # -1.0..+1.0
    steer: float = 0.0      # -1.0..+1.0
    brake: bool = False
    mode: ControlMode = ControlMode.MANUAL
    source: str = "manual"

    def clamp(self):
        self.speed = max(-1.0, min(1.0, self.speed))
        self.steer = max(-1.0, min(1.0, self.steer))
        return self

    @staticmethod
    def stop(source="system"):
        return ControlIntent(
            speed=0.0,
            steer=0.0,
            brake=True,
            mode=ControlMode.STOP,
            source=source,
        )