from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, sin
from time import time


@dataclass(frozen=True)
class LidarMeasurement:
    """One polar LiDAR reading."""

    angle_deg: float
    distance_mm: float
    quality: int = 0
    start_flag: bool = False
    timestamp: float = field(default_factory=time)

    @property
    def distance_m(self) -> float:
        return self.distance_mm / 1000.0


@dataclass(frozen=True)
class LidarScan:
    """A batch of measurements representing roughly one LiDAR revolution."""

    measurements: tuple[LidarMeasurement, ...]
    timestamp: float = field(default_factory=time)
    source: str = "unknown"

    def valid_measurements(self) -> tuple[LidarMeasurement, ...]:
        return tuple(m for m in self.measurements if m.distance_mm > 0)


@dataclass(frozen=True)
class Pose2D:
    """2D pose in meters and radians."""

    x_m: float = 0.0
    y_m: float = 0.0
    theta_rad: float = 0.0

    def transform_point(self, x_m: float, y_m: float) -> tuple[float, float]:
        c = cos(self.theta_rad)
        s = sin(self.theta_rad)
        return (
            self.x_m + c * x_m - s * y_m,
            self.y_m + s * x_m + c * y_m,
        )

    def compose(self, relative: "Pose2D") -> "Pose2D":
        x_m, y_m = self.transform_point(relative.x_m, relative.y_m)
        return Pose2D(x_m=x_m, y_m=y_m, theta_rad=self.theta_rad + relative.theta_rad)


@dataclass(frozen=True)
class CameraFrame:
    """Metadata for a captured camera frame."""

    path: str
    timestamp: float
    width: int
    height: int
