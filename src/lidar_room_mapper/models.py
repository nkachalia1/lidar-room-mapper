from __future__ import annotations

from dataclasses import dataclass, field
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
class CameraFrame:
    """Metadata for a captured camera frame."""

    path: str
    timestamp: float
    width: int
    height: int
