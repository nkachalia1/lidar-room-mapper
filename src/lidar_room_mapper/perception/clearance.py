from __future__ import annotations

from dataclasses import dataclass

from lidar_room_mapper.models import LidarMeasurement, LidarScan


@dataclass(frozen=True)
class ClearanceConfig:
    min_distance_m: float = 0.12
    max_distance_m: float = 6.0
    min_quality: int = 0
    front_half_angle_deg: float = 18.0
    side_inner_angle_deg: float = 18.0
    side_outer_angle_deg: float = 95.0
    blocked_distance_m: float = 0.35
    caution_distance_m: float = 0.75


@dataclass(frozen=True)
class ClearanceSnapshot:
    status: str
    nearest_distance_m: float | None
    nearest_angle_deg: float | None
    front_clearance_m: float | None
    left_clearance_m: float | None
    right_clearance_m: float | None
    obstacle_count: int
    angle_offset_deg: float

    def to_payload(self) -> dict[str, float | int | str | None]:
        return {
            "status": self.status,
            "nearest_distance_m": _rounded(self.nearest_distance_m),
            "nearest_angle_deg": _rounded(self.nearest_angle_deg),
            "front_clearance_m": _rounded(self.front_clearance_m),
            "left_clearance_m": _rounded(self.left_clearance_m),
            "right_clearance_m": _rounded(self.right_clearance_m),
            "obstacle_count": self.obstacle_count,
            "angle_offset_deg": round(self.angle_offset_deg, 2),
        }


class ClearanceAnalyzer:
    """Converts a scan into simple navigation-oriented clearance telemetry."""

    def __init__(
        self,
        config: ClearanceConfig | None = None,
        angle_offset_deg: float = 0.0,
    ) -> None:
        self.config = config or ClearanceConfig()
        self.angle_offset_deg = angle_offset_deg
        if self.config.min_distance_m < 0:
            raise ValueError("min_distance_m must be non-negative")
        if self.config.max_distance_m <= self.config.min_distance_m:
            raise ValueError("max_distance_m must be greater than min_distance_m")
        if self.config.front_half_angle_deg <= 0:
            raise ValueError("front_half_angle_deg must be positive")
        if self.config.side_outer_angle_deg <= self.config.side_inner_angle_deg:
            raise ValueError("side_outer_angle_deg must exceed side_inner_angle_deg")

    def waiting(self) -> ClearanceSnapshot:
        return ClearanceSnapshot(
            status="waiting",
            nearest_distance_m=None,
            nearest_angle_deg=None,
            front_clearance_m=None,
            left_clearance_m=None,
            right_clearance_m=None,
            obstacle_count=0,
            angle_offset_deg=self.angle_offset_deg,
        )

    def analyze(self, scan: LidarScan) -> ClearanceSnapshot:
        nearest: tuple[float, float] | None = None
        front: float | None = None
        left: float | None = None
        right: float | None = None
        obstacle_count = 0

        for measurement in scan.valid_measurements():
            candidate = self._candidate(measurement)
            if candidate is None:
                continue

            distance_m, relative_angle_deg = candidate
            obstacle_count += 1
            if nearest is None or distance_m < nearest[0]:
                nearest = (distance_m, relative_angle_deg)

            if abs(relative_angle_deg) <= self.config.front_half_angle_deg:
                front = _minimum(front, distance_m)
            elif (
                self.config.side_inner_angle_deg
                < relative_angle_deg
                <= self.config.side_outer_angle_deg
            ):
                left = _minimum(left, distance_m)
            elif (
                -self.config.side_outer_angle_deg
                <= relative_angle_deg
                < -self.config.side_inner_angle_deg
            ):
                right = _minimum(right, distance_m)

        if obstacle_count == 0:
            return self.waiting()

        nearest_distance_m, nearest_angle_deg = nearest or (None, None)
        return ClearanceSnapshot(
            status=self._status(front, nearest_distance_m),
            nearest_distance_m=nearest_distance_m,
            nearest_angle_deg=nearest_angle_deg,
            front_clearance_m=front,
            left_clearance_m=left,
            right_clearance_m=right,
            obstacle_count=obstacle_count,
            angle_offset_deg=self.angle_offset_deg,
        )

    def _candidate(self, measurement: LidarMeasurement) -> tuple[float, float] | None:
        if measurement.quality < self.config.min_quality:
            return None
        distance_m = measurement.distance_m
        if not self.config.min_distance_m <= distance_m <= self.config.max_distance_m:
            return None
        relative_angle_deg = _normalize_angle(
            measurement.angle_deg + self.angle_offset_deg
        )
        return distance_m, relative_angle_deg

    def _status(self, front_m: float | None, nearest_m: float | None) -> str:
        if front_m is not None:
            if front_m <= self.config.blocked_distance_m:
                return "blocked"
            if front_m <= self.config.caution_distance_m:
                return "caution"
        if nearest_m is not None and nearest_m <= self.config.blocked_distance_m:
            return "caution"
        return "clear"


def _normalize_angle(angle_deg: float) -> float:
    normalized = (angle_deg + 180.0) % 360.0 - 180.0
    if normalized == -180.0:
        return 180.0
    return normalized


def _minimum(current: float | None, candidate: float) -> float:
    return candidate if current is None else min(current, candidate)


def _rounded(value: float | None) -> float | None:
    return round(value, 3) if value is not None else None
