from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, radians, sin

from lidar_room_mapper.models import LidarScan, Pose2D


@dataclass(frozen=True)
class ScanMatchConfig:
    max_range_m: float = 4.5
    max_points: int = 80
    linear_search_m: float = 0.15
    angular_search_deg: float = 6.0
    linear_step_m: float = 0.05
    angular_step_deg: float = 2.0
    score_grid_m: float = 0.1
    unmatched_penalty: float = 1.0
    max_accepted_score: float = 0.05
    min_score_improvement: float = 0.0001
    min_score_improvement_ratio: float = 0.25


@dataclass(frozen=True)
class ScanMatchResult:
    delta_pose: Pose2D
    score: float
    reference_points: int
    current_points: int
    accepted: bool

    @property
    def heading_deg(self) -> float:
        return degrees(self.delta_pose.theta_rad)


class ScanMatcher:
    """Small correlative scan matcher for consecutive 2D LiDAR scans."""

    def __init__(self, config: ScanMatchConfig | None = None) -> None:
        self.config = config or ScanMatchConfig()

    def match(
        self,
        reference_scan: LidarScan,
        current_scan: LidarScan,
        initial: Pose2D | None = None,
    ) -> ScanMatchResult:
        reference_points = scan_to_points(
            reference_scan, self.config.max_range_m, self.config.max_points
        )
        current_points = scan_to_points(
            current_scan, self.config.max_range_m, self.config.max_points
        )
        if len(reference_points) < 6 or len(current_points) < 6:
            return ScanMatchResult(
                delta_pose=initial or Pose2D(),
                score=float("inf"),
                reference_points=len(reference_points),
                current_points=len(current_points),
                accepted=False,
            )

        initial = initial or Pose2D()
        reference_index = build_spatial_index(reference_points, self.config.score_grid_m)
        initial_score = score_alignment_indexed(
            reference_index,
            current_points,
            initial,
            self.config.score_grid_m,
            self.config.unmatched_penalty,
        )
        best_pose = initial
        best_score = initial_score
        for dtheta in _frange(
            -self.config.angular_search_deg,
            self.config.angular_search_deg,
            self.config.angular_step_deg,
        ):
            theta = initial.theta_rad + radians(dtheta)
            for dx in _frange(
                -self.config.linear_search_m,
                self.config.linear_search_m,
                self.config.linear_step_m,
            ):
                x_m = initial.x_m + dx
                for dy in _frange(
                    -self.config.linear_search_m,
                    self.config.linear_search_m,
                    self.config.linear_step_m,
                ):
                    pose = Pose2D(x_m=x_m, y_m=initial.y_m + dy, theta_rad=theta)
                    score = score_alignment_indexed(
                        reference_index,
                        current_points,
                        pose,
                        self.config.score_grid_m,
                        self.config.unmatched_penalty,
                    )
                    if score < best_score:
                        best_score = score
                        best_pose = pose

        if not self._meaningfully_better(initial_score, best_score):
            best_pose = initial
            best_score = initial_score

        return ScanMatchResult(
            delta_pose=best_pose,
            score=best_score,
            reference_points=len(reference_points),
            current_points=len(current_points),
            accepted=best_score <= self.config.max_accepted_score,
        )

    def _meaningfully_better(self, initial_score: float, best_score: float) -> bool:
        improvement = initial_score - best_score
        if improvement < self.config.min_score_improvement:
            return False
        if initial_score <= 0:
            return improvement > 0
        return improvement / initial_score >= self.config.min_score_improvement_ratio


def scan_to_points(
    scan: LidarScan, max_range_m: float, max_points: int
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    measurements = scan.valid_measurements()
    if not measurements:
        return points

    stride = max(1, len(measurements) // max_points)
    for measurement in measurements[::stride]:
        distance_m = measurement.distance_m
        if distance_m <= 0.05 or distance_m > max_range_m:
            continue
        angle_rad = radians(measurement.angle_deg)
        points.append((cos(angle_rad) * distance_m, sin(angle_rad) * distance_m))
        if len(points) >= max_points:
            break
    return points


def score_alignment(
    reference_points: list[tuple[float, float]],
    current_points: list[tuple[float, float]],
    pose: Pose2D,
) -> float:
    return score_alignment_indexed(
        build_spatial_index(reference_points, 0.1),
        current_points,
        pose,
        cell_size_m=0.1,
        unmatched_penalty=1.0,
    )


def build_spatial_index(
    points: list[tuple[float, float]], cell_size_m: float
) -> dict[tuple[int, int], list[tuple[float, float]]]:
    index: dict[tuple[int, int], list[tuple[float, float]]] = {}
    for point in points:
        cell = _point_cell(point, cell_size_m)
        index.setdefault(cell, []).append(point)
    return index


def score_alignment_indexed(
    reference_index: dict[tuple[int, int], list[tuple[float, float]]],
    current_points: list[tuple[float, float]],
    pose: Pose2D,
    cell_size_m: float,
    unmatched_penalty: float,
) -> float:
    transformed = [pose.transform_point(x_m, y_m) for x_m, y_m in current_points]
    distances = sorted(
        _nearest_distance_sq_indexed(
            point, reference_index, cell_size_m, unmatched_penalty
        )
        for point in transformed
    )
    if not distances:
        return float("inf")
    keep = max(1, int(len(distances) * 0.75))
    return sum(distances[:keep]) / keep


def points_to_scan(points: list[tuple[float, float]], source: str = "points") -> LidarScan:
    from lidar_room_mapper.models import LidarMeasurement

    measurements = []
    for index, (x_m, y_m) in enumerate(points):
        measurements.append(
            LidarMeasurement(
                angle_deg=degrees(atan2(y_m, x_m)),
                distance_mm=hypot(x_m, y_m) * 1000.0,
                quality=30,
                start_flag=index == 0,
            )
        )
    return LidarScan(measurements=tuple(measurements), source=source)


def inverse_transform_points(
    points: list[tuple[float, float]], pose: Pose2D
) -> list[tuple[float, float]]:
    c = cos(pose.theta_rad)
    s = sin(pose.theta_rad)
    local_points = []
    for x_m, y_m in points:
        dx = x_m - pose.x_m
        dy = y_m - pose.y_m
        local_points.append((c * dx + s * dy, -s * dx + c * dy))
    return local_points


def _nearest_distance_sq(
    point: tuple[float, float], candidates: list[tuple[float, float]]
) -> float:
    x_m, y_m = point
    return min((x_m - cx) ** 2 + (y_m - cy) ** 2 for cx, cy in candidates)


def _nearest_distance_sq_indexed(
    point: tuple[float, float],
    index: dict[tuple[int, int], list[tuple[float, float]]],
    cell_size_m: float,
    unmatched_penalty: float,
) -> float:
    x_cell, y_cell = _point_cell(point, cell_size_m)
    candidates: list[tuple[float, float]] = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            candidates.extend(index.get((x_cell + dx, y_cell + dy), []))
    if not candidates:
        return unmatched_penalty
    return _nearest_distance_sq(point, candidates)


def _point_cell(point: tuple[float, float], cell_size_m: float) -> tuple[int, int]:
    return (int(point[0] // cell_size_m), int(point[1] // cell_size_m))


def _frange(start: float, stop: float, step: float) -> list[float]:
    values: list[float] = []
    current = start
    epsilon = step / 1000.0
    while current <= stop + epsilon:
        values.append(round(current, 10))
        current += step
    return values
