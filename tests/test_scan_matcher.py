from math import radians

from lidar_room_mapper.mapping import OccupancyGrid, ScanMatchConfig, ScanMatcher
from lidar_room_mapper.mapping.scan_matcher import inverse_transform_points, points_to_scan
from lidar_room_mapper.models import LidarMeasurement, LidarScan, Pose2D


def test_scan_matcher_recovers_small_relative_pose() -> None:
    world_points = [
        (1.0, 0.0),
        (1.2, 0.35),
        (0.55, 1.1),
        (1.75, -0.45),
        (0.25, -1.1),
        (-0.7, 0.65),
        (-1.35, -0.2),
        (-0.4, -1.45),
        (1.9, 0.9),
        (-1.1, 1.35),
    ]
    expected_pose = Pose2D(x_m=0.1, y_m=-0.05, theta_rad=radians(4.0))
    reference_scan = points_to_scan(world_points, source="reference")
    current_scan = points_to_scan(
        inverse_transform_points(world_points, expected_pose), source="current"
    )
    matcher = ScanMatcher(
        ScanMatchConfig(
            linear_search_m=0.15,
            angular_search_deg=6.0,
            linear_step_m=0.05,
            angular_step_deg=2.0,
        )
    )

    result = matcher.match(reference_scan, current_scan)

    assert abs(result.delta_pose.x_m - expected_pose.x_m) <= 0.001
    assert abs(result.delta_pose.y_m - expected_pose.y_m) <= 0.001
    assert abs(result.heading_deg - 4.0) <= 0.001
    assert result.score < 0.001
    assert result.accepted is True


def test_scan_matcher_prefers_zero_when_improvement_is_tiny() -> None:
    world_points = [
        (1.0, 0.0),
        (1.2, 0.35),
        (0.55, 1.1),
        (1.75, -0.45),
        (0.25, -1.1),
        (-0.7, 0.65),
        (-1.35, -0.2),
        (-0.4, -1.45),
        (1.9, 0.9),
        (-1.1, 1.35),
    ]
    scan = points_to_scan(world_points)
    matcher = ScanMatcher(
        ScanMatchConfig(
            linear_search_m=0.15,
            angular_search_deg=6.0,
            linear_step_m=0.05,
            angular_step_deg=2.0,
        )
    )

    result = matcher.match(scan, scan)

    assert result.delta_pose == Pose2D()
    assert result.accepted is True


def test_occupancy_grid_integrates_scan_at_pose() -> None:
    grid = OccupancyGrid()
    scan = LidarScan(
        measurements=(
            LidarMeasurement(angle_deg=0.0, distance_mm=1000.0, quality=30),
        )
    )

    grid.integrate_scan_at_pose(scan, Pose2D(x_m=0.5, y_m=0.0, theta_rad=0.0))

    assert grid.stats().occupied_cells == 1
