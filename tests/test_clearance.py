from lidar_room_mapper.perception import ClearanceAnalyzer, ClearanceConfig
from lidar_room_mapper.models import LidarMeasurement, LidarScan


def test_front_clearance_uses_angle_offset() -> None:
    analyzer = ClearanceAnalyzer(angle_offset_deg=-100.5)
    scan = LidarScan(
        (
            LidarMeasurement(angle_deg=100.5, distance_mm=300.0, quality=15),
            LidarMeasurement(angle_deg=160.0, distance_mm=900.0, quality=15),
        )
    )

    result = analyzer.analyze(scan)

    assert result.status == "blocked"
    assert result.front_clearance_m == 0.3
    assert result.nearest_distance_m == 0.3
    assert result.nearest_angle_deg == 0.0


def test_reports_left_and_right_clearance_sectors() -> None:
    analyzer = ClearanceAnalyzer()
    scan = LidarScan(
        (
            LidarMeasurement(angle_deg=45.0, distance_mm=1200.0, quality=10),
            LidarMeasurement(angle_deg=-50.0, distance_mm=800.0, quality=10),
            LidarMeasurement(angle_deg=170.0, distance_mm=250.0, quality=10),
        )
    )

    result = analyzer.analyze(scan)

    assert result.status == "caution"
    assert result.left_clearance_m == 1.2
    assert result.right_clearance_m == 0.8
    assert result.nearest_distance_m == 0.25
    assert result.front_clearance_m is None


def test_caution_when_front_obstacle_is_close_but_not_blocking() -> None:
    analyzer = ClearanceAnalyzer()
    scan = LidarScan((LidarMeasurement(angle_deg=8.0, distance_mm=600.0),))

    result = analyzer.analyze(scan)

    assert result.status == "caution"
    assert result.front_clearance_m == 0.6


def test_waiting_when_no_measurements_pass_filters() -> None:
    analyzer = ClearanceAnalyzer(ClearanceConfig(min_quality=5))
    scan = LidarScan((LidarMeasurement(angle_deg=0.0, distance_mm=500.0, quality=1),))

    result = analyzer.analyze(scan)

    assert result.status == "waiting"
    assert result.obstacle_count == 0
