from lidar_room_mapper.config import MapConfig
from lidar_room_mapper.mapping import OccupancyGrid
from lidar_room_mapper.models import LidarMeasurement, LidarScan


def test_integrates_free_and_occupied_cells() -> None:
    grid = OccupancyGrid(MapConfig(width_m=4.0, height_m=4.0, resolution_m=0.1))
    scan = LidarScan(
        measurements=(
            LidarMeasurement(angle_deg=0.0, distance_mm=1000.0, quality=30, start_flag=True),
        )
    )

    grid.integrate_scan(scan)
    stats = grid.stats()

    assert stats.scans_integrated == 1
    assert stats.occupied_cells == 1
    assert stats.free_cells > 5


def test_out_of_map_measurements_are_ignored() -> None:
    grid = OccupancyGrid(MapConfig(width_m=1.0, height_m=1.0, resolution_m=0.1))
    scan = LidarScan(
        measurements=(
            LidarMeasurement(angle_deg=0.0, distance_mm=2000.0, quality=30, start_flag=True),
        )
    )

    grid.integrate_scan(scan)
    stats = grid.stats()

    assert stats.scans_integrated == 1
    assert stats.occupied_cells == 0
