from pathlib import Path

from lidar_room_mapper.config import MapConfig
from lidar_room_mapper.mapping import OccupancyGrid, export_grid
from lidar_room_mapper.models import LidarMeasurement, LidarScan


def test_export_grid_writes_png_pgm_and_yaml(tmp_path: Path) -> None:
    grid = OccupancyGrid(MapConfig(width_m=2.0, height_m=2.0, resolution_m=0.1))
    grid.integrate_scan(
        LidarScan(
            measurements=(
                LidarMeasurement(angle_deg=0.0, distance_mm=600.0, quality=30),
                LidarMeasurement(angle_deg=90.0, distance_mm=600.0, quality=30),
            )
        )
    )

    paths = export_grid(grid, tmp_path / "room")

    assert paths.png.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert paths.pgm.read_bytes().startswith(b"P5\n# Pi LiDAR Room Mapper\n20 20\n255\n")
    yaml_text = paths.yaml.read_text(encoding="utf-8")
    assert "image: room.pgm" in yaml_text
    assert "resolution: 0.1" in yaml_text
    assert "origin: [-1.0, -1.0, 0.0]" in yaml_text
