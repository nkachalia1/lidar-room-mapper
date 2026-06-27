from __future__ import annotations

import binascii
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path

from lidar_room_mapper.mapping.occupancy import OccupancyGrid


@dataclass(frozen=True)
class MapExportPaths:
    png: Path
    pgm: Path
    yaml: Path


def export_grid(grid: OccupancyGrid, output_prefix: str | Path) -> MapExportPaths:
    prefix = Path(output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)

    image_rows = _ros_map_rows(grid)
    png_path = prefix.with_suffix(".png")
    pgm_path = prefix.with_suffix(".pgm")
    yaml_path = prefix.with_suffix(".yaml")

    write_png(png_path, image_rows)
    write_pgm(pgm_path, image_rows)
    write_yaml(yaml_path, grid, pgm_path.name)
    return MapExportPaths(png=png_path, pgm=pgm_path, yaml=yaml_path)


def write_pgm(path: Path, rows: list[list[int]]) -> None:
    if not rows or not rows[0]:
        raise ValueError("Cannot export an empty map.")
    width = len(rows[0])
    height = len(rows)
    with path.open("wb") as handle:
        handle.write(f"P5\n# Pi LiDAR Room Mapper\n{width} {height}\n255\n".encode("ascii"))
        for row in rows:
            handle.write(bytes(row))


def write_png(path: Path, rows: list[list[int]]) -> None:
    if not rows or not rows[0]:
        raise ValueError("Cannot export an empty map.")
    width = len(rows[0])
    height = len(rows)
    raw = b"".join(b"\x00" + bytes(row) for row in rows)
    payload = b"".join(
        [
            b"\x89PNG\r\n\x1a\n",
            _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)),
            _png_chunk(b"IDAT", zlib.compress(raw, level=9)),
            _png_chunk(b"IEND", b""),
        ]
    )
    path.write_bytes(payload)


def write_yaml(path: Path, grid: OccupancyGrid, image_name: str) -> None:
    config = grid.config
    path.write_text(
        "\n".join(
            [
                f"image: {image_name}",
                "mode: trinary",
                f"resolution: {config.resolution_m}",
                f"origin: [{grid.origin_x_m}, {grid.origin_y_m}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.35",
                "",
            ]
        ),
        encoding="utf-8",
    )


def _ros_map_rows(grid: OccupancyGrid) -> list[list[int]]:
    probability_rows = grid.as_probability_rows()
    image_rows: list[list[int]] = []
    for row in reversed(probability_rows):
        image_rows.append([_probability_to_ros_pixel(probability) for probability in row])
    return image_rows


def _probability_to_ros_pixel(probability: int) -> int:
    if probability >= 65:
        return 0
    if probability <= 35:
        return 254
    return 205


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    crc = binascii.crc32(kind)
    crc = binascii.crc32(payload, crc) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", crc)
