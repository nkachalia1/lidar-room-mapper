from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin
from typing import Iterable

from lidar_room_mapper.config import MapConfig
from lidar_room_mapper.models import LidarMeasurement, LidarScan


@dataclass(frozen=True)
class GridStats:
    scans_integrated: int
    occupied_cells: int
    free_cells: int
    unknown_cells: int


class OccupancyGrid:
    """Log-odds 2D occupancy grid with the robot fixed at the map center."""

    def __init__(self, config: MapConfig | None = None) -> None:
        self.config = config or MapConfig()
        self.width_cells = int(round(self.config.width_m / self.config.resolution_m))
        self.height_cells = int(round(self.config.height_m / self.config.resolution_m))
        self.origin_x_m = -self.config.width_m / 2.0
        self.origin_y_m = -self.config.height_m / 2.0
        self._grid = [
            [0.0 for _ in range(self.width_cells)] for _ in range(self.height_cells)
        ]
        self._scans_integrated = 0

    def reset(self) -> None:
        for y in range(self.height_cells):
            row = self._grid[y]
            for x in range(self.width_cells):
                row[x] = 0.0
        self._scans_integrated = 0

    def integrate_scan(self, scan: LidarScan) -> None:
        for measurement in scan.valid_measurements():
            self.integrate_measurement(measurement)
        self._scans_integrated += 1

    def integrate_measurement(self, measurement: LidarMeasurement) -> None:
        distance_m = min(measurement.distance_m, self.config.max_range_m)
        angle_rad = radians(measurement.angle_deg)
        end_x_m = cos(angle_rad) * distance_m
        end_y_m = sin(angle_rad) * distance_m

        robot_cell = self.world_to_cell(0.0, 0.0)
        end_cell = self.world_to_cell(end_x_m, end_y_m)
        if robot_cell is None or end_cell is None:
            return

        cells = list(self._bresenham(robot_cell[0], robot_cell[1], end_cell[0], end_cell[1]))
        if not cells:
            return

        hit_obstacle = measurement.distance_m <= self.config.max_range_m
        free_cells = cells[:-1] if hit_obstacle else cells
        for x, y in free_cells:
            self._add_log_odds(x, y, self.config.free_log_odds)

        if hit_obstacle:
            self._add_log_odds(cells[-1][0], cells[-1][1], self.config.occupied_log_odds)

    def world_to_cell(self, x_m: float, y_m: float) -> tuple[int, int] | None:
        x = int((x_m - self.origin_x_m) / self.config.resolution_m)
        y = int((y_m - self.origin_y_m) / self.config.resolution_m)
        if 0 <= x < self.width_cells and 0 <= y < self.height_cells:
            return x, y
        return None

    def cell_to_world(self, x: int, y: int) -> tuple[float, float]:
        return (
            self.origin_x_m + (x + 0.5) * self.config.resolution_m,
            self.origin_y_m + (y + 0.5) * self.config.resolution_m,
        )

    def as_probability_rows(self) -> list[list[int]]:
        rows: list[list[int]] = []
        for y in range(self.height_cells):
            row: list[int] = []
            for x in range(self.width_cells):
                row.append(self._probability_percent(self._grid[y][x]))
            rows.append(row)
        return rows

    def stats(self) -> GridStats:
        occupied = 0
        free = 0
        unknown = 0
        for row in self._grid:
            for value in row:
                if value > 0.25:
                    occupied += 1
                elif value < -0.25:
                    free += 1
                else:
                    unknown += 1
        return GridStats(self._scans_integrated, occupied, free, unknown)

    def to_payload(self) -> dict[str, object]:
        stats = self.stats()
        return {
            "width": self.width_cells,
            "height": self.height_cells,
            "resolution_m": self.config.resolution_m,
            "origin": {"x_m": self.origin_x_m, "y_m": self.origin_y_m},
            "robot": {"x_m": 0.0, "y_m": 0.0},
            "grid": self.as_probability_rows(),
            "stats": {
                "scans_integrated": stats.scans_integrated,
                "occupied_cells": stats.occupied_cells,
                "free_cells": stats.free_cells,
                "unknown_cells": stats.unknown_cells,
            },
        }

    def _add_log_odds(self, x: int, y: int, delta: float) -> None:
        value = self._grid[y][x] + delta
        self._grid[y][x] = max(
            self.config.min_log_odds, min(self.config.max_log_odds, value)
        )

    @staticmethod
    def _probability_percent(log_odds: float) -> int:
        odds = 2.718281828459045 ** log_odds
        probability = odds / (1.0 + odds)
        return int(round(probability * 100.0))

    @staticmethod
    def _bresenham(x0: int, y0: int, x1: int, y1: int) -> Iterable[tuple[int, int]]:
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx + dy
        x = x0
        y = y0

        while True:
            yield x, y
            if x == x1 and y == y1:
                break
            twice_error = 2 * error
            if twice_error >= dy:
                error += dy
                x += sx
            if twice_error <= dx:
                error += dx
                y += sy
