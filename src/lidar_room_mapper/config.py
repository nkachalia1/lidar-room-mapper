from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MapConfig:
    width_m: float = 8.0
    height_m: float = 8.0
    resolution_m: float = 0.05
    max_range_m: float = 6.0
    occupied_log_odds: float = 0.85
    free_log_odds: float = -0.35
    min_log_odds: float = -4.0
    max_log_odds: float = 4.0


@dataclass(frozen=True)
class RuntimeConfig:
    camera_interval_s: float = 2.0
    artifact_dir: str = "artifacts"
    serial_baud: int = 115200
