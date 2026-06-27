from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any

from lidar_room_mapper.config import MapConfig, RuntimeConfig
from lidar_room_mapper.fusion import LidarCameraProjector, ProjectionResult
from lidar_room_mapper.mapping import OccupancyGrid
from lidar_room_mapper.models import CameraFrame, LidarScan
from lidar_room_mapper.perception import ClearanceAnalyzer, ClearanceSnapshot
from lidar_room_mapper.sensors.camera import NullCamera
from lidar_room_mapper.sensors.lidar import Scanner


class MappingRuntime:
    """Owns the scanner, camera, mapper, and background integration loop."""

    def __init__(
        self,
        scanner: Scanner,
        camera: Any | None = None,
        projector: LidarCameraProjector | None = None,
        clearance_analyzer: ClearanceAnalyzer | None = None,
        map_config: MapConfig | None = None,
        runtime_config: RuntimeConfig | None = None,
    ) -> None:
        self.scanner = scanner
        self.camera = camera or NullCamera()
        self.projector = projector
        self.clearance_analyzer = clearance_analyzer or ClearanceAnalyzer(
            angle_offset_deg=projector.rig.lidar_angle_offset_deg if projector else 0.0
        )
        self.grid = OccupancyGrid(map_config)
        self.config = runtime_config or RuntimeConfig()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._latest_scan: LidarScan | None = None
        self._latest_frame: CameraFrame | None = None
        self._latest_projection: ProjectionResult | None = None
        self._latest_clearance: ClearanceSnapshot = self.clearance_analyzer.waiting()
        self._projection_scan_timestamp: float | None = None
        self._projection_frame_timestamp: float | None = None
        self._projection_sync_delta_ms: float | None = None
        self._error: str | None = None
        self._started_at: float | None = None
        Path(self.config.artifact_dir).mkdir(parents=True, exist_ok=True)

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._started_at = time.time()
        self._thread = threading.Thread(target=self._run, name="mapping-runtime", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self.scanner.close()
        self.camera.close()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def reset(self) -> None:
        with self._lock:
            self.grid.reset()
            self._latest_scan = None
            self._latest_frame = None
            self._latest_projection = None
            self._latest_clearance = self.clearance_analyzer.waiting()
            self._projection_scan_timestamp = None
            self._projection_frame_timestamp = None
            self._projection_sync_delta_ms = None
            self._error = None
            self._started_at = time.time()

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            payload = self.grid.to_payload()
            payload["latest_scan"] = {
                "points": len(self._latest_scan.measurements) if self._latest_scan else 0,
                "timestamp": self._latest_scan.timestamp if self._latest_scan else None,
                "source": self._latest_scan.source if self._latest_scan else None,
            }
            payload["camera"] = {
                "path": self._latest_frame.path if self._latest_frame else None,
                "timestamp": self._latest_frame.timestamp if self._latest_frame else None,
                "width": self._latest_frame.width if self._latest_frame else None,
                "height": self._latest_frame.height if self._latest_frame else None,
            }
            fusion: dict[str, object] = {
                "enabled": self.projector is not None,
                "calibration_status": self.projector.rig.status
                if self.projector
                else None,
                "points": [],
                "projected_count": 0,
                "scan_timestamp": self._projection_scan_timestamp,
                "frame_timestamp": self._projection_frame_timestamp,
                "sync_delta_ms": self._projection_sync_delta_ms,
            }
            if self._latest_projection is not None:
                fusion.update(self._latest_projection.to_payload())
            payload["fusion"] = fusion
            payload["clearance"] = self._latest_clearance.to_payload()
            payload["runtime"] = {
                "running": self._thread.is_alive() if self._thread else False,
                "uptime_s": round(time.time() - self._started_at, 2)
                if self._started_at
                else 0.0,
                "error": self._error,
            }
            return payload

    def _run(self) -> None:
        next_camera_capture = 0.0
        try:
            for scan in self.scanner.iter_scans():
                if self._stop_event.is_set():
                    return
                frame = None
                now = time.time()
                capture_near = getattr(self.camera, "capture_near", None)
                if callable(capture_near):
                    frame = capture_near(scan.timestamp)
                elif now >= next_camera_capture:
                    frame = self.camera.capture()
                    next_camera_capture = now + self.config.camera_interval_s

                projection = None
                sync_delta_ms = None
                if frame is not None and self.projector is not None:
                    projection = self.projector.project_scan(
                        scan,
                        width=frame.width,
                        height=frame.height,
                    )
                    sync_delta_ms = (scan.timestamp - frame.timestamp) * 1000.0
                clearance = self.clearance_analyzer.analyze(scan)

                with self._lock:
                    self.grid.integrate_scan(scan)
                    self._latest_scan = scan
                    self._latest_clearance = clearance
                    if frame is not None:
                        self._latest_frame = frame
                    if projection is not None and frame is not None:
                        self._latest_projection = projection
                        self._projection_scan_timestamp = scan.timestamp
                        self._projection_frame_timestamp = frame.timestamp
                        self._projection_sync_delta_ms = round(sync_delta_ms or 0.0, 2)
        except Exception as exc:  # pragma: no cover - surfaced in dashboard
            with self._lock:
                self._error = str(exc)
