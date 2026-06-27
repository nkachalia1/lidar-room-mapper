from __future__ import annotations

import json
import time
from dataclasses import asdict
from pathlib import Path

from lidar_room_mapper.models import CameraFrame


class NullCamera:
    def capture(self) -> CameraFrame | None:
        return None

    def close(self) -> None:
        return None


class PiCameraCapture:
    """Small Picamera2 wrapper that keeps the hardware dependency optional."""

    def __init__(
        self,
        output_dir: str | Path = "artifacts",
        resolution: tuple[int, int] = (1280, 720),
        filename: str = "latest.jpg",
    ):
        try:
            from picamera2 import Picamera2
        except ImportError as exc:  # pragma: no cover - exercised on Pi
            raise RuntimeError(
                "Picamera2 is not importable. On Raspberry Pi OS install it with "
                "sudo apt install -y python3-picamera2"
            ) from exc

        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.filename = filename
        self.width, self.height = resolution
        self._camera = Picamera2()
        config = self._camera.create_still_configuration(
            main={"size": (self.width, self.height)}
        )
        self._camera.configure(config)
        self._camera.start()

    def capture(self) -> CameraFrame | None:
        timestamp = time.time()
        output = self.output_dir / self.filename
        self._camera.capture_file(str(output))
        return CameraFrame(
            path=str(output),
            timestamp=timestamp,
            width=self.width,
            height=self.height,
        )

    def close(self) -> None:
        self._camera.stop()


class TimestampedCameraRecorder:
    """Capture numbered still frames and write frame metadata as JSONL."""

    def __init__(self, camera: PiCameraCapture, manifest_path: str | Path):
        self.camera = camera
        self.manifest_path = Path(manifest_path)
        self.manifest_path.parent.mkdir(parents=True, exist_ok=True)
        self._count = 0
        self._handle = self.manifest_path.open("w", encoding="utf-8")

    def capture(self) -> CameraFrame | None:
        self._count += 1
        self.camera.filename = f"frame_{self._count:06d}.jpg"
        frame = self.camera.capture()
        if frame is not None:
            self._handle.write(json.dumps(asdict(frame), separators=(",", ":")) + "\n")
            self._handle.flush()
        return frame

    def close(self) -> None:
        try:
            self.camera.close()
        finally:
            self._handle.close()
