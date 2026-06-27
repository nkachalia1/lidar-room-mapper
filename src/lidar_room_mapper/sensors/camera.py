from __future__ import annotations

import time
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
        self.width, self.height = resolution
        self._camera = Picamera2()
        config = self._camera.create_still_configuration(
            main={"size": (self.width, self.height)}
        )
        self._camera.configure(config)
        self._camera.start()

    def capture(self) -> CameraFrame | None:
        timestamp = time.time()
        output = self.output_dir / "latest.jpg"
        self._camera.capture_file(str(output))
        return CameraFrame(
            path=str(output),
            timestamp=timestamp,
            width=self.width,
            height=self.height,
        )

    def close(self) -> None:
        self._camera.stop()
