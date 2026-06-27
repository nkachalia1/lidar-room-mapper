from __future__ import annotations

from bisect import bisect_left
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


class ReplayCameraFrames:
    """Replay timestamped camera frame metadata from a JSONL manifest."""

    def __init__(self, manifest_path: str | Path):
        self.manifest_path = Path(manifest_path)
        if not self.manifest_path.exists():
            raise FileNotFoundError(f"Camera frame manifest not found: {self.manifest_path}")
        self.frames = tuple(sorted(self._load_frames(), key=lambda frame: frame.timestamp))
        self._timestamps = tuple(frame.timestamp for frame in self.frames)
        self._latest: CameraFrame | None = self.frames[0] if self.frames else None

    def capture_near(self, timestamp: float) -> CameraFrame | None:
        if not self.frames:
            return None

        index = bisect_left(self._timestamps, timestamp)
        candidates: list[CameraFrame] = []
        if index < len(self.frames):
            candidates.append(self.frames[index])
        if index > 0:
            candidates.append(self.frames[index - 1])
        self._latest = min(candidates, key=lambda frame: abs(frame.timestamp - timestamp))
        return self._latest

    def capture(self) -> CameraFrame | None:
        return self._latest

    def close(self) -> None:
        return None

    def _load_frames(self) -> list[CameraFrame]:
        frames: list[CameraFrame] = []
        with self.manifest_path.open("r", encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, start=1):
                line = line.strip()
                if not line:
                    continue
                try:
                    payload = json.loads(line)
                    frames.append(
                        CameraFrame(
                            path=str(self._resolve_frame_path(payload["path"])),
                            timestamp=float(payload["timestamp"]),
                            width=int(payload["width"]),
                            height=int(payload["height"]),
                        )
                    )
                except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                    raise ValueError(
                        f"Invalid camera frame record at {self.manifest_path}:{line_number}"
                    ) from exc
        return frames

    def _resolve_frame_path(self, raw_path: object) -> Path:
        frame_path = Path(str(raw_path))
        if frame_path.is_absolute() or frame_path.exists():
            return frame_path

        candidates = [
            self.manifest_path.parent / frame_path,
            self.manifest_path.parent.parent / frame_path,
            self.manifest_path.parent / self.manifest_path.stem / frame_path.name,
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return frame_path


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
                "sudo apt install -y python3-picamera2, then recreate the venv with "
                "python3 -m venv --system-site-packages .venv so apt-installed "
                "camera packages are visible."
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
