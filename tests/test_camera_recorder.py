from pathlib import Path

from lidar_room_mapper.models import CameraFrame
from lidar_room_mapper.sensors.camera import TimestampedCameraRecorder


class FakeCamera:
    def __init__(self, output_dir: Path):
        self.output_dir = output_dir
        self.filename = "latest.jpg"
        self.closed = False

    def capture(self) -> CameraFrame:
        path = self.output_dir / self.filename
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"fake-jpeg")
        return CameraFrame(
            path=str(path),
            timestamp=123.0,
            width=640,
            height=480,
        )

    def close(self) -> None:
        self.closed = True


def test_timestamped_camera_recorder_writes_frames_and_manifest(tmp_path: Path) -> None:
    camera = FakeCamera(tmp_path / "frames")
    recorder = TimestampedCameraRecorder(camera, tmp_path / "frames.jsonl")

    frame1 = recorder.capture()
    frame2 = recorder.capture()
    recorder.close()

    assert frame1 is not None
    assert frame1.path.endswith("frame_000001.jpg")
    assert frame2 is not None
    assert frame2.path.endswith("frame_000002.jpg")
    assert camera.closed is True
    assert (tmp_path / "frames" / "frame_000001.jpg").read_bytes() == b"fake-jpeg"
    manifest = (tmp_path / "frames.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(manifest) == 2
    assert "frame_000001.jpg" in manifest[0]
    assert "frame_000002.jpg" in manifest[1]
