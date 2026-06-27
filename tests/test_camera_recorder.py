import json
from pathlib import Path

from lidar_room_mapper.models import CameraFrame
from lidar_room_mapper.sensors.camera import ReplayCameraFrames, TimestampedCameraRecorder


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


def test_replay_camera_frames_returns_nearest_frame(tmp_path: Path) -> None:
    captures = tmp_path / "captures"
    frame_dir = captures / "session_frames"
    frame_dir.mkdir(parents=True)
    frame1 = frame_dir / "frame_000001.jpg"
    frame2 = frame_dir / "frame_000002.jpg"
    frame1.write_bytes(b"first")
    frame2.write_bytes(b"second")

    manifest = captures / "session_frames.jsonl"
    records = [
        {
            "path": str(Path("captures") / "session_frames" / "frame_000001.jpg"),
            "timestamp": 10.0,
            "width": 640,
            "height": 480,
        },
        {
            "path": str(Path("captures") / "session_frames" / "frame_000002.jpg"),
            "timestamp": 20.0,
            "width": 640,
            "height": 480,
        },
    ]
    manifest.write_text(
        "\n".join(json.dumps(record) for record in records) + "\n",
        encoding="utf-8",
    )

    camera = ReplayCameraFrames(manifest)
    selected = camera.capture_near(18.0)

    assert selected is not None
    assert Path(selected.path) == frame2
    assert selected.timestamp == 20.0
