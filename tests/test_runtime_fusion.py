from lidar_room_mapper.fusion import (
    CameraIntrinsics,
    LidarCameraProjector,
    RigExtrinsics,
)
from lidar_room_mapper.models import CameraFrame, LidarMeasurement, LidarScan
from lidar_room_mapper.runtime import MappingRuntime


class OneScanScanner:
    def __init__(self, scan: LidarScan):
        self.scan = scan

    def iter_scans(self):
        yield self.scan

    def close(self) -> None:
        return None


class OneFrameCamera:
    def __init__(self, frame: CameraFrame):
        self.frame = frame

    def capture(self) -> CameraFrame:
        return self.frame

    def close(self) -> None:
        return None


def test_runtime_pairs_projection_with_captured_frame() -> None:
    scan = LidarScan(
        measurements=(LidarMeasurement(angle_deg=0.0, distance_mm=2000.0),),
        timestamp=100.0,
        source="test",
    )
    frame = CameraFrame(
        path="frame.jpg",
        timestamp=100.02,
        width=100,
        height=80,
    )
    projector = LidarCameraProjector(
        CameraIntrinsics(
            width=100,
            height=80,
            fx=100.0,
            fy=100.0,
            cx=50.0,
            cy=40.0,
            distortion=(0.0, 0.0, 0.0, 0.0, 0.0),
        ),
        RigExtrinsics(
            camera_forward_m=0.0,
            camera_left_m=0.0,
            camera_up_m=0.0,
            camera_yaw_deg=0.0,
            camera_pitch_deg=0.0,
            camera_roll_deg=0.0,
            lidar_angle_offset_deg=0.0,
            lidar_height_m=0.0,
            status="test calibration",
        ),
    )
    runtime = MappingRuntime(
        scanner=OneScanScanner(scan),
        camera=OneFrameCamera(frame),
        projector=projector,
    )

    runtime._run()
    state = runtime.snapshot()

    assert state["fusion"]["enabled"] is True
    assert state["fusion"]["calibration_status"] == "test calibration"
    assert state["fusion"]["projected_count"] == 1
    assert state["fusion"]["scan_timestamp"] == 100.0
    assert state["fusion"]["frame_timestamp"] == 100.02
    assert state["fusion"]["sync_delta_ms"] == -20.0
    assert state["clearance"]["status"] == "clear"
    assert state["clearance"]["front_clearance_m"] == 2.0
