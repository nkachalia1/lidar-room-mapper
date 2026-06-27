import json
from pathlib import Path

import pytest

from lidar_room_mapper.fusion import (
    CameraIntrinsics,
    LidarCameraProjector,
    RigExtrinsics,
)
from lidar_room_mapper.models import LidarMeasurement, LidarScan


def test_nominal_forward_lidar_point_projects_to_image_center() -> None:
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
        ),
    )
    scan = LidarScan((LidarMeasurement(angle_deg=0.0, distance_mm=2000.0),))

    result = projector.project_scan(scan, width=100, height=80)

    assert len(result.points) == 1
    assert result.points[0].u == pytest.approx(50.0)
    assert result.points[0].v == pytest.approx(40.0)


def test_camera_height_moves_scan_plane_below_image_center() -> None:
    projector = LidarCameraProjector(
        CameraIntrinsics(
            width=100,
            height=100,
            fx=100.0,
            fy=100.0,
            cx=50.0,
            cy=50.0,
            distortion=(0.0, 0.0, 0.0, 0.0, 0.0),
        ),
        RigExtrinsics(
            camera_forward_m=0.0,
            camera_left_m=0.0,
            camera_up_m=0.5,
            camera_yaw_deg=0.0,
            camera_pitch_deg=0.0,
            camera_roll_deg=0.0,
            lidar_angle_offset_deg=0.0,
            lidar_height_m=0.0,
        ),
    )
    scan = LidarScan((LidarMeasurement(angle_deg=0.0, distance_mm=2000.0),))

    result = projector.project_scan(scan, width=100, height=100)

    assert result.points[0].v == pytest.approx(75.0)


def test_intrinsics_scale_to_same_aspect_ratio() -> None:
    intrinsics = CameraIntrinsics(
        width=1920,
        height=1080,
        fx=1200.0,
        fy=1180.0,
        cx=960.0,
        cy=540.0,
        distortion=(0.0, 0.0, 0.0, 0.0, 0.0),
    )

    scaled = intrinsics.scaled_to(1280, 720)

    assert scaled.fx == pytest.approx(800.0)
    assert scaled.fy == pytest.approx(1180.0 * 2.0 / 3.0)
    assert scaled.cx == pytest.approx(640.0)
    assert scaled.cy == pytest.approx(360.0)


def test_loads_calibration_and_rig_json(tmp_path: Path) -> None:
    intrinsics_path = tmp_path / "intrinsics.json"
    rig_path = tmp_path / "rig.json"
    intrinsics_path.write_text(
        json.dumps(
            {
                "calibration_image_size": {"width": 640, "height": 480},
                "camera_matrix": {
                    "data": [500, 0, 320, 0, 501, 240, 0, 0, 1]
                },
                "distortion_coefficients": {"data": [0, 0, 0, 0, 0]},
                "rms_reprojection_error_px": 0.4,
            }
        ),
        encoding="utf-8",
    )
    rig_path.write_text(
        json.dumps(
            {
                "status": "test",
                "lidar": {"height_above_ground_m": 0.05, "angle_offset_deg": 12},
                "lidar_to_camera": {
                    "translation_m": {"forward": -0.1, "left": 0.01, "up": 0.2},
                    "rotation_deg": {"yaw": 1, "pitch": 2, "roll": 3},
                },
            }
        ),
        encoding="utf-8",
    )

    projector = LidarCameraProjector.from_json_files(intrinsics_path, rig_path)

    assert projector.intrinsics.fx == 500.0
    assert projector.intrinsics.rms_reprojection_error_px == 0.4
    assert projector.rig.camera_forward_m == -0.1
    assert projector.rig.lidar_angle_offset_deg == 12.0
    assert projector.rig.status == "test"


def test_validated_rig_projects_48_inch_reference_onto_target_stripe() -> None:
    project_root = Path(__file__).parents[1]
    projector = LidarCameraProjector.from_json_files(
        project_root / "config" / "camera_intrinsics_pi_camera_v2_1920x1080.json",
        project_root / "config" / "rig_geometry.json",
    )
    scan = LidarScan(
        (
            LidarMeasurement(
                angle_deg=90.72,
                distance_mm=1229.0,
                quality=15,
            ),
        )
    )

    result = projector.project_scan(scan, width=1280, height=720)

    assert len(result.points) == 1
    assert result.points[0].u == pytest.approx(851.3, abs=0.5)
    assert result.points[0].v == pytest.approx(297.7, abs=0.5)


def test_validated_rig_projects_36_inch_holdout_onto_target_stripe() -> None:
    project_root = Path(__file__).parents[1]
    projector = LidarCameraProjector.from_json_files(
        project_root / "config" / "camera_intrinsics_pi_camera_v2_1920x1080.json",
        project_root / "config" / "rig_geometry.json",
    )
    scan = LidarScan(
        (
            LidarMeasurement(
                angle_deg=90.55,
                distance_mm=905.0,
                quality=15,
            ),
        )
    )

    result = projector.project_scan(scan, width=1280, height=720)

    assert len(result.points) == 1
    assert result.points[0].u == pytest.approx(845.6, abs=0.5)
    assert result.points[0].v == pytest.approx(357.4, abs=0.5)
