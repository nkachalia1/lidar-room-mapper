from __future__ import annotations

import json
import math
from dataclasses import dataclass, replace
from pathlib import Path

from lidar_room_mapper.models import LidarScan


DistortionCoefficients = tuple[float, float, float, float, float]


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: DistortionCoefficients
    rms_reprojection_error_px: float | None = None

    @classmethod
    def from_json(cls, path: str | Path) -> "CameraIntrinsics":
        source = Path(path)
        payload = json.loads(source.read_text(encoding="utf-8"))
        try:
            size = payload["calibration_image_size"]
            matrix = payload["camera_matrix"]["data"]
            distortion = payload["distortion_coefficients"]["data"]
            if len(matrix) != 9 or len(distortion) < 5:
                raise ValueError("invalid matrix dimensions")
            return cls(
                width=int(size["width"]),
                height=int(size["height"]),
                fx=float(matrix[0]),
                fy=float(matrix[4]),
                cx=float(matrix[2]),
                cy=float(matrix[5]),
                distortion=tuple(float(value) for value in distortion[:5]),
                rms_reprojection_error_px=_optional_float(
                    payload.get("rms_reprojection_error_px")
                ),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Invalid camera intrinsics file: {source}") from exc

    def scaled_to(self, width: int, height: int) -> "CameraIntrinsics":
        if width <= 0 or height <= 0:
            raise ValueError("Camera frame dimensions must be positive")
        source_aspect = self.width / self.height
        target_aspect = width / height
        if not math.isclose(source_aspect, target_aspect, rel_tol=0.01):
            raise ValueError(
                "Camera frame aspect ratio does not match the calibrated sensor crop"
            )
        scale_x = width / self.width
        scale_y = height / self.height
        return replace(
            self,
            width=width,
            height=height,
            fx=self.fx * scale_x,
            fy=self.fy * scale_y,
            cx=self.cx * scale_x,
            cy=self.cy * scale_y,
        )


@dataclass(frozen=True)
class RigExtrinsics:
    camera_forward_m: float
    camera_left_m: float
    camera_up_m: float
    camera_yaw_deg: float
    camera_pitch_deg: float
    camera_roll_deg: float
    lidar_angle_offset_deg: float
    lidar_height_m: float
    status: str = "provisional"

    @classmethod
    def from_json(cls, path: str | Path) -> "RigExtrinsics":
        source = Path(path)
        payload = json.loads(source.read_text(encoding="utf-8"))
        try:
            lidar = payload["lidar"]
            transform = payload["lidar_to_camera"]
            translation = transform["translation_m"]
            rotation = transform["rotation_deg"]
            return cls(
                camera_forward_m=float(translation["forward"]),
                camera_left_m=float(translation["left"]),
                camera_up_m=float(translation["up"]),
                camera_yaw_deg=float(rotation["yaw"]),
                camera_pitch_deg=float(rotation["pitch"]),
                camera_roll_deg=float(rotation["roll"]),
                lidar_angle_offset_deg=float(lidar["angle_offset_deg"]),
                lidar_height_m=float(lidar["height_above_ground_m"]),
                status=str(payload.get("status", "provisional")),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Invalid rig geometry file: {source}") from exc


@dataclass(frozen=True)
class ProjectedLidarPoint:
    u: float
    v: float
    distance_m: float
    quality: int

    def to_payload(self) -> dict[str, float | int]:
        return {
            "u": round(self.u, 2),
            "v": round(self.v, 2),
            "distance_m": round(self.distance_m, 3),
            "quality": self.quality,
        }


@dataclass(frozen=True)
class ProjectionResult:
    points: tuple[ProjectedLidarPoint, ...]
    clipped_count: int
    behind_count: int
    filtered_count: int

    def to_payload(self) -> dict[str, object]:
        return {
            "points": [point.to_payload() for point in self.points],
            "projected_count": len(self.points),
            "clipped_count": self.clipped_count,
            "behind_count": self.behind_count,
            "filtered_count": self.filtered_count,
        }


class LidarCameraProjector:
    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        rig: RigExtrinsics,
        min_distance_m: float = 0.15,
        max_distance_m: float = 8.0,
    ) -> None:
        if min_distance_m < 0 or max_distance_m <= min_distance_m:
            raise ValueError("Invalid LiDAR projection distance limits")
        self.intrinsics = intrinsics
        self.rig = rig
        self.min_distance_m = min_distance_m
        self.max_distance_m = max_distance_m

    @classmethod
    def from_json_files(
        cls,
        intrinsics_path: str | Path,
        rig_path: str | Path,
    ) -> "LidarCameraProjector":
        return cls(
            CameraIntrinsics.from_json(intrinsics_path),
            RigExtrinsics.from_json(rig_path),
        )

    def project_scan(self, scan: LidarScan, width: int, height: int) -> ProjectionResult:
        intrinsics = self.intrinsics.scaled_to(width, height)
        points: list[ProjectedLidarPoint] = []
        clipped_count = 0
        behind_count = 0
        filtered_count = 0

        for measurement in scan.valid_measurements():
            distance_m = measurement.distance_m
            if not self.min_distance_m <= distance_m <= self.max_distance_m:
                filtered_count += 1
                continue

            angle_rad = math.radians(
                measurement.angle_deg + self.rig.lidar_angle_offset_deg
            )
            point_lidar = (
                distance_m * math.cos(angle_rad),
                distance_m * math.sin(angle_rad),
                0.0,
            )
            point_camera = _lidar_to_camera(point_lidar, self.rig)
            projected = _project_camera_point(point_camera, intrinsics)
            if projected is None:
                behind_count += 1
                continue
            u, v = projected
            if not 0 <= u < width or not 0 <= v < height:
                clipped_count += 1
                continue
            points.append(
                ProjectedLidarPoint(
                    u=u,
                    v=v,
                    distance_m=distance_m,
                    quality=measurement.quality,
                )
            )

        return ProjectionResult(
            points=tuple(points),
            clipped_count=clipped_count,
            behind_count=behind_count,
            filtered_count=filtered_count,
        )


def _lidar_to_camera(
    point_lidar: tuple[float, float, float],
    rig: RigExtrinsics,
) -> tuple[float, float, float]:
    x_lidar, y_lidar, z_lidar = point_lidar
    relative_forward = x_lidar - rig.camera_forward_m
    relative_left = y_lidar - rig.camera_left_m
    relative_up = z_lidar - rig.camera_up_m

    x_camera = -relative_left
    y_camera = -relative_up
    z_camera = relative_forward

    yaw = math.radians(rig.camera_yaw_deg)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x_camera, z_camera = (
        cos_yaw * x_camera + sin_yaw * z_camera,
        -sin_yaw * x_camera + cos_yaw * z_camera,
    )

    pitch = math.radians(rig.camera_pitch_deg)
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    y_camera, z_camera = (
        cos_pitch * y_camera - sin_pitch * z_camera,
        sin_pitch * y_camera + cos_pitch * z_camera,
    )

    roll = math.radians(rig.camera_roll_deg)
    cos_roll = math.cos(roll)
    sin_roll = math.sin(roll)
    x_camera, y_camera = (
        cos_roll * x_camera - sin_roll * y_camera,
        sin_roll * x_camera + cos_roll * y_camera,
    )
    return x_camera, y_camera, z_camera


def _project_camera_point(
    point_camera: tuple[float, float, float],
    intrinsics: CameraIntrinsics,
) -> tuple[float, float] | None:
    x_camera, y_camera, z_camera = point_camera
    if z_camera <= 0.01:
        return None

    x_normalized = x_camera / z_camera
    y_normalized = y_camera / z_camera
    k1, k2, p1, p2, k3 = intrinsics.distortion
    radius_squared = x_normalized**2 + y_normalized**2
    radial = (
        1.0
        + k1 * radius_squared
        + k2 * radius_squared**2
        + k3 * radius_squared**3
    )
    x_tangential = (
        2.0 * p1 * x_normalized * y_normalized
        + p2 * (radius_squared + 2.0 * x_normalized**2)
    )
    y_tangential = (
        p1 * (radius_squared + 2.0 * y_normalized**2)
        + 2.0 * p2 * x_normalized * y_normalized
    )
    x_distorted = x_normalized * radial + x_tangential
    y_distorted = y_normalized * radial + y_tangential
    return (
        intrinsics.fx * x_distorted + intrinsics.cx,
        intrinsics.fy * y_distorted + intrinsics.cy,
    )


def _optional_float(value: object) -> float | None:
    return float(value) if value is not None else None
