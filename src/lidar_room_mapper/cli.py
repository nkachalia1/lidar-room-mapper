from __future__ import annotations

import argparse
import sys
from dataclasses import replace
from pathlib import Path

from lidar_room_mapper.config import MapConfig, RuntimeConfig
from lidar_room_mapper.dashboard.server import DashboardServer
from lidar_room_mapper.fusion import LidarCameraProjector
from lidar_room_mapper.mapping import OccupancyGrid, ScanMatcher, export_grid
from lidar_room_mapper.models import LidarScan, Pose2D
from lidar_room_mapper.perception import ClearanceAnalyzer
from lidar_room_mapper.runtime import MappingRuntime
from lidar_room_mapper.sensors.camera import (
    NullCamera,
    PiCameraCapture,
    ReplayCameraFrames,
    TimestampedCameraRecorder,
)
from lidar_room_mapper.sensors.lidar import (
    ReplayScanner,
    RplidarScanner,
    SimulatedScanner,
    scan_to_json,
)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="lidar-room-mapper",
        description="Map a room with RPLIDAR A1M8 and a Raspberry Pi camera.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    serve = subparsers.add_parser("serve", help="Run the live dashboard")
    add_source_args(serve)
    serve.add_argument("--host", default="127.0.0.1")
    serve.add_argument("--http-port", type=int, default=8000)
    serve.add_argument("--camera", action="store_true", help="Enable Pi Camera snapshots")
    serve.add_argument(
        "--frames",
        default="",
        help="Camera frame manifest JSONL to synchronize with replayed scans",
    )
    serve.add_argument(
        "--overlay",
        action="store_true",
        help="Project the paired LiDAR scan over each camera frame",
    )
    serve.add_argument(
        "--camera-intrinsics",
        default="config/camera_intrinsics_pi_camera_v2_1920x1080.json",
        help="Camera intrinsics JSON used by --overlay",
    )
    serve.add_argument(
        "--rig-config",
        default="config/rig_geometry.json",
        help="LiDAR-to-camera geometry JSON used by --overlay",
    )
    serve.add_argument("--camera-forward-m", type=float, default=None)
    serve.add_argument("--camera-left-m", type=float, default=None)
    serve.add_argument("--camera-up-m", type=float, default=None)
    serve.add_argument("--camera-yaw-deg", type=float, default=None)
    serve.add_argument("--camera-pitch-deg", type=float, default=None)
    serve.add_argument("--camera-roll-deg", type=float, default=None)
    serve.add_argument("--lidar-angle-offset-deg", type=float, default=None)
    serve.add_argument(
        "--front-angle-offset-deg",
        type=float,
        default=None,
        help=(
            "LiDAR angle offset that makes 0 degrees point forward for clearance; "
            "defaults to the overlay rig offset when available"
        ),
    )
    serve.set_defaults(func=serve_command)

    record = subparsers.add_parser("record", help="Record scans to JSONL")
    add_source_args(record)
    record.add_argument("--output", required=True)
    record.add_argument("--limit", type=int, default=0, help="Stop after N scans; 0 means forever")
    record.add_argument("--camera", action="store_true", help="Capture camera frames while recording")
    record.add_argument(
        "--camera-every",
        type=int,
        default=10,
        help="Capture one camera frame every N scans when --camera is enabled",
    )
    record.set_defaults(func=record_command)

    scan_once = subparsers.add_parser("scan-once", help="Integrate one scan and print map stats")
    add_source_args(scan_once)
    scan_once.set_defaults(func=scan_once_command)

    export_map = subparsers.add_parser("export-map", help="Export a map from scans")
    add_source_args(export_map)
    export_map.add_argument("--output", default="artifacts/map", help="Output path prefix")
    export_map.add_argument("--scans", type=int, default=100, help="Number of scans to integrate")
    export_map.add_argument(
        "--pose-mode",
        choices=("fixed", "scan-match"),
        default="fixed",
        help="How to place scans into the map",
    )
    export_map.set_defaults(func=export_map_command)

    scan_match = subparsers.add_parser(
        "scan-match", help="Estimate relative motion between consecutive scans"
    )
    add_source_args(scan_match)
    scan_match.add_argument("--scans", type=int, default=20, help="Number of scans to inspect")
    scan_match.set_defaults(func=scan_match_command)

    return parser


def add_source_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--source",
        choices=("sim", "replay", "rplidar"),
        default="sim",
        help="LiDAR source",
    )
    parser.add_argument("--input", default="data/sample_scan.jsonl", help="Replay JSONL file")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="RPLIDAR serial port")
    parser.add_argument("--baud", type=int, default=115200, help="RPLIDAR baud rate")


def serve_command(args: argparse.Namespace) -> int:
    scanner = make_scanner(args)
    camera = make_dashboard_camera(args)
    projector = make_dashboard_projector(args)
    clearance_analyzer = make_clearance_analyzer(args, projector)
    runtime = MappingRuntime(
        scanner=scanner,
        camera=camera,
        projector=projector,
        clearance_analyzer=clearance_analyzer,
        map_config=MapConfig(),
        runtime_config=RuntimeConfig(serial_baud=args.baud),
    )
    runtime.start()
    server = DashboardServer((args.host, args.http_port), runtime)
    print(f"Dashboard running at http://{args.host}:{args.http_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping dashboard...")
    finally:
        runtime.stop()
        server.server_close()
    return 0


def make_dashboard_camera(args: argparse.Namespace):
    if args.frames:
        return ReplayCameraFrames(args.frames)
    if args.camera:
        return PiCameraCapture()
    if args.source == "replay":
        manifest_path = default_frame_manifest_path(Path(args.input))
        if manifest_path.exists():
            print(f"Using replay camera frames from {manifest_path}")
            return ReplayCameraFrames(manifest_path)
    return NullCamera()


def make_dashboard_projector(args: argparse.Namespace) -> LidarCameraProjector | None:
    if not args.overlay:
        return None
    try:
        projector = LidarCameraProjector.from_json_files(
            args.camera_intrinsics,
            args.rig_config,
        )
    except (OSError, ValueError) as exc:
        raise SystemExit(f"Unable to load LiDAR-camera calibration: {exc}") from exc

    override_names = {
        "camera_forward_m": args.camera_forward_m,
        "camera_left_m": args.camera_left_m,
        "camera_up_m": args.camera_up_m,
        "camera_yaw_deg": args.camera_yaw_deg,
        "camera_pitch_deg": args.camera_pitch_deg,
        "camera_roll_deg": args.camera_roll_deg,
        "lidar_angle_offset_deg": args.lidar_angle_offset_deg,
    }
    overrides = {name: value for name, value in override_names.items() if value is not None}
    if overrides:
        projector = LidarCameraProjector(
            intrinsics=projector.intrinsics,
            rig=replace(projector.rig, **overrides),
        )

    rig = projector.rig
    print(
        "LiDAR-camera overlay enabled: "
        f"translation=({rig.camera_forward_m:.5f}, {rig.camera_left_m:.5f}, "
        f"{rig.camera_up_m:.5f})m "
        f"yaw/pitch/roll=({rig.camera_yaw_deg:.1f}, {rig.camera_pitch_deg:.1f}, "
        f"{rig.camera_roll_deg:.1f})deg "
        f"lidar_offset={rig.lidar_angle_offset_deg:.1f}deg"
    )
    return projector


def make_clearance_analyzer(
    args: argparse.Namespace,
    projector: LidarCameraProjector | None,
) -> ClearanceAnalyzer:
    angle_offset_deg = args.front_angle_offset_deg
    if angle_offset_deg is None and projector is not None:
        angle_offset_deg = projector.rig.lidar_angle_offset_deg
    return ClearanceAnalyzer(angle_offset_deg=angle_offset_deg or 0.0)


def default_frame_manifest_path(replay_path: Path) -> Path:
    return replay_path.with_name(f"{replay_path.stem}_frames.jsonl")


def record_command(args: argparse.Namespace) -> int:
    if args.camera_every <= 0:
        raise SystemExit("--camera-every must be greater than zero")

    scanner = make_scanner(args)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    camera = make_recording_camera(output) if args.camera else NullCamera()
    count = 0
    try:
        with output.open("w", encoding="utf-8") as handle:
            for scan in scanner.iter_scans():
                handle.write(scan_to_json(scan) + "\n")
                count += 1
                frame = None
                if args.camera and (count - 1) % args.camera_every == 0:
                    frame = camera.capture()
                suffix = f", frame={frame.path}" if frame is not None else ""
                print(f"recorded scan {count}: {len(scan.measurements)} points{suffix}")
                if args.limit and count >= args.limit:
                    break
    finally:
        scanner.close()
        camera.close()
    return 0


def make_recording_camera(output: Path) -> TimestampedCameraRecorder:
    frame_dir = output.with_suffix("").parent / f"{output.with_suffix('').name}_frames"
    manifest_path = output.with_suffix("").parent / f"{output.with_suffix('').name}_frames.jsonl"
    return TimestampedCameraRecorder(
        PiCameraCapture(output_dir=frame_dir),
        manifest_path=manifest_path,
    )


def scan_once_command(args: argparse.Namespace) -> int:
    scanner = make_scanner(args)
    grid = OccupancyGrid(MapConfig())
    try:
        scan = next(iter(scanner.iter_scans()))
        grid.integrate_scan(scan)
    finally:
        scanner.close()

    stats = grid.stats()
    print(f"source={args.source}")
    print(f"points={len(scan.measurements)}")
    print(f"occupied_cells={stats.occupied_cells}")
    print(f"free_cells={stats.free_cells}")
    print(f"unknown_cells={stats.unknown_cells}")
    return 0


def export_map_command(args: argparse.Namespace) -> int:
    if args.scans <= 0:
        raise SystemExit("--scans must be greater than zero")

    scanner = make_scanner(args, replay_loop=False, replay_scan_hz=1000.0)
    grid = OccupancyGrid(MapConfig())
    integrated = 0
    pose = Pose2D()
    previous_scan: LidarScan | None = None
    matcher = ScanMatcher() if args.pose_mode == "scan-match" else None
    try:
        for scan in scanner.iter_scans():
            if matcher is not None and previous_scan is not None:
                result = matcher.match(previous_scan, scan)
                if result.accepted:
                    pose = pose.compose(result.delta_pose)
            grid.integrate_scan_at_pose(scan, pose)
            previous_scan = scan
            integrated += 1
            if integrated >= args.scans:
                break
    finally:
        scanner.close()

    if integrated == 0:
        raise SystemExit("No scans were available to export.")

    paths = export_grid(grid, args.output)
    stats = grid.stats()
    print(f"integrated_scans={integrated}")
    print(f"pose_mode={args.pose_mode}")
    print(f"final_pose_m=({pose.x_m:.3f},{pose.y_m:.3f})")
    print(f"final_heading_deg={pose.theta_rad * 57.29577951308232:.2f}")
    print(f"occupied_cells={stats.occupied_cells}")
    print(f"free_cells={stats.free_cells}")
    print(f"png={paths.png}")
    print(f"pgm={paths.pgm}")
    print(f"yaml={paths.yaml}")
    return 0


def scan_match_command(args: argparse.Namespace) -> int:
    if args.scans <= 1:
        raise SystemExit("--scans must be greater than one")

    scanner = make_scanner(args, replay_loop=False, replay_scan_hz=1000.0)
    matcher = ScanMatcher()
    pose = Pose2D()
    previous_scan: LidarScan | None = None
    inspected = 0
    try:
        for scan in scanner.iter_scans():
            inspected += 1
            if previous_scan is not None:
                result = matcher.match(previous_scan, scan)
                if result.accepted:
                    pose = pose.compose(result.delta_pose)
                print(
                    "scan={scan} dx_m={dx:.3f} dy_m={dy:.3f} dtheta_deg={theta:.2f} "
                    "score={score:.5f} accepted={accepted} "
                    "pose=({x:.3f},{y:.3f},{heading:.2f}deg)".format(
                        scan=inspected,
                        dx=result.delta_pose.x_m,
                        dy=result.delta_pose.y_m,
                        theta=result.heading_deg,
                        score=result.score,
                        accepted=str(result.accepted).lower(),
                        x=pose.x_m,
                        y=pose.y_m,
                        heading=pose.theta_rad * 57.29577951308232,
                    )
                )
            previous_scan = scan
            if inspected >= args.scans:
                break
    finally:
        scanner.close()

    if inspected < 2:
        raise SystemExit("Need at least two scans to estimate motion.")
    return 0


def make_scanner(
    args: argparse.Namespace,
    replay_loop: bool = True,
    replay_scan_hz: float = 4.0,
):
    if args.source == "sim":
        return SimulatedScanner()
    if args.source == "replay":
        replay_path = Path(args.input)
        if not replay_path.exists():
            raise SystemExit(f"Replay input not found: {replay_path}")
        return ReplayScanner(replay_path, loop=replay_loop, scan_hz=replay_scan_hz)
    if args.source == "rplidar":
        return RplidarScanner(port=args.port, baudrate=args.baud)
    raise SystemExit(f"Unsupported source: {args.source}")


if __name__ == "__main__":
    sys.exit(main())
