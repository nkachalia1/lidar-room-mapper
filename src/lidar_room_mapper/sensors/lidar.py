from __future__ import annotations

import json
import math
import random
import time
from dataclasses import asdict
from pathlib import Path
from typing import Iterable, Protocol

from lidar_room_mapper.models import LidarMeasurement, LidarScan


class Scanner(Protocol):
    def iter_scans(self) -> Iterable[LidarScan]:
        ...

    def close(self) -> None:
        ...


class SimulatedScanner:
    """Deterministic room-like LiDAR source for demos and development."""

    def __init__(self, scan_hz: float = 3.0, points_per_scan: int = 180, seed: int = 7):
        self.scan_hz = scan_hz
        self.points_per_scan = points_per_scan
        self._rng = random.Random(seed)
        self._closed = False
        self._phase = 0.0

    def iter_scans(self) -> Iterable[LidarScan]:
        while not self._closed:
            started = time.time()
            measurements: list[LidarMeasurement] = []
            for index in range(self.points_per_scan):
                angle = index * (360.0 / self.points_per_scan)
                distance = self._distance_for_angle(angle)
                noise = self._rng.uniform(-18.0, 18.0)
                measurements.append(
                    LidarMeasurement(
                        angle_deg=angle,
                        distance_mm=max(120.0, distance + noise),
                        quality=28,
                        start_flag=index == 0,
                    )
                )
            self._phase = (self._phase + 0.07) % (math.pi * 2.0)
            yield LidarScan(tuple(measurements), source="sim")
            elapsed = time.time() - started
            time.sleep(max(0.0, (1.0 / self.scan_hz) - elapsed))

    def close(self) -> None:
        self._closed = True

    def _distance_for_angle(self, angle_deg: float) -> float:
        angle_rad = math.radians(angle_deg)
        room_half_x = 2650.0
        room_half_y = 1850.0
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
        candidates: list[float] = []
        if abs(dx) > 1e-6:
            candidates.extend([room_half_x / dx, -room_half_x / dx])
        if abs(dy) > 1e-6:
            candidates.extend([room_half_y / dy, -room_half_y / dy])
        wall_distance = min(distance for distance in candidates if distance > 0)

        table_center = math.radians(42.0 + math.sin(self._phase) * 3.0)
        table_width = math.radians(12.0)
        if abs(_angle_delta(angle_rad, table_center)) < table_width:
            return 1250.0

        shelf_center = math.radians(226.0)
        shelf_width = math.radians(9.0)
        if abs(_angle_delta(angle_rad, shelf_center)) < shelf_width:
            return 920.0

        doorway_center = math.radians(90.0)
        if abs(_angle_delta(angle_rad, doorway_center)) < math.radians(7.0):
            return 5200.0

        return wall_distance


class ReplayScanner:
    """Replay scans from newline-delimited JSON."""

    def __init__(self, path: str | Path, loop: bool = True, scan_hz: float = 4.0):
        self.path = Path(path)
        self.loop = loop
        self.scan_hz = scan_hz
        self._closed = False

    def iter_scans(self) -> Iterable[LidarScan]:
        while not self._closed:
            yielded_any = False
            with self.path.open("r", encoding="utf-8") as handle:
                for line in handle:
                    if self._closed:
                        return
                    line = line.strip()
                    if not line:
                        continue
                    yielded_any = True
                    yield scan_from_json(line)
                    time.sleep(max(0.0, 1.0 / self.scan_hz))
            if not self.loop or not yielded_any:
                return

    def close(self) -> None:
        self._closed = True


class RplidarScanner:
    """Slamtec RPLIDAR A-series scanner using the standard serial protocol."""

    SYNC_BYTE = 0xA5
    RESET = 0x40
    STOP = 0x25
    SCAN = 0x20
    SET_PWM = 0xF0
    DEFAULT_MOTOR_PWM = 660

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        motor_pwm: int = DEFAULT_MOTOR_PWM,
    ):
        try:
            import serial
        except ImportError as exc:  # pragma: no cover - exercised on Pi
            raise RuntimeError(
                "pyserial is required for --source rplidar. Install with: "
                "python -m pip install -e '.[hardware]'"
            ) from exc

        self._serial = serial.Serial(port, baudrate=baudrate, timeout=2.0)
        self.motor_pwm = motor_pwm
        self._closed = False

    def iter_scans(self) -> Iterable[LidarScan]:
        self._send_command(self.STOP)
        time.sleep(0.05)
        self.start_motor()
        self._send_command(self.SCAN)
        self._read_descriptor()

        measurements: list[LidarMeasurement] = []
        consecutive_timeouts = 0
        while not self._closed:
            packet = self._serial.read(5)
            if len(packet) != 5:
                consecutive_timeouts += 1
                if consecutive_timeouts >= 5:
                    raise RuntimeError(
                        "Timed out waiting for RPLIDAR scan packets. Check USB power, "
                        "the serial port, and whether the sensor is spinning."
                    )
                continue
            consecutive_timeouts = 0
            measurement = parse_standard_scan_packet(packet, time.time())
            if measurement is None:
                continue
            if measurement.start_flag and measurements:
                yield LidarScan(tuple(measurements), source="rplidar")
                measurements = []
            measurements.append(measurement)

    def close(self) -> None:
        self._closed = True
        try:
            self._send_command(self.STOP)
        finally:
            try:
                self.stop_motor()
            finally:
                self._serial.close()

    def start_motor(self) -> None:
        self._serial.setDTR(False)
        self._send_payload_command(self.SET_PWM, _uint16le(self.motor_pwm))
        time.sleep(1.5)

    def stop_motor(self) -> None:
        self._send_payload_command(self.SET_PWM, _uint16le(0))
        self._serial.setDTR(True)

    def _send_command(self, command: int) -> None:
        self._serial.write(bytes([self.SYNC_BYTE, command]))

    def _send_payload_command(self, command: int, payload: bytes) -> None:
        self._serial.write(build_payload_command(command, payload))

    def _read_descriptor(self) -> bytes:
        descriptor = self._serial.read(7)
        if len(descriptor) != 7 or descriptor[0] != 0xA5 or descriptor[1] != 0x5A:
            raise RuntimeError(
                "RPLIDAR did not return a valid scan descriptor. Check that the "
                "port is correct, the sensor has enough power, and the motor is spinning."
            )
        return descriptor


def parse_standard_scan_packet(
    packet: bytes, timestamp: float | None = None
) -> LidarMeasurement | None:
    """Parse the 5-byte standard RPLIDAR measurement response."""

    if len(packet) != 5:
        raise ValueError("Standard RPLIDAR scan packets are exactly 5 bytes.")

    start_flag = bool(packet[0] & 0x01)
    inverted_start_flag = bool(packet[0] & 0x02)
    if start_flag == inverted_start_flag:
        return None

    check_bit = packet[1] & 0x01
    if check_bit != 1:
        return None

    quality = packet[0] >> 2
    angle_q6 = ((packet[2] << 8) | packet[1]) >> 1
    distance_q2 = (packet[4] << 8) | packet[3]
    return LidarMeasurement(
        angle_deg=angle_q6 / 64.0,
        distance_mm=distance_q2 / 4.0,
        quality=quality,
        start_flag=start_flag,
        timestamp=timestamp or time.time(),
    )


def scan_to_json(scan: LidarScan) -> str:
    payload = {
        "timestamp": scan.timestamp,
        "source": scan.source,
        "measurements": [asdict(measurement) for measurement in scan.measurements],
    }
    return json.dumps(payload, separators=(",", ":"))


def scan_from_json(line: str) -> LidarScan:
    payload = json.loads(line)
    measurements = tuple(
        LidarMeasurement(
            angle_deg=float(item["angle_deg"]),
            distance_mm=float(item["distance_mm"]),
            quality=int(item.get("quality", 0)),
            start_flag=bool(item.get("start_flag", False)),
            timestamp=float(item.get("timestamp", payload.get("timestamp", time.time()))),
        )
        for item in payload["measurements"]
    )
    return LidarScan(
        measurements=measurements,
        timestamp=float(payload.get("timestamp", time.time())),
        source=str(payload.get("source", "replay")),
    )


def build_payload_command(command: int, payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("RPLIDAR payload commands can carry at most 255 bytes.")
    checksum = RplidarScanner.SYNC_BYTE ^ command ^ len(payload)
    for byte in payload:
        checksum ^= byte
    return bytes([RplidarScanner.SYNC_BYTE, command, len(payload), *payload, checksum])


def _uint16le(value: int) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise ValueError("uint16 value out of range")
    return bytes([value & 0xFF, (value >> 8) & 0xFF])


def _angle_delta(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi
