from lidar_room_mapper.sensors.lidar import build_payload_command, parse_standard_scan_packet


def make_packet(
    angle_deg: float,
    distance_mm: float,
    quality: int = 31,
    start_flag: bool = False,
) -> bytes:
    first = (quality << 2) | (0x01 if start_flag else 0x02)
    angle_q6 = int(round(angle_deg * 64.0))
    distance_q2 = int(round(distance_mm * 4.0))
    second = ((angle_q6 << 1) & 0xFF) | 0x01
    third = (angle_q6 >> 7) & 0xFF
    fourth = distance_q2 & 0xFF
    fifth = (distance_q2 >> 8) & 0xFF
    return bytes([first, second, third, fourth, fifth])


def test_parse_standard_scan_packet() -> None:
    measurement = parse_standard_scan_packet(
        make_packet(angle_deg=123.25, distance_mm=987.5, start_flag=True),
        timestamp=123.0,
    )

    assert measurement is not None
    assert measurement.start_flag is True
    assert measurement.quality == 31
    assert measurement.angle_deg == 123.25
    assert measurement.distance_mm == 987.5
    assert measurement.timestamp == 123.0


def test_rejects_invalid_start_bits() -> None:
    packet = bytes([0, 1, 0, 0, 0])

    assert parse_standard_scan_packet(packet) is None


def test_builds_set_pwm_payload_command() -> None:
    assert build_payload_command(0xF0, bytes([0x94, 0x02])) == bytes(
        [0xA5, 0xF0, 0x02, 0x94, 0x02, 0xC1]
    )
    assert build_payload_command(0xF0, bytes([0x00, 0x00])) == bytes(
        [0xA5, 0xF0, 0x02, 0x00, 0x00, 0x57]
    )
