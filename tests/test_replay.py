from pathlib import Path

from lidar_room_mapper.sensors.lidar import ReplayScanner


def test_replay_scanner_reads_sample_data() -> None:
    scanner = ReplayScanner(Path("data/sample_scan.jsonl"), loop=False, scan_hz=1000.0)
    scan = next(iter(scanner.iter_scans()))

    assert scan.source == "sample"
    assert len(scan.measurements) == 36
    assert scan.measurements[0].start_flag is True
