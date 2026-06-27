# Pi LiDAR Room Mapper

An interview-ready robotics project for a Raspberry Pi 5, Raspberry Pi Camera Module v2, and Slamtec RPLIDAR A1M8.

The project builds a live 2D occupancy-grid map from LiDAR scans, optionally captures synchronized camera snapshots, and serves a browser dashboard for demos. It is designed to work in three modes:

- `sim`: deterministic synthetic scans for development without hardware.
- `replay`: recorded JSONL scans for repeatable demos and tests.
- `rplidar`: live data from the Slamtec RPLIDAR A1M8 over serial.

## Why This Is Interview-Ready

- Clear sensor abstraction for simulation, replay, and real hardware.
- Occupancy-grid mapping with ray tracing and log-odds updates.
- Live dashboard that can run directly on the Pi.
- Data capture/replay workflow for reproducible debugging.
- Hardware setup, architecture notes, and interview talking points.
- Tests around the math and protocol parsing.

## Quick Start

```powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
.\.venv\Scripts\python.exe -m pytest
.\.venv\Scripts\python.exe -m lidar_room_mapper serve --source sim --http-port 8000
```

Then open [http://127.0.0.1:8000](http://127.0.0.1:8000).

On a Raspberry Pi:

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -e ".[hardware]"
python -m lidar_room_mapper serve --source rplidar --port /dev/ttyUSB0 --camera
```

## Common Commands

Run the dashboard with simulated scans:

```bash
python -m lidar_room_mapper serve --source sim
```

Run from a replay file:

```bash
python -m lidar_room_mapper serve --source replay --input data/sample_scan.jsonl
```

Record live LiDAR scans:

```bash
python -m lidar_room_mapper record --source rplidar --port /dev/ttyUSB0 --output captures/session.jsonl
```

Print one integrated map summary:

```bash
python -m lidar_room_mapper scan-once --source sim
```

## Hardware

- Raspberry Pi 5, 8 GB
- Raspberry Pi Camera Module v2
- Slamtec RPLIDAR A1M8
- USB power that can comfortably supply the Pi 5 and peripherals

See [docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md) for wiring, permissions, and first-run checks.

## Recommended Next Steps

1. Commit this baseline so the clean simulator, replay, dashboard, tests, and docs are preserved as v0.
2. Boot the Raspberry Pi, enable SSH, and connect from your laptop. Start with [docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md#first-boot-and-ssh).
3. Bring up the hardware one piece at a time: simulated mapper, camera check, then live RPLIDAR.
4. Record one real room dataset:

```bash
python -m lidar_room_mapper record --source rplidar --port /dev/ttyUSB0 --output captures/first_room.jsonl --limit 200
```

5. Use that replay file for a reliable interview demo, then add map export as the next engineering milestone.

## Project Layout

```text
src/lidar_room_mapper/
  cli.py                 Command-line entry points
  models.py              Shared dataclasses
  mapping/occupancy.py   Occupancy-grid mapper
  sensors/               Sim, replay, RPLIDAR, and camera adapters
  dashboard/             Browser dashboard and HTTP server
tests/                   Protocol, replay, and mapping tests
docs/                    Architecture and interview guide
deploy/                  systemd unit for Pi deployment
```

## Sources

The implementation follows the current Raspberry Pi Picamera2 documentation and Slamtec RPLIDAR A-series SDK/protocol references. Links are collected in the hardware setup doc.
