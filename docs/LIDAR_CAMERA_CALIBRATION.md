# LiDAR-Camera Calibration

The dashboard can project the RPLIDAR scan paired with each Pi Camera frame.
This is the geometry foundation for later RGB and sparse-depth fusion.

## Current Calibration Inputs

Camera intrinsics are stored in
`config/camera_intrinsics_pi_camera_v2_1920x1080.json`. They were estimated
from 16 accepted checkerboard views at 1920x1080 with an RMS reprojection error
of 0.373 pixels.

The measured rig geometry is stored in `config/rig_geometry.json`:

- camera is 3.0 inches behind the LiDAR center: `forward=-0.0762 m`;
- camera is provisionally 0.2 inches right: `left=-0.00508 m`;
- camera is 4.75 inches above the LiDAR scan plane: `up=0.12065 m`;
- LiDAR scan plane is 2.25 inches above the floor: `0.05715 m`;
- camera optical center is therefore about 7.0 inches above the floor.

The translation is measured. The LiDAR angle offset and camera yaw, pitch, and
roll remain provisional until the single-plane target test passes.

## Run the Live Overlay

From the Pi repository and activated virtual environment:

```bash
python -m lidar_room_mapper serve \
  --source rplidar \
  --port /dev/ttyUSB0 \
  --camera \
  --overlay \
  --host 0.0.0.0
```

Open `http://10.0.0.199:8000`. The camera panel reports the number of projected
points and the scan-to-frame timestamp difference.

Replay mode uses the same calibration:

```bash
python -m lidar_room_mapper serve \
  --source replay \
  --input captures/first_room_camera.jsonl \
  --frames captures/first_room_camera_frames.jsonl \
  --overlay \
  --host 0.0.0.0
```

## Refine the Rotation

Use one large, flat, opaque board approximately 0.5 to 1.0 meter in front of
the rig. Put a horizontal stripe on the board at the LiDAR scan-plane height.
The rig and target must rest on the same level surface.

Keep the measured translation fixed. Tune in this order:

1. LiDAR angle offset until returns from the board occupy the board horizontally.
2. Camera yaw until the board returns are centered left-to-right.
3. Camera pitch until the returns cross the horizontal stripe.
4. Camera roll until the projected row has the same slope as the stripe.

Command-line overrides avoid editing the tracked calibration file during tests:

```bash
python -m lidar_room_mapper serve \
  --source rplidar --port /dev/ttyUSB0 --camera --overlay --host 0.0.0.0 \
  --lidar-angle-offset-deg 143 \
  --camera-yaw-deg 0 \
  --camera-pitch-deg 10 \
  --camera-roll-deg 6
```

Stop with `Ctrl+C`, adjust one value, and restart. Promote values into
`config/rig_geometry.json` only after they align on a new capture and a held-out
target position.

## Interpretation

- A whole row too high or low indicates camera height or pitch error.
- Alignment near the center but drift at image edges indicates rotation or
  lens-model error.
- Left-right mirroring indicates the LiDAR angle convention is wrong.
- A curved row on a truly flat board suggests the selected returns include
  other surfaces or the target is not intersecting the scan plane cleanly.
- Large timestamp differences while moving cause spatial misalignment even
  when the geometric calibration is correct.
