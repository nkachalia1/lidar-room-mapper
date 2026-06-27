# LiDAR-Camera Calibration

The dashboard can project the RPLIDAR scan paired with each Pi Camera frame.
This is the geometry foundation for later RGB and sparse-depth fusion.

## Current Calibration Inputs

Camera intrinsics are stored in
`config/camera_intrinsics_pi_camera_v2_1920x1080.json`. They were estimated
from 16 accepted checkerboard views at 1920x1080 with an RMS reprojection error
of 0.373 pixels.

Live capture explicitly selects the Pi Camera v2 `1920x1080`, 10-bit sensor
mode. The dashboard image may be scaled to 1280x720, but the sensor crop remains
the same as the checkerboard calibration. Do not remove the explicit sensor
mode and assume that equal 16:9 aspect ratios imply equal camera intrinsics.

The measured rig geometry is stored in `config/rig_geometry.json`:

- camera is 3.0 inches behind the LiDAR center: `forward=-0.0762 m`;
- camera is provisionally 0.2 inches right: `left=-0.00508 m`;
- camera was measured 4.75 inches above the LiDAR housing center;
- the two-distance image fit gives an effective scan-plane-to-camera vertical
  translation of 5.32 inches: `up=0.13525 m`;
- LiDAR scan plane is 2.25 inches above the floor: `0.05715 m`;
- camera optical center is therefore about 7.0 inches above the floor.

Forward and lateral translation are measured. A two-distance planar-target fit
currently uses `angle_offset=-100.5 deg`, `yaw=0 deg`, `pitch=8.045 deg`, and
`roll=1.05 deg`. Four complete scans were fitted at both 24 and 48 inches. The
target ranges were `0.604-0.630 m` and `1.227-1.237 m`, and the fitted points
crossed manually marked stripe centerlines with RMS errors of about 1.05 and
1.98 pixels. The effective vertical translation differs from the housing-center
measurement because the modeled LiDAR origin must represent the laser scan
plane.

The frozen calibration then passed a held-out 36-inch target without retuning.
Across 56 target points from four complete scans, the observed range was
`0.903-0.919 m` for the `0.9144 m` placement, projection RMS was 0.97 pixels,
and mean vertical residual was 0.26 pixels. The rig is therefore marked
validated for this fixed sensor mount.

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
  --lidar-angle-offset-deg -100.5 \
  --camera-yaw-deg 0 \
  --camera-up-m 0.13525 \
  --camera-pitch-deg 8.045 \
  --camera-roll-deg 1.05
```

Stop with `Ctrl+C`, adjust one value, and restart. Candidate values may be kept
in `config/rig_geometry.json` when the status and provenance clearly identify
their validation limits. Mark the calibration validated only after it aligns
on a held-out target position without retuning.

Use complete rotations when identifying the target. A partial startup scan can
omit the target bearing and make an unrelated room surface look like the target
cluster. Confirm candidate returns by moving the target and checking that their
measured range changes by the same amount.

## Interpretation

- A whole row too high or low indicates camera height or pitch error.
- Alignment near the center but drift at image edges indicates rotation or
  lens-model error.
- Left-right mirroring indicates the LiDAR angle convention is wrong.
- A curved row on a truly flat board suggests the selected returns include
  other surfaces or the target is not intersecting the scan plane cleanly.
- Large timestamp differences while moving cause spatial misalignment even
  when the geometric calibration is correct.
