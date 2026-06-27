# Interview Guide

## 30-Second Pitch

I built a Raspberry Pi robotics mapping stack that turns a low-cost RPLIDAR and Pi Camera into a live room-mapping demo. The system has hardware adapters, replayable data capture, a log-odds occupancy grid, and a browser dashboard, so it is both demoable and testable without the robot on hand.

## Technical Highlights

- RPLIDAR protocol parser for the 5-byte standard scan response.
- Replay/simulator modes for deterministic development.
- Occupancy-grid mapping with ray tracing and log-odds confidence updates.
- First-pass correlative scan matching for relative pose estimates.
- Timestamped camera still recording alongside LiDAR replay files.
- Runtime state isolation with a thread-safe snapshot API.
- No frontend build step, which keeps Pi deployment simple.
- Tests for parsing, replay, and map integration.

## Good Architecture Questions

- Why use log odds instead of raw probabilities?
- How would you handle robot motion?
- How would you timestamp and synchronize LiDAR and camera data?
- What changes if the scanner reports bad quality or zero-distance points?
- How would you make this production-ready for field deployment?

## Honest Limitations

- The live dashboard defaults to a stationary robot at the map center.
- Scan matching is a first-pass local alignment method, not a full SLAM backend with loop closure.
- Camera frames are captured as periodic stills rather than a continuous stream.
- Hardware error handling is intentionally simple in the first pass.

## Strong Next Milestones

1. Add wheel odometry or visual odometry as a pose prior.
2. Improve scan matching with multi-resolution search and loop closure.
3. Export ROS-compatible occupancy maps.
4. Add a calibration command for LiDAR-to-camera timing offsets.
5. Containerize the dashboard for repeatable Pi deployment.
