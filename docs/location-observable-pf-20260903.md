# Observable Fence Particle Filter Experiment

## Input

- Run: `location-20260903-065016`
- Frames: 1723 at 1280x720
- Intrinsics: `config/camera_fisheye_1280x720.yaml`
- Extrinsics: height 0.1291 m, pitch 30.0296 deg, roll 0.2071 deg
- Fence height: 0.254 m
- HSV: `(96, 128, 82)` through `(121, 255, 255)`

The replay uses wheel encoder velocity for incremental translation and the
sign-corrected IMU gyro-Z rate for relative yaw. Optical-flow velocity is not
used. Visual fence geometry is the only absolute measurement.

## Extraction

Both boundaries of the blue fence mask are used. The lower edge is projected
onto z=0 and the upper edge onto z=0.254 m. Samples coincident with the top or
bottom image boundary are rejected as clipping. A point's geometry score uses
`1 / (0.20 + range)^2`, making nearby fence evidence much stronger.

## Fusion

A single wall is a partial measurement. Candidate spread determines the weak
position axis: visual correction is disabled along a clearly unobservable
axis while wall-normal distance and yaw can still be corrected. Wheel odometry
continues to carry position along the wall. Corner and multi-wall observations
permit full 2D correction.

Normal visual correction uses gain 0.12 and limits of 0.04 m and 1.5 deg.
Certain correction uses gain 0.50 and limits of 0.15 m and 6 deg. Very-certain
yaw observations within 15 deg reset the integrated heading and provide
low-pass gyro-bias observations over intervals of at least one second.

## Observations

- Frame 0 constrains y near 0.20 m but leaves x near its wheel-derived value.
- Frame 1185 is a full, precise observation and converges to V1 immediately.
- Frame 1405 accepts the long lower edge as a partial observation rather than
  rejecting it for large global innovation.
- Replay end pose: approximately `(0.35, 0.35, -2.70 rad)`.
- Final estimated gyro-Z bias: approximately `0.025 deg/s`.

Generated files are under the ignored run directory:

- `run-log/location-20260903-065016/dual-edge/telemetry.jsonl`
- `run-log/location-20260903-065016/dual-edge/offline-pf.jsonl`
- `run-log/location-20260903-065016/dual-edge/bev-observable-pf.mp4`
