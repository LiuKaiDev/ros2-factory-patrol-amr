# Engineering Project Summary

## Project

**ROS2 Factory Patrol AMR with Visual Perception, Navigation and Safety
Integration**

This project extends an existing ROS 2 Jazzy patrol robot with a modular RGB-D
perception loop while preserving Nav2, AMCL/localization, velocity arbitration,
and the final Safety Gate. Validation is currently WSL2 Ubuntu 24.04 and Gazebo
simulation; it is not a physical-robot deployment.

## Problem

The original AMR could execute planned patrol/navigation goals, arbitrate
velocity sources, and stop or limit motion through a safety gate. It could not
turn camera observations into stable map-frame targets, task-owned inspection
goals, or semantic person restrictions.

The upgrade needed to add those capabilities without creating a parallel
navigation stack, allowing perception to command motion, or coupling the whole
system to one detector model.

## Implemented Architecture

```text
RGB-D -> Detection2D -> robust depth -> optical-frame 3D
      -> observation-time TF2 -> map-frame TargetManager
                                  /                 \
                     inspection event          safety event
                            |                       |
                       robot_tasks              Safety Gate
                            |                       |
                          Nav2 -> mux --------------+
                                  |
                             final /cmd_vel
```

Key ownership boundaries:

- `robot_perception` produces detections, geometry, managed targets, semantic
  mission/safety events, markers, and diagnostics.
- `robot_tasks` owns observation-pose planning, action lifecycle, and the
  existing `/navigate_sequence` to Nav2 path.
- Nav2 remains responsible for planning and control and publishes
  `/nav2_cmd_vel` into the existing mux.
- The Safety Gate remains the only component that resolves all safety sources
  before final `/cmd_vel`.
- Perception publishes neither `/cmd_vel` nor `/nav2_cmd_vel`.

## Engineering Work to Discuss

### Sensor and geometry integration

- Added one authoritative camera extrinsic in Xacro and matching Gazebo RGB-D
  sensor poses in both Factory Patrol worlds.
- Added conventional `camera_color_optical_frame`, ROS-Gazebo topic bridges,
  and RGB/depth/CameraInfo validation.
- Implemented synchronized 2D/depth/CameraInfo processing, bbox-center ROI
  median depth, invalid depth rejection, pinhole projection, and
  observation-time TF2 conversion into `map`.

### Replaceable detector boundary

- Wrapped CPU OpenCV-DNN YOLOX-S behind `DetectorBackend`.
- Published standard `vision_msgs/msg/Detection2DArray`, leaving geometry,
  tracking, mission, and safety code independent of YOLO-specific details.
- Added a checksum-verified model preparation script; weights are not committed
  or downloaded implicitly at launch.

### Stateful targets and decisions

- Added class-aware spatial association and the target lifecycle `TENTATIVE`,
  `CONFIRMED`, `LOST`, and `PROCESSED`.
- Added multi-frame confirmation, loss/reacquisition behavior, EMA, cooldown,
  and semantic inspection events.
- Kept mission duplicate suppression in `robot_tasks`, including suppression of
  new target-ID events while a mission is active. The long benchmark recorded
  three such extra events and zero false mission starts.

### Navigation integration

- Planned an observation pose at a configured `1.2 m` standoff and oriented the
  robot toward the target rather than navigating to the object center.
- Reused the existing `NavigateSequence` adapter and Nav2; no visual servoing or
  perception velocity publisher was introduced.
- A Phase 5 smoke run returned Nav2 `SUCCEEDED` with a measured `1.342032 m`
  final robot-target distance. Nav2 goal tolerance explains part of the
  difference; this value is not part of the formal Phase 8 standoff metrics.

### Safety and fault handling

- Converted confirmed person observations into semantic CLEAR, SPEED_LIMITED,
  STOP, and danger-zone STOP events.
- Integrated those events into both existing Safety Gate variants using the
  most restrictive active source.
- Added standard diagnostics for camera streams, CameraInfo, detector, TF,
  depth quality, and pipeline health, then connected them to the existing
  system monitor and fault supervisor.
- Validated that interrupted or invalid perception suppresses downstream output
  rather than generating false coordinates or tasks.

### Reproducible evaluation

- Added isolated headless profiles for detector, geometry, mission, safety, and
  invalid-depth measurements.
- Recorded clock policy, warmup/exclusions, source commit/tree state,
  configuration, raw samples, and derived nearest-rank statistics in JSON and
  CSV artifacts.
- Kept early smoke observations separate from the formal benchmark.

## Resume-ready Verified Metrics

All values below are Gazebo/WSL simulation results from the committed Phase 8
artifact, not physical-robot guarantees.

- 3D localization RMSE: `0.02351 m` over 30 samples at `1.7-3.7 m`; P95 error
  `0.05239 m`.
- CPU OpenCV-DNN YOLOX-S inference: 30 samples, mean `526.189 ms`, P95
  `568.830 ms`.
- Detection-to-Nav2-goal: 5 runs, mean `2.050 s`, P95 `2.790 s`.
- Safety STOP response: 10 runs, mean `0.1806 s`, P95 `0.214 s`.
- Visual inspection: `5/5` successful, zero categorized failures, zero false
  mission starts.
- Invalid-depth rejection: `20/20`, zero false-valid 3D outputs.
- Speed limiting: upstream `0.35 m/s` clamped to `0.15 m/s` in `0.226 s`.
- Full ROS 2 baseline: 21 packages, 648 tests, 0 errors, 0 failures, 0 skipped.

Evidence:

- [Phase 8 JSON](../src/robot_experiments/results/factory_patrol_phase8_20260815_011022.json)
- [Phase 8 CSV](../src/robot_experiments/results/factory_patrol_phase8_20260815_011022.csv)
- [Experiment method and interpretation](experiment_report.md)

## Results That Must Not Be Overstated

For 32 paired stationary observations, raw x/y/z standard deviation was
`0.00161 / 0.00167 / 0.00540 m`, while EMA was
`0.00181 / 0.00197 / 0.00607 m`. EMA was slightly worse on all axes in this
sample, so no filtering improvement is claimed.

One static physical target produced two IDs during a longer run. Short tests
demonstrated same-ID reacquisition, and task-level suppression prevented extra
events from starting duplicate missions, but persistent appearance identity is
not implemented.

Thirteen transient non-OK `perception/tf` samples were observed across three
mission trials. They recovered and all missions succeeded; failed TF lookups do
not produce map coordinates.

## Major Design Decisions

| Decision | Reason |
| --- | --- |
| RGB-D rather than RGB-only | Provides metric geometry without monocular scale assumptions. |
| Standard detector output | Makes the inference backend replaceable. |
| Geometry validated before detector | Separates camera/TF correctness from model variability. |
| Confirm before action | Prevents one-frame detections from triggering missions or safety transitions. |
| Observation pose | Preserves standoff and view orientation instead of driving to target center. |
| Semantic safety event | Keeps motion authority in the existing final gate. |
| Distinct health channels | Prevents perception failure from being interpreted as an empty safe scene. |
| Defer TensorRT and ReID | Avoids unsupported deployment claims and keeps the benchmarked scope coherent. |

## Known Limitations

- Metrics are WSL2/Gazebo only; hardware calibration, timing, and safety remain
  unvalidated.
- CPU inference is slow and currently dominates perception latency.
- Tracking is short-horizon spatial association without appearance ReID.
- EMA did not improve stability in the collected static sample.
- Transient TF warnings occurred under mission load and recovered.
- Passive Gazebo settling and integrated wheel odometry can have different
  origins, contributing to geometry error.
- Inspection assumes a static target and fixed observation goal; it is not
  pursuit or visual servoing.

## Implementation Status

Visual perception upgrade Phases 0 through 9 are complete. Phase 9 finalized
documentation, reproducible commands, evidence links, and the portfolio
presentation without changing benchmarked runtime behavior.
