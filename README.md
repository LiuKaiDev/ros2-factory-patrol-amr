# ROS2 Factory Patrol AMR with Visual Perception, Navigation and Safety Integration

A ROS 2 Jazzy factory patrol AMR that integrates RGB-D perception with the
existing Nav2, AMCL/localization, velocity arbitration, and Safety Gate
infrastructure. It detects objects, projects robust depth through TF2 into the
map frame, manages target lifecycle, starts task-owned visual inspection
missions, and turns person observations into semantic safety events.

The engineering story is the complete robot loop, not a detector demo:

```text
Perception -> Spatial Understanding -> Decision -> Navigation -> Control -> Safety
```

The detector is one replaceable perception backend. Nav2 remains the navigation
stack, `robot_tasks` owns mission execution, and perception never publishes
`/cmd_vel` or `/nav2_cmd_vel`.

Current validation platform: **WSL2, Ubuntu 24.04, ROS 2 Jazzy, and Gazebo
simulation**. No physical-robot or production-factory deployment is claimed.

## Project Overview

The original navigation and control path was preserved:

```text
Mission / Goal -> Nav2 -> cmd_vel mux -> Safety Gate -> Robot
```

The visual upgrade adds spatial and semantic inputs around that path:

```text
RGB-D Camera -> Detector -> Depth + CameraInfo -> 3D Localization -> TF2 (map)
                                                          |
                                                   Target Manager
                                                    /          \
                                             robot_tasks      Safety event
                                                  |               |
                                                Nav2          Safety Gate
                                                  \              /
                                                   Robot motion
```

Implemented capabilities include:

- RGB-D simulation at 640x480 and 15 Hz with ROS image, depth, and CameraInfo topics.
- Replaceable OpenCV-DNN YOLOX-S 2D detection using `vision_msgs` output.
- Median depth projection from a bbox ROI, invalid-depth rejection, and
  observation-time TF2 conversion to `map`.
- Stateful 3D targets with confirmation, loss, processing, cooldown, spatial
  association, and duplicate mission suppression.
- Visual inspection goals planned at a configured standoff and executed through
  the existing Nav2 adapter.
- Person distance/zone decisions integrated into the existing final velocity
  gate without granting perception motion authority.
- Standard diagnostics for camera, detector, depth quality, TF, and pipeline
  health, connected to the existing monitor and fault supervisor.

## Demo / What the Robot Can Do

### Demo 1: RGB-D Detection and 3D Localization

```text
RGB-D -> Detection2D -> robust depth -> optical-frame point
      -> observation-time TF2 -> map-frame target -> RViz marker
```

After model preparation and workspace sourcing:

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  gui:=true use_rviz:=true use_detector:=true geometry_input_mode:=detector
```

Inspect `/perception/detections_2d`, `/perception/objects_3d`,
`/perception/debug_image`, and `/perception/markers`.

### Demo 2: Visual Inspection / Nav2 Approach

```text
CONFIRMED -> INSPECTION_REQUIRED -> robot_tasks -> observation pose
          -> NavigateSequence -> Nav2 -> arrival -> PROCESSED
```

```bash
bash scripts/run_factory_patrol_demo.sh --phase5
```

The planner uses a `1.2 m` target standoff and faces the target rather than
driving to its center. A recorded Phase 5 smoke run returned Nav2 `SUCCEEDED`
with a final robot-target distance of `1.342032 m`; Nav2 goal tolerance
contributed to the difference from the requested standoff. The formal Phase 8
benchmark measured mission success and latency, not physical standoff error.

### Demo 3: Person Safety Integration

```text
Person -> TargetManager -> PerceptionSafetyPolicy
       -> /perception/safety_event -> Safety Gate -> final /cmd_vel
```

```bash
bash scripts/run_factory_patrol_demo.sh --phase6
```

The validation profile exercises `CLEAR`, `SPEED_LIMITED`, distance-based
`STOP`, danger-zone `STOP`, and recovery. **Perception never publishes
`/cmd_vel`; the Safety Gate remains the final velocity authority.**

### Demo 4: Perception Fault Handling

```bash
bash scripts/run_factory_patrol_demo.sh --phase7
```

In another sourced shell:

```bash
bash scripts/check_factory_patrol_perception_diagnostics_runtime.sh
```

The runtime probe injects RGB interruption, depth interruption, invalid depth,
observation-time TF failure, and detector failure, then verifies diagnostic
recovery. Invalid or stale inputs suppress downstream coordinates and mission
triggers instead of fabricating targets or treating unknown perception as a
clear environment.

## System Architecture

### Closed-Loop Pipeline

```mermaid
flowchart TD
  Camera[RGB-D Camera] --> Perception[robot_perception]
  Perception --> Detector[Replaceable Detector]
  Detector --> Geometry[DepthProjector and TF2]
  Geometry --> Targets[TargetManager]
  Targets --> Events[Inspection Event Policy]
  Targets --> SafetyPolicy[Perception Safety Policy]
  Geometry --> Diagnostics[Perception Diagnostics]
  Events --> Tasks[robot_tasks]
  Tasks --> Nav2[Nav2 and AMCL]
  Nav2 --> NavCmd[/nav2_cmd_vel]
  NavCmd --> Mux[cmd_vel mux]
  Mux --> Gate[Safety Gate]
  SafetyPolicy --> SafetyEvent[/perception/safety_event]
  SafetyEvent --> Gate
  Diagnostics --> Monitor[system_monitor and fault_supervisor]
  Monitor --> Gate
  Gate --> Cmd[/cmd_vel]
  Cmd --> Robot[Robot or Gazebo]
  Robot --> Feedback[odom, TF, scan, state]
  Feedback --> Nav2
  Feedback --> Monitor
```

The combined Factory Patrol simulation node implements muxing and safety gating
in one process; the authority boundary remains the same. Other safety inputs
(estop, watchdog, localization, chassis, scan, manual takeover, and legacy
safety state) are resolved with the perception restriction using the most
restrictive state.

Detailed architecture: [docs/architecture.md](docs/architecture.md) and
[docs/safety_state_machine.md](docs/safety_state_machine.md).

## Perception Pipeline

`robot_perception` owns detector adaptation, depth projection, geometry/TF,
target management, inspection event policy, perception safety policy, and
diagnostics. Its detector backend emits standard `Detection2DArray`; depth, TF,
tracking, mission, and safety code do not depend on YOLO-specific output.

### Robust depth projection

The projector samples the central `0.3` portion of the detection bbox, rejects
zero, NaN, Inf, and values outside `0.2-8.0 m`, requires at least five valid
samples, and takes their median. With CameraInfo intrinsics:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

This avoids relying on a single center pixel. Invalid depth or intrinsics
produce no 3D point.

### Target lifecycle

Targets progress through `TENTATIVE`, `CONFIRMED`, `LOST`, and `PROCESSED`.
The manager uses class-aware 3D spatial association, three-frame confirmation,
five-frame loss, EMA (`alpha=0.4`), processed cooldown, and event suppression.
Short validation demonstrated stable IDs and same-ID reacquisition. The longer
Phase 8 run assigned two IDs to one static target after lifecycle transitions,
so long-term identity persistence is not claimed.

## Visual Inspection Workflow

Single-frame detections do not start missions. An allowlisted confirmed target
causes `INSPECTION_REQUIRED`; `robot_tasks` validates it, plans a map-frame
observation pose approximately `1.2 m` from the target, points the robot toward
the target, and uses the existing `/navigate_sequence` adapter and Nav2. Only a
successful navigation result produces `INSPECTION_COMPLETED` and marks the
target `PROCESSED`.

This design avoids target-center collisions and keeps retries, goal lifecycle,
and duplicate mission suppression in the task layer.

## Perception Safety Integration

For currently observed eligible person targets, planar map-frame distance and
configured danger-zone membership produce:

| Condition | Semantic state | Final gate behavior |
| --- | --- | --- |
| Distance `> 3.0 m` | `CLEAR` / normal | No perception restriction |
| Distance `1.5-3.0 m` | `SPEED_LIMITED` | Clamp to configured low speed |
| Distance `< 1.5 m` | `STOP` | Publish zero final velocity |
| Inside danger zone | `STOP` | Publish zero final velocity |

Hysteresis prevents boundary chatter, and three valid clear observations are
required for recovery. A stale perception restriction cannot override another
active safety source. This is supervisory simulation behavior, not a
functional-safety certification.

## Fault Handling / Diagnostics

The standard `/perception/diagnostics` stream contains:

```text
perception/camera_rgb
perception/camera_depth
perception/camera_info
perception/detector
perception/tf
perception/depth_quality
perception/pipeline
```

```text
Perception diagnostics -> system_monitor -> fault_supervisor -> Safety Gate
```

Camera freshness, CameraInfo validity, detector health, observation-time TF,
and depth quality remain distinct so a perception failure is not confused with
a safe empty scene. Fault injection and recovery details are in
[docs/simulation_scenarios.md](docs/simulation_scenarios.md).

## Quantitative Evaluation

### Gazebo / WSL Simulation Benchmark

Source of truth: the committed [JSON](src/robot_experiments/results/factory_patrol_phase8_20260815_011022.json)
and [CSV](src/robot_experiments/results/factory_patrol_phase8_20260815_011022.csv).
The run used headless `factory_patrol.sdf`, CPU OpenCV-DNN YOLOX-S, 640x640
detector input, 640x480 RGB-D at 15 Hz, and confidence threshold `0.45`.

| Metric | Samples | Mean | P50 | P95 | Max |
| --- | ---: | ---: | ---: | ---: | ---: |
| Detector inference | 30 | 526.189 ms | 519.714 ms | 568.830 ms | 656.778 ms |
| 3D localization error | 30 | 0.02045 m | 0.01645 m | 0.05239 m | 0.07075 m |
| Detection to confirmation | 5 | 1.970 s | 2.023 s | 2.405 s | 2.405 s |
| Confirmation to inspection event | 5 | 0.000 s | 0.000 s | 0.000 s | 0.000 s |
| Inspection event to Nav2 goal | 5 | 0.080 s | 0.002 s | 0.385 s | 0.385 s |
| Detection to Nav2 goal | 5 | 2.050 s | 2.034 s | 2.790 s | 2.790 s |
| Safety STOP response | 10 | 0.1806 s | 0.173 s | 0.214 s | 0.214 s |

The 3D localization RMSE was `0.02351 m` across the tested `1.7-3.7 m` range.
All `5/5` visual inspection trials succeeded with zero failures and zero false
mission starts. Invalid depth was correctly rejected in `20/20` cases with zero
false-valid outputs. SPEED_LIMITED clamped a real upstream `0.35 m/s` Nav2
request to `0.15 m/s` in `0.226 s`.

Three additional `INSPECTION_REQUIRED` events from newly assigned IDs arrived
while a task was already active. The task node ignored them, so five intended
trials produced exactly five mission starts.

For 32 paired stationary-target observations, raw x/y/z standard deviations
were `0.00161 / 0.00167 / 0.00540 m`; EMA values were
`0.00181 / 0.00197 / 0.00607 m`. **This benchmark did not demonstrate an EMA
stability improvement.** Filtering remains available, but the deterministic
static simulation sample was slightly worse after EMA.

Thirteen transient non-OK `perception/tf` samples occurred across three mission
trials. Diagnostics detected them, all recovered, and all missions completed;
no coordinate is emitted for a failed lookup.

See [docs/experiment_report.md](docs/experiment_report.md) for method,
exclusions, and interpretation.

## Package Architecture

| Package | Responsibility |
| --- | --- |
| `robot_bringup` | Composes Factory Patrol simulation, Nav2, tasks, perception, and monitoring profiles. |
| `robot_description` | Xacro/URDF model, camera extrinsic, optical frame, and robot assets. |
| `robot_simulation` | Gazebo worlds, ROS-Gazebo bridges, fixtures, configs, and RViz views. |
| `robot_navigation` | Nav2, AMCL, maps, costmaps, localization health, and navigation parameters. |
| `robot_teleop` | Velocity-source arbitration, manual input, Safety Gate, watchdog, estop, and limits. |
| `robot_tasks` | Mission lifecycle, observation-pose planning, visual inspection, and Nav2 action ownership. |
| `robot_perception` | Detection, depth/TF geometry, target management, semantic policies, and diagnostics. |
| `robot_utils` | System health aggregation and fault supervision. |
| `robot_experiments` | Repeatable benchmark probes, statistics, and JSON/CSV output. |
| `robot_interfaces_perception` | 3D target, mission event, and semantic safety message definitions. |
| `robot_interfaces*` | Core, navigation, mission, facility, fleet, business, and site interfaces. |

The workspace currently contains 21 ROS 2 packages.

## ROS Topics and Interfaces

| Area | Topic | Type |
| --- | --- | --- |
| Camera | `/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| Camera | `/camera/depth/image_raw` | `sensor_msgs/msg/Image` (`32FC1`) |
| Camera | `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| Perception | `/perception/detections_2d` | `vision_msgs/msg/Detection2DArray` |
| Perception | `/perception/objects_3d` | `robot_interfaces_perception/msg/DetectedObject3D` |
| Perception | `/perception/events` | `robot_interfaces_perception/msg/PerceptionEvent` |
| Perception safety | `/perception/safety_event` | `robot_interfaces_perception/msg/PerceptionSafetyEvent` |
| Debug | `/perception/debug_image` | `sensor_msgs/msg/Image` |
| Debug | `/perception/markers` | `visualization_msgs/msg/Marker` |
| Diagnostics | `/perception/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` |
| Inspection | `/inspection/observation_pose` | `geometry_msgs/msg/PoseStamped` |
| Inspection | `/inspection/observation_marker` | `visualization_msgs/msg/Marker` |
| Inspection | `/inspection/status` | `std_msgs/msg/String` |
| Safety/control | `/safety/state`, `/safety/reason` | `std_msgs/msg/String` |
| Control | `/nav2_cmd_vel`, `/cmd_vel` | `geometry_msgs/msg/Twist` |

Custom perception interfaces:

- [DetectedObject3D.msg](src/robot_interfaces_perception/msg/DetectedObject3D.msg)
  carries a map-frame target ID, class, confidence, 3D position, depth validity,
  and lifecycle state.
- [PerceptionEvent.msg](src/robot_interfaces_perception/msg/PerceptionEvent.msg)
  carries target confirmation, inspection request, and inspection completion
  events with a map-frame pose.
- [PerceptionSafetyEvent.msg](src/robot_interfaces_perception/msg/PerceptionSafetyEvent.msg)
  carries clear/near/too-close/zone semantics, requested safety state, distance,
  source, and reason. It does not carry velocity commands.

## TF / Coordinate Frames

```text
map -> odom -> base_footprint -> base_link -> camera_link
                                             -> camera_color_optical_frame
```

Projection starts in `camera_color_optical_frame`. The geometry node looks up
TF at the image observation timestamp and transforms the point to `map`, which
is the common frame for association, inspection goals, robot-person distance,
and safety zones. The validated path does not fall back to latest TF to hide
synchronization errors. Without Nav2, the simulation profile publishes an
identity `map -> odom`; with Nav2 enabled, AMCL is authoritative.

## Quick Start

Prerequisites are Ubuntu 24.04 with ROS 2 Jazzy desktop and project dependencies
available through `rosdep`.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
bash scripts/prepare_phase3_detector_model.sh
bash scripts/run_factory_patrol_demo.sh --phase5
```

The model script downloads the official OpenCV Zoo YOLOX-S ONNX file into the
user cache and verifies SHA-256
`c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063`.
Weights are not committed and normal launch never downloads them. The current
backend is CPU OpenCV-DNN; CUDA, TensorRT, and ONNX Runtime are not implemented.

## Demo Commands

```bash
# Core Factory Patrol Gazebo + RViz
bash scripts/run_factory_patrol_demo.sh --launch

# RGB-D detector, 3D targets, task-owned inspection, and Nav2
bash scripts/run_factory_patrol_demo.sh --phase5

# Detector, person policy, Nav2 intent, and final Safety Gate
bash scripts/run_factory_patrol_demo.sh --phase6

# Phase 6 plus perception diagnostics and fault supervision
bash scripts/run_factory_patrol_demo.sh --phase7
```

The earlier Phase 5B multipoint, temporary-obstacle, and localization-recovery
workflows remain documented in [scripts/README.md](scripts/README.md).

## Validation / Tests

### Validation Scripts

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon test
colcon test-result --verbose

bash scripts/check_factory_patrol_assets.sh
bash scripts/check_factory_patrol_runtime_topics.sh
bash scripts/check_factory_patrol_target_manager_runtime.sh
bash scripts/check_factory_patrol_visual_inspection_runtime.sh
bash scripts/check_factory_patrol_perception_safety_runtime.sh
bash scripts/check_factory_patrol_perception_diagnostics_runtime.sh
bash scripts/check_safety_state_machine.sh
bash scripts/check_project_showcase_readiness.sh
```

Runtime scripts require the matching demo profile in another sourced shell.
The latest full WSL2 baseline is **21 packages, 648 tests, 0 errors, 0 failures,
0 skipped**. This is a simulation/software validation result, not hardware
acceptance. The complete script inventory is in
[scripts/README.md](scripts/README.md).

## Benchmark Reproduction

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/run_factory_patrol_benchmarks.sh
```

The runner uses isolated DDS domains and Gazebo partitions for multiple
headless profiles. It writes timestamped JSON and CSV under
`src/robot_experiments/results/`; runtime varies with host load. The reviewed
artifacts cited above are:

- [factory_patrol_phase8_20260815_011022.json](src/robot_experiments/results/factory_patrol_phase8_20260815_011022.json)
- [factory_patrol_phase8_20260815_011022.csv](src/robot_experiments/results/factory_patrol_phase8_20260815_011022.csv)

Phase 9 changes documentation only, so the Phase 8 runtime benchmark is not
rerun or silently altered.

## Design Decisions

1. **RGB-D instead of RGB-only:** metric depth enables explicit 3D coordinates,
   standoff goals, and distance-based policies without monocular scale guesses.
2. **Replaceable detector:** `Detection2DArray` separates backend inference from
   depth, TF, target, mission, and safety logic.
3. **Geometry before detector integration:** deterministic synthetic bboxes made
   camera intrinsics, optical conventions, depth filtering, and TF independently testable.
4. **No velocity from perception:** semantic policy is separated from the final
   Safety Gate, preserving all existing safety sources and control ownership.
5. **Observation pose instead of target center:** the robot stops at a safe,
   view-oriented standoff instead of colliding with the object coordinate.
6. **Confirmed targets only:** multi-frame evidence and task suppression prevent
   single-frame detections and repeated events from starting duplicate missions.
7. **Distinct diagnostics:** failed/stale perception is unknown, not evidence of
   an empty safe environment.
8. **Deferred acceleration and ReID:** TensorRT and advanced identity tracking
   would expand deployment scope and invalidate the measured baseline.

## Known Limitations

- **Simulation only:** quantitative results are from WSL2/Gazebo, not physical hardware.
- **CPU detector:** OpenCV-DNN YOLOX-S inference is CPU-bound (P95 `568.830 ms`).
- **Target identity:** one static target produced two IDs in a longer run;
  task-level suppression prevented duplicate mission starts. Appearance ReID is absent.
- **EMA result:** filtered standard deviation was slightly worse than raw data
  in the 32-pair static sample; no stability improvement is claimed.
- **TF transients:** 13 non-OK TF diagnostic samples occurred across three
  high-load mission trials, recovered, and caused no permanent mission failure.
- **Gazebo pose/odometry alignment:** passive model settling and bridged wheel
  odometry can use different origins; measured geometry error includes this effect.
- **Static inspection target:** the mission plans a fixed observation pose. It
  is not moving-target pursuit or visual servoing.
- **Scope:** no SLAM/VIO upgrade, 3D detector, functional-safety certification,
  hardware deployment, or production object dataset is claimed.

## Roadmap / Optional Future Work

- C++ inference and evaluated ONNX Runtime/TensorRT backends.
- Physical-robot calibration, deployment, and real-factory validation.
- Appearance-aware target re-identification and more robust lifecycle persistence.
- Adaptive filtering evaluated against rosbag replay and dynamic targets.
- Expanded labeled factory-object datasets and additional benchmark repetitions.
- Reviewed runtime screenshots/video with command, commit, parameters, and logs.

## Documentation

- [Engineering project summary](docs/project_summary.md)
- [Detailed architecture](docs/architecture.md)
- [Simulation scenarios and runtime evidence](docs/simulation_scenarios.md)
- [Benchmark method and results](docs/experiment_report.md)
- [Safety state machine](docs/safety_state_machine.md)
- [Visual perception upgrade plan](docs/upgrade/visual_perception_upgrade_plan.md)
- [Showcase evidence policy](docs/showcase/README.md)
- [Validation script inventory](scripts/README.md)
