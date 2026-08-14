# System Architecture

The project is a low-speed factory patrol AMR whose navigation, control, safety,
perception, and diagnostic paths have explicit ownership boundaries. The visual
upgrade extends the existing Nav2-based robot; it does not replace navigation or
create a second velocity controller.

## End-to-End Closed Loop

```mermaid
flowchart TD
  Goal[Patrol goal or visual target] --> Tasks[robot_tasks]
  Camera[RGB-D camera] --> Perception[robot_perception]
  Perception --> Target[Map-frame TargetManager]
  Target --> Inspection[Inspection event policy]
  Target --> Person[Person safety policy]
  Inspection --> Tasks
  Tasks --> Nav2[Nav2 planner and controller]
  Localization[AMCL and localization health] --> Nav2
  Nav2 --> NavCmd[/nav2_cmd_vel]
  Teleop[Teleop or virtual RC] --> Mux[cmd_vel mux]
  Tracking[Standalone tracking experiments] --> Mux
  NavCmd --> Mux
  Mux --> Gate[Safety Gate]
  Person --> SafetyEvent[/perception/safety_event]
  SafetyEvent --> Gate
  Diagnostics[System and perception diagnostics] --> Fault[fault_supervisor]
  Fault --> Gate
  Gate --> Cmd[/cmd_vel]
  Cmd --> Backend[Gazebo or chassis adapter]
  Backend --> Feedback[odom, TF, scan, chassis state]
  Feedback --> Localization
  Feedback --> Diagnostics
```

The normal hardware bringup uses `twist_mux` followed by
`cmd_vel_safety_gate_node`. Factory Patrol simulation uses the existing combined
`cmd_vel_mux_node`, which performs source selection and the same final safety
resolution before `/cmd_vel`. Both preserve this authority order:

```text
candidate commands -> velocity arbitration -> Safety Gate -> final /cmd_vel
```

Perception publishes spatial targets, mission events, safety events, markers,
and diagnostics. It has no `/cmd_vel` or `/nav2_cmd_vel` publisher.

## Package Responsibilities

| Package | Current responsibility |
| --- | --- |
| `robot_bringup` | Top-level composition for simulation, navigation, tasks, perception, and monitoring. |
| `robot_description` | Xacro/URDF, physical links, authoritative RGB-D extrinsic, and optical frame. |
| `robot_hardware` | Chassis protocol, mock/serial/UDP backends, driver, kinematics, state, and odometry covariance. |
| `robot_sensors` | Laser/IMU normalization, fake sources, and sensor diagnostics. |
| `robot_navigation` | Nav2, AMCL, maps, costmaps, map management, and localization health. |
| `robot_path_tracking` | Standalone Pure Pursuit and Stanley experiment controllers. |
| `robot_teleop` | Manual input, velocity muxing, watchdog, estop, speed limits, and final Safety Gate. |
| `robot_tasks` | Mission lifecycle, station tasks, observation-pose planning, visual inspection, and Nav2 action ownership. |
| `robot_perception` | Detector adapter, depth/TF geometry, TargetManager, semantic task/safety policies, and diagnostics. |
| `robot_simulation` | Gazebo worlds/bridges, RGB-D sensor, fixtures, semantic config, and RViz views. |
| `robot_utils` | System monitor and fault supervisor. |
| `robot_experiments` | Repeatable benchmark probes, statistics, and result serialization. |
| `robot_interfaces*` | Domain-separated custom interfaces, including perception targets and semantic events. |

## Perception Boundary

```text
/camera/color/image_raw -> replaceable Detector -> Detection2DArray
                                              + synchronized depth/CameraInfo
                                              -> DepthProjector
                                              -> camera optical point
                                              -> observation-time TF2
                                              -> map-frame observation
                                              -> TargetManager
```

`DetectorBackend` isolates the OpenCV-DNN YOLOX-S implementation. Downstream
geometry consumes the standard `vision_msgs/msg/Detection2DArray`, so a future
detector can be substituted without changing projection, target, task, or
safety message contracts.

Depth is a median of valid samples from the central bbox ROI, not a single
pixel. The projection rejects invalid depth and invalid intrinsics. TF lookup
uses the image observation timestamp; latest-TF fallback is intentionally not
used.

The shared coordinate path is:

```text
map -> odom -> base_footprint -> base_link -> camera_link
                                             -> camera_color_optical_frame
```

`map` coordinates are authoritative for target association, observation goals,
robot-person distance, and configured zones.

## Target and Mission Ownership

TargetManager assigns class-aware spatial IDs and manages:

```text
TENTATIVE -> CONFIRMED -> LOST
                    \-> PROCESSED
```

Confirmation prevents a single detector frame from starting work. Lost-frame
handling allows short reacquisition; EMA reduces abrupt position updates when
useful; processed cooldown and task state suppress duplicate actions. This is
short-horizon spatial tracking, not appearance ReID. The longer Phase 8 run
observed two IDs for one static target after lifecycle transitions.

For an eligible confirmed target:

```text
TargetManager
  -> /perception/events (INSPECTION_REQUIRED, map-frame target pose)
  -> robot_tasks visual_inspection_task_node
  -> observation pose planner (configured 1.2 m standoff, face target)
  -> existing /navigate_sequence action
  -> Nav2 NavigateToPose
  -> /nav2_cmd_vel
  -> cmd_vel mux
  -> Safety Gate
  -> /cmd_vel
```

`robot_tasks` owns event validation, observation-pose planning, action lifecycle,
retry policy, and completion/failure decisions. It emits
`INSPECTION_COMPLETED` only after navigation succeeds. Perception consumes that
semantic completion and marks its target `PROCESSED`; task code does not access
perception process memory.

## Safety Ownership

The perception policy evaluates only currently observed eligible person targets
using planar map-frame distance from the observation-time `map -> base_link`
transform:

| Condition | Policy result |
| --- | --- |
| `distance > 3.0 m` | `CLEAR` |
| `1.5 m <= distance <= 3.0 m` | `SPEED_LIMITED` |
| `distance < 1.5 m` | `STOP` |
| Target inside configured danger zone | `STOP` |

The policy uses hysteresis and requires three clear observations for recovery.
It publishes `robot_interfaces_perception/msg/PerceptionSafetyEvent`; it does
not clamp Twist itself. The Safety Gate combines this semantic restriction with
watchdog, estop, scan, localization, chassis, manual takeover, and legacy safety
inputs using the most restrictive active state.

## Diagnostics and Failure Semantics

```text
/perception/diagnostics -> system_monitor -> fault_supervisor -> Safety Gate
```

Independent status names distinguish RGB freshness, depth freshness,
CameraInfo validity, detector health, observation-time TF, depth quality, and
aggregate pipeline health. Missing, invalid, or stale data does not generate a
new point, target, task, or artificial CLEAR event. This is fault-aware
supervision, not a functional-safety certification.

## Simulation and Navigation Authority

Factory Patrol contains both `factory_patrol.sdf` and the independent industrial
preview world. The primary profile bridges RGB-D, scan, IMU, odom, TF, and final
velocity without changing the original robot geometry or navigation stack.

When Nav2 is disabled, perception validation publishes an identity
`map -> odom` transform. When Nav2 is enabled, AMCL owns `map -> odom`. Passive
Gazebo settling can occur before bridged wheel odometry begins, so world pose
and integrated odometry can have different origins; geometry benchmarks retain
that alignment effect rather than masking it with a TF fallback.

## Evidence Boundary

The formal quantitative evidence is the Phase 8 WSL2/Gazebo benchmark documented
in [experiment_report.md](experiment_report.md). Early Phase 2/3/5/6 values in
[simulation_scenarios.md](simulation_scenarios.md) are labeled smoke runs and are
not aggregated into the final benchmark. Physical hardware behavior remains
unvalidated.
