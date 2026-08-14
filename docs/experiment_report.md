# Experiment Report

This file contains two evidence levels: legacy acceptance templates retain
`TBD` where their runs have not been recorded, while Phase 8 contains the formal
reviewed perception/mission benchmark. The committed Phase 8 JSON/CSV pair is
the primary quantitative source. Do not invent metrics, screenshots, or
pass/fail results.

## Environment

| Item | Value |
| --- | --- |
| ROS distro | Jazzy |
| OS | Ubuntu 24.04 / WSL / other: TBD |
| Commit | TBD |
| Build command | `colcon build --symlink-install` |
| Map / world | TBD |
| Robot model | TBD |
| Nav2 parameter file | TBD |
| Date | TBD |
| Operator | TBD |

## 1. Nav2 Navigation Acceptance

Purpose: verify AMCL + Nav2 behavior for patrol goals.

| Run | Map/world | Planner | Controller | success_rate | arrival_time | final_error | stop_count | result_log |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| TBD | TBD | Navfn | RPP | TBD | TBD | TBD | TBD | TBD |
| TBD | TBD | SmacPlanner2D | MPPI | TBD | TBD | TBD | TBD | TBD |

Required evidence:

- Launch command and parameter files.
- RViz/Gazebo screenshot or video path.
- Topic logs or rosbag path.
- Commit hash and map version.

## 2. Standalone Tracking Experiment

Purpose: compare standalone Pure Pursuit and Stanley tracking outputs using
recorded CSV files.

Run helper:

```bash
bash scripts/run_tracking_experiment_demo.sh
```

Analysis commands:

```bash
python3 scripts/analyze_tracking_result.py <csv_file>
python3 scripts/compare_tracking_results.py --format markdown <pure_pursuit.csv> <stanley.csv>
python3 scripts/plot_tracking_result.py <csv_file> --output-dir src/robot_experiments/results/figures
```

| Run | controller | path_name | goal_success | sample_count | rms_lateral_error | max_lateral_error | mean_abs_heading_error | max_abs_heading_error | mean_linear_velocity | max_abs_angular_velocity | final_distance_to_goal | result_csv |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| TBD | pure_pursuit | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| TBD | stanley | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

Generated figures:

| Figure | Status |
| --- | --- |
| trajectory | TBD |
| lateral error | TBD |
| heading error | TBD |
| cmd_vel | TBD |

## 3. Safety Acceptance

Purpose: verify that safety inputs affect final `/cmd_vel` and safety state
topics as expected.

| Case | Trigger | Expected behavior | Observed behavior | result_log | Notes |
| --- | --- | --- | --- | --- | --- |
| Emergency stop | `/enable_emergency_stop` | final `/cmd_vel` becomes zero | TBD | TBD | TBD |
| Cmd watchdog | mux input timeout | final `/cmd_vel` becomes zero | TBD | TBD | TBD |
| Dynamic speed limit | safety speed limit | velocity is clipped | TBD | TBD | TBD |
| Localization lost | high AMCL covariance / TF timeout | safety state changes and command is gated | TBD | TBD | TBD |
| Chassis fault | heartbeat/fault code issue | command is gated or fault is reported | TBD | TBD | TBD |

## 4. Factory Patrol Demo Acceptance

These navigation-only Factory Patrol workflow rows remain acceptance templates.
They are separate from the recorded visual-perception benchmark below and stay
`TBD` until each exact workflow is logged and reviewed.

| demo_name | scenario | launch_or_script | expected_topics | success_criteria | runtime_result | notes |
| --- | --- | --- | --- | --- | --- | --- |
| Factory Patrol Multipoint | start -> station_A -> station_B -> station_C -> dock | `bash scripts/run_factory_patrol_multipoint_demo.sh` | `/navigate_sequence/current_goal`, `/mission_runner/state`, `/cmd_vel`, `/safety/state` | waypoints reached in order without unhandled safety faults | TBD | TBD |
| Temporary Obstacle | obstacle on patrol route | `bash scripts/run_factory_patrol_obstacle_demo.sh` | `/scan`, `/local_costmap/costmap`, `/cmd_vel`, `/safety/state` | obstacle visible and robot response recorded | TBD | TBD |
| Localization Recovery | bad pose followed by recovery pose | `bash scripts/run_factory_patrol_localization_recovery_demo.sh` | `/localization/health`, `/safety/state`, `/amcl_pose`, `/tf` | LOST and RECOVERED transitions observed | TBD | TBD |

## 5. Showcase Evidence

Showcase screenshots, videos, and generated figures should be listed in
[docs/showcase/README.md](showcase/README.md) only after they are produced from
real runs.

| Artifact | Path | Source command | Status |
| --- | --- | --- | --- |
| RViz Nav2 debug screenshot | TBD | TBD | TBD |
| Gazebo factory patrol screenshot | TBD | TBD | TBD |
| Tracking comparison figure | TBD | TBD | TBD |
| Demo video | TBD | TBD | TBD |

## WSL2 Full Validation

This section records real local ROS2 Jazzy validation on WSL2, not physical
robot deployment or real factory operation.

| Item | Result |
| --- | --- |
| Platform | WSL2 |
| OS | Ubuntu 24.04 LTS |
| ROS distribution | Jazzy |
| ROS package set | `ros-jazzy-desktop` with project dependencies installed through `rosdep` |
| Build result | Full workspace build completed locally |
| Latest full test result | 648 tests, 0 errors, 0 failures, 0 skipped |
| robot_navigation retry | 100% tests passed, 0 tests failed out of 6 |
| RViz launch | Passed in WSL2 validation |
| Gazebo Factory Patrol launch | Passed after adding lightweight label mesh assets |

An earlier full test run had one transient `/map` message timeout in
`robot_navigation/test_nav2_mock_runtime.launch.py`. Re-running
`robot_navigation` independently passed, and the final aggregated
`colcon test-result` reported all tests passing. The likely cause was WSL2/Nav2
runtime timing or DDS topic timing, not a syntax-level build issue.

Runtime topics observed during Factory Patrol simulation validation included
`/map`, `/scan`, `/odom`, `/cmd_vel`, `/tf`, `/joint_states`,
`/mission_runner/state`, `/safety_state`, `/localization/health`,
`/global_costmap/costmap`, `/local_costmap/costmap`,
`/amr_simulation/markers`, and `/amr_simulation/demo_timeline`.

## Phase 8: Perception and Mission Evaluation

Phase 8 extends `robot_experiments` with quantitative probes for the existing
perception, inspection, Nav2, and Safety Gate chain. It does not change the
robot behavior or add a perception algorithm. All outputs from this benchmark
must be labeled **Gazebo / WSL simulation results** and are not physical robot
performance guarantees.

Run the standard headless profile after building and sourcing the workspace:

```bash
bash scripts/run_factory_patrol_benchmarks.sh
```

The command validates assets and the detector model, then produces a
timestamped JSON artifact and a compact performance-table CSV under
`src/robot_experiments/results/`. A failed prerequisite or profile stops the
suite; incomplete profiles are not merged into a final result.
The runner defaults to DDS domain base `140` and increments the domain for
every profile launch so stale WSL ROS processes and Fast DDS participants do
not contaminate later trials; set `FACTORY_PATROL_BENCHMARK_DOMAIN_ID` to choose
a different unused range. It also assigns a unique Gazebo Transport partition
per profile; override `FACTORY_PATROL_BENCHMARK_GZ_PARTITION` only when
deliberate cross-process Gazebo communication is required.

### Method

| Metric | Definition | Clock |
| --- | --- | --- |
| Detector processing | OpenCV-DNN `infer()` call duration | monotonic wall duration inside detector |
| Detector message age | source image stamp to detection callback receipt | ROS simulation clock |
| 3D localization | Euclidean `P_est - P_gt` at near/medium/far center-ray fixture positions | observation data |
| Position stability | population x/y/z standard deviation for raw and TargetManager EMA points sharing an observation stamp | observation data |
| Detection to action | benchmark receipt at first person detection, confirmation, inspection event, and navigation goal | ROS simulation clock |
| STOP response | perception safety condition header stamp to the first gated zero `/cmd_vel` with active upstream Nav2 intent | ROS simulation clock |
| Mission success | detected, confirmed, event, goal, Nav2 success, completion event, and `PROCESSED` | per clean launch trial |
| Invalid depth | controlled invalid depth input with no same-stamp valid map projection | ROS simulation clock |

Percentiles use nearest rank: sort the samples and select
`ceil(percentile / 100 * count)`. Standard deviation is population standard
deviation. Outliers are retained. The first five detector diagnostic samples
are explicitly recorded as warmup exclusions; diagnostic sampling gaps and
fixture-settle exclusions are counted in JSON rather than silently dropped.

The standard WSL CPU profile collects 30 detector samples after warmup, 10
geometry samples at each of three ranges, five independently reset visual
inspection missions, 10 STOP transitions, and 20 invalid-depth cases. Each
mission restarts the launch so TargetManager, cooldown, and task state begin
cleanly. Safety trials alternate the existing Gazebo fixtures while keeping a
Nav2 goal active. A 20-second post-completion observation window counts late
duplicate task triggers instead of stopping as soon as the first mission wins.
An inspection mission start is the task node's `REQUESTED: observation pose
planned` status. Extra perception events that the active task ignores and Nav2
retries are retained as separate raw counts, but are not mislabeled as new
mission starts. GUI and RViz remain disabled to reduce, but not conceal,
resource contention.

### Result Policy

The JSON records the git revision and dirty-state fingerprint, branch, ROS
distro, world, camera, detector backend/model/device, tracking and safety
parameters, sample counts, exclusions, failure categories, and observed
CameraInfo/TF diagnostic faults. The CSV is derived from that JSON. Only a
small reviewed JSON/CSV pair is committed; temporary launch logs remain under
`/tmp` only when `KEEP_FACTORY_PATROL_BENCHMARK_LOGS=true` is set.

### WSL Simulation Result (2026-08-15)

Curated artifacts:

- `src/robot_experiments/results/factory_patrol_phase8_20260815_011022.json`
- `src/robot_experiments/results/factory_patrol_phase8_20260815_011022.csv`

Environment: ROS 2 Jazzy, `factory_patrol.sdf`, headless Gazebo, no RViz,
simulation time, OpenCV YOLOX on the actual CPU device, 640x640 detector input,
640x480 RGB-D at 15 Hz, confidence threshold 0.45. The JSON records commit
`bfb59f9` plus the dirty-tree fingerprint because the result is committed with
the Phase 8 implementation.

| Metric | Count | Mean | P50 | P95 | Max | Unit |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Detector processing | 30 | 526.189 | 519.714 | 568.830 | 656.778 | ms |
| 3D localization error | 30 | 0.02045 | 0.01645 | 0.05239 | 0.07075 | m |
| Detection to confirmation | 5 | 1.970 | 2.023 | 2.405 | 2.405 | s |
| Confirmation to inspection event | 5 | 0.000 | 0.000 | 0.000 | 0.000 | s |
| Inspection event to Nav2 goal | 5 | 0.080 | 0.002 | 0.385 | 0.385 | s |
| Detection to Nav2 goal | 5 | 2.050 | 2.034 | 2.790 | 2.790 | s |
| Safety STOP response | 10 | 0.1806 | 0.173 | 0.214 | 0.214 | s |

Localization RMSE was `0.02351 m`. The medium and far fixture mean errors were
`0.01645 m` and `0.01612 m`; the near fixture retained RGB-D render-transition
samples after the predefined three-sample settle exclusion and had `0.02879 m`
mean and `0.07075 m` max error. Nine settle samples were excluded by policy;
no measured sample was removed after inspection.

For the primary stationary target (`32` paired observations), raw x/y/z
standard deviations were `0.00161 / 0.00167 / 0.00540 m`; EMA values were
`0.00181 / 0.00197 / 0.00607 m`. EMA was slightly worse on all three axes in
this run, so no stability improvement is claimed. Two target IDs were observed
during the continuous detector profile, so ID stability was not perfect.

All five end-to-end inspection trials succeeded (`5/5`, success rate `1.0`),
with zero categorized failures and zero false mission starts. Exactly five
missions started for five intended trials. Three additional inspection-required
events were emitted for newly assigned target IDs while a task was active and
were ignored by the task node; they did not start missions. Inspection event to
completion averaged `6.339 s` (P50 `6.364 s`, P95/max `6.682 s`). The benchmark
did not collect final physical standoff error, so no value is claimed for it.

The repeated STOP test kept a nonzero Nav2 request and required final
`/cmd_vel` to be zero. The separate speed-limit sample clamped an upstream
`0.35 m/s` request to `0.15 m/s` in `0.226 s`. Invalid-depth rejection was
`20/20` with zero false-valid projections.

Thirteen non-OK diagnostic samples reported `perception/tf: observation-time
TF intermittently unavailable` across three mission trials. All recovered and
succeeded; no CameraInfo fault sample was observed. Detector warmup excluded
five recorded samples, six detector diagnostic sampling gaps were counted, and
one Nav2-inactive setup goal was excluded before the Safety trial began. The
runner isolated every profile by DDS domain and Gazebo partition and left no
Phase 8 partition process after the final run.

Interpretation: detector time is CPU-bound in this WSL/OpenCV-DNN setup;
geometry includes RGB-D rendering and simulated odom/map alignment; action
latency includes detector frame cadence and ROS event propagation; safety time
includes perception propagation and the existing final Safety Gate. Historical
Phase 2/3/6 smoke observations remain context, not aggregate samples.

## Result Policy

- Keep `TBD` until the run is real and repeatable.
- Record the exact commit, command, map, params, and logs for every table row.
- Failed runs are valid data if the trigger, observed behavior, and next action
  are documented.
- Generated CSV, PNG, SVG, PDF, and video artifacts are not committed by default.
