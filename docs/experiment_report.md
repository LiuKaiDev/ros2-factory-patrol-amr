# Experiment Report

This file is the project report template. It intentionally contains `TBD`
placeholders until a real ROS2/Gazebo/Nav2 or hardware run has been executed,
logged, and reviewed. Do not invent metrics, screenshots, or pass/fail results.

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

The factory patrol assets provide entry points for demos, but runtime results
remain `TBD` until recorded in a real environment.

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
| Final test result | 514 tests, 0 errors, 0 failures, 0 skipped |
| robot_navigation retry | 100% tests passed, 0 tests failed out of 6 |

An earlier full test run had one transient `/map` message timeout in
`robot_navigation/test_nav2_mock_runtime.launch.py`. Re-running
`robot_navigation` independently passed, and the final aggregated
`colcon test-result` reported all tests passing. The likely cause was WSL2/Nav2
runtime timing or DDS topic timing, not a syntax-level build issue.

## Result Policy

- Keep `TBD` until the run is real and repeatable.
- Record the exact commit, command, map, params, and logs for every table row.
- Failed runs are valid data if the trigger, observed behavior, and next action
  are documented.
- Generated CSV, PNG, SVG, PDF, and video artifacts are not committed by default.
