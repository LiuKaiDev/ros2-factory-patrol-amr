# Showcase

This directory is reserved for future reviewed showcase materials after real
ROS2/Gazebo/RViz validation. Current Phase 6 work only provides placeholders
and documentation.

Do not add screenshots, videos, CSV-derived figures, or demo claims unless they
are tied to a real command, commit, parameter set, map/world, and log or rosbag
path.

## Planned Artifact Index

| Artifact | Directory | Status | Notes |
| --- | --- | --- | --- |
| RViz Nav2 debug screenshot | `screenshots/` | TBD | To be filled after real Nav2 basic validation. |
| Gazebo factory patrol screenshot | `screenshots/` | TBD | To be filled after real factory patrol validation. |
| Tracking comparison figures | `figures/` | TBD | To be filled after real tracking CSV review. |
| Demo video link | external / TBD | TBD | To be filled after real runtime validation. |

## Validation Summaries

These concise summaries record build, test, and static-check evidence. They do
not claim physical robot deployment or real factory operation.

- [WSL2 full validation summary](wsl2_full_validation_summary.md): local WSL2
  Ubuntu 24.04 + ROS2 Jazzy desktop validation with 514 tests, 0 errors,
  0 failures, and 0 skipped after a package-level retry for one transient Nav2
  `/map` timeout.
- [Server Docker validation summary](server_docker_validation_summary.md):
  Alibaba Cloud Linux 3 + `ros:jazzy-ros-base` partial validation with 17
  packages finished and 104 tests passing while excluding `robot_tasks` because
  the low-memory server killed `cc1plus` during compilation.

## Gazebo + RViz Simulation Debugging

Default Factory Patrol simulation launch:

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true
```

Headless launch:

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=false use_rviz:=false
```

Runtime topic check:

```bash
bash scripts/check_factory_patrol_runtime_topics.sh
```

The default RViz config is
`src/robot_simulation/rviz/factory_patrol_showcase.rviz`. It uses `odom` as the
fixed frame and focuses on robot model, laser scan, odometry, odom path, and
`/amr_simulation/markers` so the default non-Nav2 demo is screenshot-friendly.
`src/robot_simulation/rviz/factory_patrol_debug.rviz` remains available when a
more verbose TF-oriented view is useful.

For the showcase layout, the Factory Semantics marker layer is included but
disabled by default to avoid a debug-looking red marker overlay. Enable it from
the Displays panel when semantic zones, reservations, or state markers need to
be inspected.

The Gazebo world is procedural and lightweight: a 16 m x 12 m factory floor,
widened AMR aisles, receiving buffer details, back storage rack rows, packing
workcell props, dock guidance, safety rails, landmark plates, muted station
signs, floor finish seams, scuff marks, and orthogonal inspection-route floor
markings are modeled with SDF primitives rather than downloaded third-party
assets.

The world config keeps the standard Scene Manager, Camera Tracking, and
Interactive view control plugins enabled so Gazebo can render the 3D scene and
provide camera control services. Fold the right-side panels manually before
capturing screenshots when a cleaner frame is needed.

Expected topics include `/clock`, `/tf`, `/joint_states`, `/odom`, `/scan`,
`/cmd_vel`, `/mission_runner/state`, `/safety_state`, `/localization/health`,
`/amr_simulation/markers`, and `/amr_simulation/demo_timeline`.

After Windows-side edits, final visual review should be done in WSL2 Ubuntu
24.04 with ROS2 Jazzy by launching the demo, checking runtime topics, and
capturing screenshots only after the scene is visually reviewed.

## Future Artifact Checklist

For every future artifact, record:

- commit hash
- command used to launch the run
- map/world and parameter files
- relevant logs or rosbag path
- whether the run was simulation, mock backend, or real hardware

Showcase screenshots and videos are not yet validated by the presence of this
placeholder directory.
