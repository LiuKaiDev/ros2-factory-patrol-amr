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

## Future Artifact Checklist

For every future artifact, record:

- commit hash
- command used to launch the run
- map/world and parameter files
- relevant logs or rosbag path
- whether the run was simulation, mock backend, or real hardware

Showcase screenshots and videos are not yet validated by the presence of this
placeholder directory.
