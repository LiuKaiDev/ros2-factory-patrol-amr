# Showcase

This directory is reserved for reviewed showcase materials from real runs.
Current Phase 6 work creates placeholders and documentation only.

Do not add generated screenshots, videos, CSV-derived figures, or demo claims
unless they are tied to a real command, commit, parameter set, map/world, and
log or rosbag path.

## Planned Artifact Index

| Artifact | Directory | Status | Notes |
| --- | --- | --- | --- |
| RViz Nav2 debug screenshot | `screenshots/` | TBD | Capture after Nav2 basic demo is running. |
| Gazebo factory patrol screenshot | `screenshots/` | TBD | Capture after factory patrol demo is running. |
| Tracking comparison figures | `figures/` | TBD | Generate from real tracking CSV files. |
| Demo video link | external / TBD | TBD | Store link and run metadata here, not a fake local result. |

## Capture Checklist

For every future artifact, record:

- commit hash
- command used to launch the run
- map/world and parameter files
- relevant logs or rosbag path
- whether the run was simulation, mock backend, or real hardware

Runtime success is not claimed by the presence of this directory.
