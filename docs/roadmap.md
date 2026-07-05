# Roadmap

This roadmap records the staged engineering cleanup from Phase 0 to Phase 6.
The repository uses `main` as the active development branch.

## Priority Definition

| Priority | Meaning |
| --- | --- |
| P0 | Core project positioning and navigation-control demo credibility |
| P1 | Engineering completeness for chassis, safety, localization, and scenarios |
| P2 | Final presentation, CI, report templates, and showcase readiness |

## Phase Summary

| Phase | Priority | Status | Goal | Key deliverables |
| --- | --- | --- | --- | --- |
| Phase 0 | P0 | complete | Documentation structure and project positioning | README/docs cleanup, current/planned/TBD boundaries |
| Phase 1A | P0 | complete | Nav2 basic costmap and controller hardening | local obstacle layer, `/scan`, footprint, inflation, RPP collision checks |
| Phase 1B | P0 | complete | Nav2 RViz and minimum validation | `nav2_basic_debug.rviz`, Nav2 demo helper, runtime topic check |
| Phase 2A | P0 | complete | Standalone tracking experiment logging | Pure Pursuit / Stanley CSV metrics and analysis scripts |
| Phase 2B | P0 | complete | Tracking comparison workflow | comparison and plotting helpers, report placeholders |
| Phase 3A | P1 | complete | Chassis protocol v2 baseline | heartbeat, fault code, sequence/timestamp, UDP simulator compatibility |
| Phase 3B | P1 | complete | Odometry and calibration readiness | wheel/track parameters, covariance, calibration docs/checks |
| Phase 4A | P1 | complete | Localization health monitoring | `/localization/health`, AMCL/TF checks, runtime topic validation |
| Phase 4B | P1 | complete | Unified safety state integration | `/safety/state`, `/safety/reason`, safety gate monitoring |
| Phase 5A | P1 | complete | Factory patrol simulation assets | factory world, route, stations, zones, launch/helper scripts |
| Phase 5B | P1 | complete | Factory patrol demo workflow entries | multipoint, temporary obstacle, localization recovery helpers |
| Phase 6 | P2 | current | Final docs, CI, experiment report, and showcase readiness | README, CI static checks, showcase placeholders, readiness script |

## Final Project Shape

The project is now organized around a closed-loop AMR navigation story:

```text
goal / route -> localization -> planning -> control -> safety gate
-> chassis adapter -> odom / TF / state feedback -> health and recovery checks
```

The repository contains enough static validation and documentation to explain
the system clearly, while keeping all runtime performance claims tied to future
real ROS2/Gazebo/Nav2 runs.

## Remaining Work

The following items are intentionally left as future work because they require
real runtime execution or hardware access:

- Fill `docs/experiment_report.md` with measured metrics from recorded runs.
- Add reviewed screenshots and videos under `docs/showcase/`.
- Run full ROS2 Jazzy `colcon build --symlink-install` in a supported Ubuntu
  environment and record the result.
- Tune Nav2 and chassis parameters on the actual target robot.
- Promote selected static checks to a richer ROS2 integration CI when the runner
  image and dependency set are stable.
