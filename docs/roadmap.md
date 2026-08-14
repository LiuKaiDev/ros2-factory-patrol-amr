# Roadmap

This file separates completed repository history from optional future work. The
active visual-perception work is developed on
`feature/visual-perception-upgrade`; this document does not imply a merge to
`main`.

## Navigation and Control Baseline

The original Phase 0-6 roadmap is complete. It established the Nav2/AMCL loop,
standalone tracking experiments, chassis protocol and odometry readiness,
localization health, unified safety state, Factory Patrol simulation assets,
demo workflows, CI/static checks, and evidence templates.

| Baseline phase | Status | Result |
| --- | --- | --- |
| Phase 0 | complete | Project structure and evidence boundaries |
| Phase 1A/1B | complete | Nav2 costmap/controller and RViz/runtime checks |
| Phase 2A/2B | complete | Pure Pursuit/Stanley logging and comparison workflow |
| Phase 3A/3B | complete | Chassis protocol v2, odometry, and calibration readiness |
| Phase 4A/4B | complete | Localization health and unified Safety Gate state |
| Phase 5A/5B | complete | Factory world/assets and reproducible demo workflows |
| Phase 6 | complete | Documentation, CI, report, and showcase readiness baseline |

## Visual Perception Upgrade

| Phase | Status | Result |
| --- | --- | --- |
| Phase 0 | complete | Scope and architecture audit |
| Phase 1 | complete | RGB-D sensor, optical TF, topics, bridge, RViz, validation |
| Phase 2 | complete | Robust depth projection and observation-time TF geometry |
| Phase 3 | complete | Replaceable OpenCV-DNN YOLOX-S detector integration |
| Phase 4 | complete | Map-frame TargetManager and lifecycle/event policy |
| Phase 5 | complete | Task-owned visual inspection through existing Nav2 |
| Phase 6 | complete | Person safety events integrated into existing Safety Gate |
| Phase 7 | complete | Perception diagnostics, monitoring, fault injection/recovery |
| Phase 8 | complete | Repeatable Gazebo/WSL benchmark and committed JSON/CSV |
| Phase 9 | complete | Final README, documentation, evidence links, and portfolio summary |

## Current Project Shape

```text
RGB-D -> 2D detection -> depth/TF -> managed map target
                                  /                 \
                         robot_tasks             Safety Gate
                              |                       |
                            Nav2 -> cmd_vel mux ------+
                                      |
                                 final /cmd_vel
```

Nav2 remains the navigation stack. Perception has no velocity publisher, and
the Safety Gate remains final authority.

## Optional Future Work

- Evaluate C++ inference, ONNX Runtime, and TensorRT on target hardware.
- Calibrate and validate the complete loop on a physical robot.
- Add appearance-aware target re-identification and dynamic-target evaluation.
- Evaluate filtering choices with rosbag replay and non-static scenes.
- Expand the factory-object dataset and benchmark repetitions.
- Record reviewed screenshots/video with exact command, commit, parameters, and logs.

These items are not implemented capabilities.
