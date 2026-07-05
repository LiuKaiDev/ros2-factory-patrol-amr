# Low-Speed Patrol AMR Navigation Control System

This repository is a ROS2 Jazzy + C++17 engineering showcase for a low-speed
patrol AMR in semi-closed environments such as labs, corridors, parks, and
factory aisles. The project focuses on the navigation-control loop rather than
claiming production deployment.

The current codebase includes Nav2, AMCL, Gazebo/RViz simulation, task and
station orchestration, mock/serial/UDP chassis adapters, velocity muxing, safety
gating, localization health monitoring, standalone tracking experiments, and
Factory Patrol demo assets. Factory Patrol scenarios are provided as simulation
assets and demo workflows for low-speed AMR patrol validation. Real hardware
tuning, long-duration field results,
and production certification remain out of scope until they are measured and
documented with real logs.

## Current Scope

| Area | Status | Main assets |
| --- | --- | --- |
| Navigation | current | `src/robot_navigation`, Nav2 basic/advanced params, RViz debug config |
| Localization | current | AMCL plus `/localization/health` monitoring |
| Control | current | Nav2 RPP/MPPI configs and standalone Pure Pursuit / Stanley tracking |
| Chassis adapter | current | mock, serial, UDP, text v1/v2 protocol helpers |
| Safety | current | final `/cmd_vel` safety gate, fault supervision, safety state topics |
| Simulation | current | indoor room and factory patrol Gazebo/RViz assets |
| Experiments | template/current tooling | scripts and report templates; metrics are `TBD` until real runs |
| CI | static current | repository readiness, shell checks, and Python syntax checks |

## Closed-Loop Pipeline

```text
Patrol goal / waypoint
  -> map_server and AMCL
  -> Nav2 global planner
  -> Nav2 local controller
  -> cmd_vel mux
  -> cmd_vel safety gate
  -> chassis driver
  -> mock / serial / UDP backend
  -> odom, TF, chassis state, health feedback
```

Detailed architecture: [docs/architecture.md](docs/architecture.md).

## Quick Start

The intended runtime environment is Ubuntu 24.04 with ROS2 Jazzy.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Common launch entry points:

```bash
# Gazebo + RViz AMR demo
ros2 launch robot_bringup amr_demo.launch.py gui:=true use_rviz:=true

# Mock chassis, sensors, and safety chain
ros2 launch robot_bringup bringup.launch.py mode:=hardware backend:=mock

# Nav2 basic navigation
ros2 launch robot_navigation nav.launch.py navigation_start_delay:=5.0

# Factory patrol demo helper
bash scripts/run_factory_patrol_demo.sh
```

Factory Patrol Demo includes Phase 5B demo workflows for multipoint patrol,
temporary obstacle validation entries, and localization recovery entries. These
workflows are demo and validation entry points, not measured runtime results.

Nav2 basic debug RViz config:

```text
src/robot_simulation/rviz/nav2_basic_debug.rviz
```

Navigation details: [docs/navigation.md](docs/navigation.md).

## Validation Scripts

Static checks do not require a live ROS2 graph unless explicitly stated.

```bash
bash scripts/check_nav2_costmap_obstacle_layer.sh
bash scripts/check_chassis_protocol_v2.sh
bash scripts/check_chassis_odom_calibration.sh
bash scripts/check_localization_health.sh
bash scripts/check_safety_state_machine.sh
bash scripts/check_factory_patrol_assets.sh
bash scripts/check_factory_patrol_demo_workflows.sh
bash scripts/check_project_showcase_readiness.sh
```

Runtime topic checks require a running ROS2/Gazebo/Nav2 environment:

```bash
bash scripts/check_nav2_runtime_topics.sh
bash scripts/check_localization_runtime_topics.sh
bash scripts/check_safety_runtime_topics.sh
bash scripts/check_factory_patrol_demo_runtime.sh
```

Script index: [scripts/README.md](scripts/README.md).

## Experiments And Reports

Experiment tables are intentionally templates until real runs are executed and
reviewed. Do not fill `TBD` fields with generated-looking numbers.

- Report template: [docs/experiment_report.md](docs/experiment_report.md)
- Showcase placeholder index: [docs/showcase/README.md](docs/showcase/README.md)
- Generated CSV files and figures should stay out of git unless a later phase
  explicitly chooses a reviewed artifact.

## Documentation Index

- [Architecture](docs/architecture.md)
- [Navigation](docs/navigation.md)
- [Localization](docs/localization.md)
- [Control](docs/control.md)
- [Chassis Protocol](docs/chassis_protocol.md)
- [Calibration](docs/calibration.md)
- [Safety State Machine](docs/safety_state_machine.md)
- [Simulation Scenarios](docs/simulation_scenarios.md)
- [Experiment Report](docs/experiment_report.md)
- [Interview Notes](docs/interview_notes.md)
- [Roadmap](docs/roadmap.md)
- [Showcase](docs/showcase/README.md)

## Roadmap

Phase 0 through Phase 6 now cover documentation cleanup, Nav2 debug readiness,
tracking experiment tooling, chassis protocol and odometry checks, localization
and safety integration, factory patrol demo assets, and final docs/CI/showcase
readiness. See [docs/roadmap.md](docs/roadmap.md) for the detailed status.

## Project Boundaries

- This is an AMR navigation-control engineering showcase, not production vehicle
  firmware or a certified autonomous vehicle stack.
- Mock, simulation, and static checks are clearly separated from real runtime
  validation.
- Real metrics require commit IDs, parameters, maps, commands, logs, and result
  files before they are written into reports.
