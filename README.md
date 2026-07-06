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

Factory Patrol default RViz config:

```text
src/robot_simulation/rviz/factory_patrol_showcase.rviz
```

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
bash scripts/check_factory_patrol_runtime_topics.sh
bash scripts/check_factory_patrol_demo_runtime.sh
```

Script index: [scripts/README.md](scripts/README.md).

## Gazebo + RViz Simulation Debugging

Launch the Factory Patrol showcase simulation with Gazebo and RViz:

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true
```

Preview the independent Factory Patrol Scene V2 industrial layout:

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  world_file:=$(ros2 pkg prefix robot_simulation)/share/robot_simulation/worlds/factory_patrol_industrial.sdf \
  gui:=true use_rviz:=true
```

Headless mode:

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=false use_rviz:=false
```

After launch, check the expected runtime graph:

```bash
bash scripts/check_factory_patrol_runtime_topics.sh
```

Expected topics include `/clock`, `/tf`, `/joint_states`, `/odom`, `/scan`,
`/cmd_vel`, `/mission_runner/state`, `/safety_state`, `/localization/health`,
`/amr_simulation/markers`, and `/amr_simulation/demo_timeline`.

The Factory Patrol world uses lightweight procedural SDF primitives for a
16 m x 12 m factory floor, widened AMR aisles, receiving buffers, back storage
rack rows, a right-side packing workcell, dock guidance, safety landmarks,
orthogonal inspection-route floor markings, muted station signs, and subtle
floor finish detail.
`factory_patrol_industrial.sdf` is an optional Scene V2 preview world with a
24 m x 16 m industrial floor, clearer receiving / storage / packing / dock
zones, and a wider closed inspection loop. The original `factory_patrol.sdf`
remains the default stable baseline until the V2 scene is reviewed in WSL2.
The default RViz view is a non-Nav2 showcase layout; Nav2 map and costmap
debugging remains in `nav2_basic_debug.rviz`. The Factory Semantics marker layer
is present in RViz but disabled by default for cleaner screenshots.

The Gazebo GUI world config keeps the standard Scene Manager, Camera Tracking,
and Interactive view control plugins enabled so the 3D view and camera services
are available. For clean screenshots, fold the right-side panels manually; this
does not affect the runtime graph.

After Windows-side edits, final visual acceptance should be performed in WSL2
Ubuntu 24.04 with ROS2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true
bash scripts/check_factory_patrol_runtime_topics.sh
```

This is WSL2 / ROS2 Jazzy simulation validation. It does not claim physical
robot deployment or real factory operation.

AMR motion smoke test with direct velocity command:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Command multiplexer inputs are `/teleop_cmd_vel`, `/nav2_cmd_vel`,
`/tracking_cmd_vel`, and `/virtual_rc/cmd_vel`. Useful observations:
`/cmd_vel_mux/active_source`, `/safety_state`, `/cmd_vel`, `/odom`, and
`/mission_runner/state`.

Robot-not-moving checks:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic echo /safety_state
ros2 topic echo /mission_runner/state
ros2 topic info -v /cmd_vel
```

## Validation Status

- Static checks pass in the documented validation environment.
- Server Docker validation on Alibaba Cloud Linux 3 with `ros:jazzy-ros-base`
  passed for the main workspace subset excluding `robot_tasks`: 17 packages
  finished, and `colcon test` reported 104 tests, 0 errors, 0 failures, and
  0 skipped. `robot_tasks` was excluded on that low-resource server because
  `mission_runner_node.cpp` triggered a `cc1plus` out-of-memory kill on about
  1.8 GB RAM.
- Local WSL2 Ubuntu 24.04 + ROS2 Jazzy desktop full validation passed after a
  package-level retry for one transient Nav2 runtime timing issue: final
  aggregated `colcon test-result` reported 514 tests, 0 errors, 0 failures, and
  0 skipped.
- RViz and Gazebo Sim launched in WSL2, and the Factory Patrol world loaded
  after replacing temporary label placeholders with lightweight text mesh
  assets.

These validation notes cover build, tests, and static checks. They do not claim
real robot deployment or real factory operation.

## Experiments And Reports

Experiment tables are intentionally templates until real runs are executed and
reviewed. Do not fill `TBD` fields with generated-looking numbers.

- Report template: [docs/experiment_report.md](docs/experiment_report.md)
- Showcase placeholder index: [docs/showcase/README.md](docs/showcase/README.md)
- WSL2 full validation summary:
  [docs/showcase/wsl2_full_validation_summary.md](docs/showcase/wsl2_full_validation_summary.md)
- Server Docker validation summary:
  [docs/showcase/server_docker_validation_summary.md](docs/showcase/server_docker_validation_summary.md)
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
