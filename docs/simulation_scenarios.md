# Simulation Scenarios

当前仿真基础已经能够支撑 AMR 演示和部分验收脚本，但面向“厂区 / 园区半封闭低速巡检”的专用场景仍是 planned。

## Current Simulation Basis

| Asset / module | Status | Notes |
| --- | --- | --- |
| `src/robot_simulation/worlds/indoor_room.sdf` | current | 当前 Gazebo 室内场景。 |
| `src/robot_navigation/maps/indoor_room.yaml` | current | 当前静态地图。 |
| `src/robot_simulation/config/amr_sim_stations.yaml` | current | 仿真站点配置。 |
| `src/robot_simulation/config/amr_sim_map_zones.yaml` | current | 仿真 map zones 配置。 |
| `amr_sim_visualizer_node` | current | RViz marker 可视化。 |
| `amr_sim_demo_director_node` | current | 自动演示编排器。 |
| Gazebo entity bridge | current | 将部分业务 / 安全状态映射到 Gazebo 实体。 |

## Planned Factory Patrol Assets

| Asset | Status | Purpose |
| --- | --- | --- |
| `factory_patrol.sdf` | planned | 厂区 / 园区巡检 Gazebo world。 |
| `factory_patrol.yaml` map | planned | 与巡检 world 对齐的静态地图。 |
| patrol stations | planned | 巡检点、充电点、等待点。 |
| patrol zones | planned | 限速区、禁行区、临时障碍区。 |
| scenario launch | planned | 一键运行厂区巡检 demo。 |

## Demo Design

### 1. Multi-point Patrol

Status: partial current / planned factory demo.

目标：

- 从起点出发；
- 依次到达多个巡检点；
- 每个点停留或发布到达状态；
- 完成后返回等待点或充电点。

当前可复用任务 / 站点 / mission runner 能力；最终 factory 场景、巡检点配置和报告待 Phase 5 补齐。

### 2. Temporary Obstacle Avoidance

Status: planned.

目标：

- 在巡检路径上放置临时障碍；
- 验证 local costmap 感知、清障和绕行；
- 记录是否停车、是否重新规划、到达时间和 stop_count。

不能在未运行实验前声称成功率或避障性能。

### 3. Localization Lost and Recovery

Status: current logic / planned factory demo.

目标：

- 模拟 AMCL covariance 超阈值；
- localization health 进入 `LOST`；
- 任务暂停并请求重定位；
- 设置初始位姿后进入 `RECOVERED`；
- 任务恢复。

当前代码和检查脚本已有定位健康 / 重定位入口；最终巡检场景中的可视化演示和实验报告仍待补齐。

## Phase 5A Factory Patrol Assets

Phase 5A adds a factory patrol simulation asset skeleton for a low-speed
semi-closed AMR patrol site. This phase adds files, configuration, and launch
entry points only. It does not claim a full Gazebo/Nav2 mission has been run.

Scanned current structure:

| Area | Existing path |
| --- | --- |
| Gazebo world | `src/robot_simulation/worlds/indoor_room.sdf` |
| Maps | `src/robot_navigation/maps/indoor_room.yaml`, `src/robot_navigation/maps/indoor_room.pgm` |
| RViz | `src/robot_simulation/rviz/amr_sim.rviz`, `src/robot_simulation/rviz/nav2_basic_debug.rviz` |
| Simulation launch | `src/robot_simulation/launch/sim.launch.py` |
| Navigation launch | `src/robot_navigation/launch/nav.launch.py`, `src/robot_navigation/launch/navigation_no_docking.launch.py` |
| Bringup launch | `src/robot_bringup/launch/amr_demo.launch.py`, `src/robot_bringup/launch/bringup.launch.py` |
| Mission launch | `src/robot_tasks/launch/mission_runner.launch.py` |
| Existing sim stations/zones | `src/robot_simulation/config/amr_sim_stations.yaml`, `src/robot_simulation/config/amr_sim_map_zones.yaml` |
| Existing task station config | `src/robot_tasks/config/stations/warehouse_stations.yaml` |

New Phase 5A assets:

| Asset | Path | Status |
| --- | --- | --- |
| Factory world | `src/robot_simulation/worlds/factory_patrol.sdf` | current asset |
| Station seed config | `src/robot_simulation/config/factory_patrol_stations.yaml` | current config asset |
| Zone seed config | `src/robot_simulation/config/factory_patrol_zones.yaml` | current config asset |
| Patrol route config | `src/robot_simulation/config/factory_patrol_route.yaml` | current config asset / planned mission input |
| Map note | `src/robot_navigation/maps/factory_patrol_map_README.md` | current documentation |
| Demo launch | `src/robot_bringup/launch/factory_patrol_demo.launch.py` | current launch skeleton |
| Demo helper | `scripts/run_factory_patrol_demo.sh` | current helper script |
| Static check | `scripts/check_factory_patrol_assets.sh` | current static check |

`factory_patrol.sdf` contains:

- 16 m x 12 m factory floor with widened AMR aisles
- orthogonal dock -> receiving -> storage -> packing -> dock inspection route
- main patrol road
- equipment inspection area
- narrow corridor marker
- turning area
- station markers for `station_A`, `station_B`, `station_C`, and `dock`
- static obstacles using simple boxes/cylinders
- slow-zone visual overlay
- no-go planned visual overlay

Station config:

`src/robot_simulation/config/factory_patrol_stations.yaml` defines `start`,
`station_A`, `station_B`, `station_C`, and `dock` in the `map` frame. Coordinates
are simulation seed poses aligned to the SDF layout, not measured field poses.

Zone config:

`src/robot_simulation/config/factory_patrol_zones.yaml` defines
`factory_slow_corridor`, `factory_no_go_equipment_service_area`, and
`factory_turning_caution_area`. These are Phase 5A configuration assets only and
are not claimed to be wired into Nav2 costmap filters yet.

Route config:

`src/robot_simulation/config/factory_patrol_route.yaml` defines
`factory_patrol_loop`:

```text
start -> station_A -> station_B -> station_C -> dock
```

The current mission runner is not claimed to execute this YAML directly. Later
phases can wire it into mission execution or convert it into the package's
existing mission format.

Map handling:

Phase 5A does not commit a fake occupancy map. See
`src/robot_navigation/maps/factory_patrol_map_README.md` for the planned
SLAM/map_saver flow. Until a real or reviewed placeholder map exists, the
factory demo launch keeps Nav2 disabled by default.

Demo entry:

```bash
bash scripts/run_factory_patrol_demo.sh
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true
```

To experiment with Nav2 after providing an explicit factory map:

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  use_nav2:=true nav2_map:=/absolute/path/to/factory_patrol.yaml
```

RViz views:

The Factory Patrol demo launch uses
`src/robot_simulation/rviz/factory_patrol_showcase.rviz` by default when
`use_rviz:=true`. This is the non-Nav2 showcase view with `odom` fixed frame,
RobotModel, Lidar Scan, Odometry, Odom Path, and Factory Semantics markers.
Factory Semantics is included but disabled by default for cleaner screenshots;
enable it when inspecting runtime marker state.

Use `src/robot_simulation/rviz/factory_patrol_debug.rviz` for a more verbose
factory debug view, or `src/robot_simulation/rviz/nav2_basic_debug.rviz` when
running Nav2 map/costmap debugging separately.

The Gazebo world uses lightweight procedural SDF primitives for receiving,
storage, packing, dock, safety, muted station signage, floor finish seams, scuff
marks, landmark details, a larger factory layout layer, and route markings that
keep the AMR inspection loop visually separated from racks, rails, walls, and
workcell props. These visual assets do not claim runtime mission success by
themselves.

Current / planned boundary:

- Current in Phase 5A: world/config assets, map-generation note, demo launch
  skeleton, run script, static check script, documentation.
- Planned after Phase 5A: dynamic obstacle demo, real/reviewed factory map,
  closed-loop multi-point mission execution, localization-lost recovery demo,
  and any experiment result report.

No Gazebo, RViz, Nav2, or real robot runtime result is claimed by this document.

## Phase 5B Factory Patrol Demo Workflows

Phase 5B turns the Phase 5A factory patrol assets into three demo workflow
entries. These are launch/script/config entry points and acceptance steps, not
claimed runtime results.

### Demo 1: Multipoint Patrol

Goal:

```text
start -> station_A -> station_B -> station_C -> dock
```

Entry points:

```bash
bash scripts/run_factory_patrol_multipoint_demo.sh
python3 scripts/print_factory_patrol_goals.py
```

Config assets:

- `src/robot_simulation/config/factory_patrol_route.yaml`
- `src/robot_simulation/config/factory_patrol_stations.yaml`
- `src/robot_simulation/config/factory_patrol_multipoint_mission.yaml`

Observed topics during a real run:

- `/navigate_sequence/current_goal`
- `/navigate_sequence/current_path`
- `/mission_runner/state`
- `/cmd_vel`
- `/odom`
- `/safety/state`
- `/safety/reason`

Expected behavior to verify in runtime logs: each waypoint is issued in order,
the robot approaches each station, and the mission returns to `dock`. Current
boundary: scripts and mission profile are present; no success rate or travel
time is filled until a real run is captured.

### Demo 2: Temporary Obstacle

Goal: place a simple temporary box obstacle near the `station_A` to `station_B`
segment and observe perception/planning/safety topics.

Entry point:

```bash
bash scripts/run_factory_patrol_obstacle_demo.sh
```

Config/model assets:

- `src/robot_simulation/config/factory_patrol_obstacle_demo.yaml`
- `src/robot_simulation/models/temporary_box_obstacle/model.sdf`
- `src/robot_simulation/models/temporary_box_obstacle/model.config`

Suggested runtime observations:

- `/scan`
- `/local_costmap/costmap`
- `/cmd_vel`
- `/safety/state`
- `/safety/reason`

Expected behavior to verify in runtime logs: the obstacle appears in scan data
and local costmap, and the controller response is visible in `/cmd_vel`. Whether
the robot slows, stops, or replans depends on the actual Nav2 runtime state.
This document does not claim the avoidance behavior has passed.

### Demo 3: Localization Lost And Recovery

Goal: inject a bad `/initialpose`, then inject a recovery `/initialpose`, and
observe localization health and safety-state linkage.

Entry point:

```bash
bash scripts/run_factory_patrol_localization_recovery_demo.sh
```

Config asset:

- `src/robot_simulation/config/factory_patrol_localization_recovery.yaml`

Expected state labels to observe if the runtime conditions trigger them:

```text
LOCALIZATION_LOST -> LOCALIZATION_RECOVERING -> LOCALIZATION_RECOVERED -> LOCALIZATION_OK
```

Observed topics:

- `/localization/health`
- `/safety/state`
- `/safety/reason`
- `/amcl_pose`
- `/tf`

Expected safety linkage: `LOCALIZATION_LOST` should be visible through
`/safety/state` and should force zero command according to Phase 4B policy. This
must be verified from a real ROS2/Nav2 run; no result is filled here.

### Checks

Static workflow check:

```bash
bash scripts/check_factory_patrol_demo_workflows.sh
```

Runtime topic check after the demo and Nav2 are running:

```bash
bash scripts/check_factory_patrol_demo_runtime.sh
```

Current in Phase 5B:

- demo workflow scripts
- multipoint mission/profile asset
- temporary obstacle config and simple SDF model
- localization recovery pose config
- runtime and static check scripts
- acceptance documentation

Planned after Phase 5B:

- real Gazebo/RViz screenshots
- navigation success-rate statistics
- dynamic pedestrians or moving obstacles
- Nav2 keepout/speed filter integration for factory zones
- automatic mission pause/resume full closed loop

No Gazebo, RViz, Nav2, localization recovery, or obstacle-avoidance runtime
success is claimed by this phase.

## Phase 6 Showcase Boundary

Factory patrol assets are ready to support screenshots, videos, and report
figures, but Phase 6 only adds the placeholder index under `docs/showcase/`.
Future artifacts should record the exact launch command, commit, map/world,
parameters, and log or rosbag path before being cited in the README or report.
