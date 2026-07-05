# Low-Speed Patrol AMR Navigation Control System

面向厂区 / 园区半封闭场景的低速巡检 AMR 导航控制系统，基于 ROS2 Jazzy + C++17 构建。项目重点展示移动机器人规控岗位相关能力：AMCL 定位、Nav2 全局规划、RPP / MPPI 局部控制、cmd_vel 速度仲裁、安全门控、底盘通信适配、里程计 / TF / 底盘状态反馈，以及 Gazebo / RViz 仿真验证。

当前仓库已有较完整的 ROS2 AMR 工程骨架，包含 Nav2、AMCL、Gazebo、RViz、Mock / Serial / UDP 底盘后端、速度仲裁、安全门控、任务调度、站点 / 充电 / 车队、REST、VDA5050 mock 桥接等模块。本阶段文档将项目定位收敛到“低速巡检 AMR 导航控制系统”，并明确区分 current、mock、planned / TBD 内容。

## 应用场景：厂区 / 园区半封闭低速巡检

目标场景是厂区、园区、仓储通道、实验室走廊等半封闭环境中的低速巡检任务：

- 按巡检点或目标点序列运行；
- 基于静态地图进行定位和全局路径规划；
- 在局部控制层输出速度指令；
- 通过速度仲裁和安全门控约束最终 `/cmd_vel`；
- 经底盘适配层接入 Mock / Serial / UDP 后端；
- 通过 `/odom`、`/wheel/odom`、`/chassis/state`、TF 和诊断链路形成反馈闭环。

不把当前项目描述为量产级无人车、VCU 固件或真实工厂部署系统。真实硬件联调、性能指标和现场标定结果需要后续阶段补充。

## 系统架构

核心闭环：

```text
巡检点 / 目标点
  -> map_server 地图加载
  -> AMCL 定位
  -> localization_health_monitor /localization/health
  -> Nav2 全局规划 Navfn / Smac
  -> RPP / MPPI 局部控制
  -> cmd_vel_mux 速度仲裁
  -> cmd_vel_safety_gate 安全门控
  -> chassis_driver 底盘驱动
  -> Mock / Serial / UDP 后端
  -> /odom、/wheel/odom、/chassis/state、TF
  -> system_monitor、fault_supervisor、安全停车
```

详细架构见 [docs/architecture.md](docs/architecture.md)。

## 核心闭环链路

| 环节 | 当前实现 | 说明 |
| --- | --- | --- |
| 地图 | current | `robot_navigation/maps/indoor_room.yaml`，Nav2 map_server 通过 launch 加载 |
| 定位 | current | Nav2 AMCL，订阅 `/scan`，发布 `map -> odom`；Phase 4A 增加 localization health monitor，发布 `/localization/health` |
| 全局规划 | current | `nav2_basic.yaml` 使用 Navfn，`nav2_advanced.yaml` 使用 SmacPlanner2D |
| 局部控制 | current | basic 使用 RPP，advanced 使用 MPPI |
| 路径跟踪实验 | current | standalone Pure Pursuit / Stanley 控制器与参考路径发布 |
| 速度仲裁 | current | `cmd_vel_mux_node` / `twist_mux` 相关链路 |
| 安全门控 | current | `cmd_vel_safety_gate_node` 支持急停、watchdog、动态限速和人工接管状态 |
| 底盘适配 | current | `chassis_driver_node` + Mock / Serial / UDP 后端，另有 ros2_control SystemInterface |
| 反馈 | current | `/odom`、`/wheel/odom`、`/chassis/state`、TF、system health |
| 安全状态机 | partial current / planned | 已有安全门控和 fault supervisor；完整安全状态机文档见 [docs/safety_state_machine.md](docs/safety_state_machine.md) |

## 快速启动

依赖 ROS2 Jazzy（Ubuntu 24.04）。首次构建：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Docker 复现环境：

```bash
docker build -t robot-amr -f docker/Dockerfile .
docker run -it --rm robot-amr bash
```

低速巡检相关启动入口：

```bash
# Gazebo + RViz + 自动演示编排
ros2 launch robot_bringup amr_demo.launch.py gui:=true use_rviz:=true

# Mock 底盘 + 传感器 + 安全链路
ros2 launch robot_bringup bringup.launch.py mode:=hardware backend:=mock

# Nav2 导航
ros2 launch robot_navigation nav.launch.py navigation_start_delay:=5.0

# 任务 / 巡检点调度入口
ros2 launch robot_tasks mission_runner.launch.py autostart:=true

# Pure Pursuit / Stanley standalone 路径跟踪实验
ros2 launch robot_bringup tracking.launch.py controller:=pure_pursuit
ros2 launch robot_bringup tracking.launch.py controller:=stanley
```

真实 Serial / UDP 底盘接入入口存在，但真实硬件参数、协议版本和标定数据需要按设备补齐：

```bash
ros2 launch robot_bringup bringup.launch.py mode:=hardware backend:=serial dev:=/dev/ttyUSB0 baud:=115200
ros2 launch robot_bringup bringup.launch.py mode:=hardware backend:=udp ip:=192.168.1.10 port:=9000
```

## 核心模块

### Navigation

- `robot_navigation` 提供 Nav2、AMCL、SLAM、地图管理、语义区和 costmap 相关配置。
- `nav2_basic.yaml` 当前使用 NavfnPlanner + Regulated Pure Pursuit，并已补强 local obstacle layer、footprint、`/scan` observation source、inflation 和 RPP collision checks。
- `nav2_advanced.yaml` 当前使用 SmacPlanner2D + MPPI，并包含 voxel / inflation / footprint 配置。
- Nav2 basic 调试视图：`src/robot_simulation/rviz/nav2_basic_debug.rviz`。
- 最小检查脚本：`scripts/check_nav2_costmap_obstacle_layer.sh`、`scripts/run_nav2_basic_demo.sh`、`scripts/check_nav2_runtime_topics.sh`。
- 详情见 [docs/navigation.md](docs/navigation.md)。

### Localization

- 当前导航定位以 AMCL 为主，围绕 `/amcl_pose`、`map -> odom`、`odom -> base_footprint` 建立定位链路。
- Phase 4A 新增 `localization_health_monitor_node`，检查 `/amcl_pose` timeout、covariance 和 `map -> odom -> base_footprint` TF，发布 `/localization/health`。
- `mission_runner` 中已有 LOST / RECOVERED、重定位服务相关逻辑；完整安全停车 / 任务暂停接入留到 Phase 4B。
- 静态检查脚本：`scripts/check_localization_health.sh`；runtime topic 检查：`scripts/check_localization_runtime_topics.sh`。
- 详情见 [docs/localization.md](docs/localization.md)。

### Control

- Nav2 controller：RPP / MPPI 负责导航闭环下的局部速度输出。
- standalone tracking：Pure Pursuit / Stanley 用于路径跟踪实验和控制器对比，已支持可选 CSV 指标输出。
- CSV 分析 / 对比 / 绘图脚本：`scripts/analyze_tracking_result.py`、`scripts/compare_tracking_results.py`、`scripts/plot_tracking_result.py`。
- tracking demo / 分析流程提示脚本：`scripts/run_tracking_experiment_demo.sh`、`scripts/run_tracking_analysis_workflow.sh`。
- 后续实验指标不在 README 中伪造，统一用 TBD 模板记录。
- 详情见 [docs/control.md](docs/control.md) 和 [docs/experiment_report.md](docs/experiment_report.md)。

### Chassis Adapter

- `robot_hardware` 提供底盘协议编解码、Mock / Serial / UDP 后端、`chassis_driver_node`、`chassis_simulator_node` 和 ros2_control 插件。
- 当前协议包含 `CMD`、`ODOM`、`STATE` 行文本协议、Phase 3A `text_v2` 的 `CMDV2` / `ODOMV2` / `STATEV2` / `HEARTBEATV2` 最小骨架、UDP simulator v2 兼容入口，以及部分 Mick binary frame 支持。
- Phase 3A 新增 heartbeat、fault_code、seq / timestamp 和 `/cmd_vel` 超时停车 hook；真实底盘故障码表和现场联调结论仍需后续补齐。
- Phase 3B 补充 `wheel_diameter_m` / `track_width_m` 参数统一、`/odom` 和 `/wheel/odom` covariance、低速上限参数和标定流程文档。
- 静态检查脚本：`scripts/check_chassis_protocol_v2.sh`、`scripts/check_chassis_odom_calibration.sh`。
- 这是 ROS2 上位机到底盘控制器的通信适配，不是 VCU 固件开发。
- 详情见 [docs/chassis_protocol.md](docs/chassis_protocol.md) 和 [docs/calibration.md](docs/calibration.md)。

### Safety

- `robot_teleop` 中的 `cmd_vel_safety_gate_node` 统一发布最终 `/cmd_vel`，支持 emergency stop、watchdog、manual takeover、dynamic speed limit。
- `robot_utils` 中的 `system_monitor_node` 和 `fault_supervisor_node` 构成诊断到急停的闭环。
- 详情见 [docs/safety_state_machine.md](docs/safety_state_machine.md)。

### Simulation

- `robot_simulation` 包含 Gazebo world、仿真桥接、RViz marker 可视化和自动演示编排。
- 当前场景为 `indoor_room.sdf` / `indoor_room` map；厂区巡检场景 `factory_patrol.sdf`、factory map、stations、zones 是 planned。
- 详情见 [docs/simulation_scenarios.md](docs/simulation_scenarios.md)。

## Demo 场景

| Demo | 状态 | 说明 |
| --- | --- | --- |
| Gazebo / RViz AMR showcase | current | `robot_bringup amr_demo.launch.py`，基于现有 indoor_room 场景 |
| Mock 底盘闭环 | current | `bringup.launch.py backend:=mock` |
| Nav2 basic / advanced 对比 | current config / planned report | 配置存在，系统化指标报告待补充 |
| Pure Pursuit / Stanley 路径跟踪 | current code / planned report | 控制器和 benchmark runner 存在，实验报告模板为 TBD |
| 多点巡检 | partial current / planned factory demo | 当前可通过任务 / 站点链路表达，厂区巡检地图与脚本待 Phase 5 补齐 |
| 临时障碍避障 | planned factory demo | 后续结合 costmap 与场景脚本补强 |
| 定位丢失恢复 | current logic / planned scenario | localization health 与重定位流程已有代码入口，厂区演示脚本待补齐 |

## 实验与评估规划

本仓库已有 `robot_experiments`、`scripts/run_*benchmark.sh` 和部分历史 CSV / JSON 结果文件，但 Phase 0 不新增、不美化、不伪造实验结果。后续实验报告统一采用 [docs/experiment_report.md](docs/experiment_report.md) 模板，指标包括：

- `success_rate`
- `arrival_time`
- `final_error`
- `rms_lateral_error`
- `max_lateral_error`
- `mean_heading_error`
- `max_angular_velocity`
- `stop_count`

标定思路见 [docs/calibration.md](docs/calibration.md)。

## 测试与 CI

当前测试形态：

- GoogleTest：底盘协议、控制数学、导航地图 / zone、任务调度 workflow / behavior tree 等；
- launch_testing：bringup、Nav2 / SLAM mock runtime、fault supervisor 等；
- shell 验收脚本：`scripts/check_*.sh` 覆盖导航、仿真、任务、安全、REST、VDA5050 mock 等链路；
- 底盘协议 v2 静态检查：`bash scripts/check_chassis_protocol_v2.sh`；
- 底盘 odom / 标定静态检查：`bash scripts/check_chassis_odom_calibration.sh`；
- Localization health 静态检查：`bash scripts/check_localization_health.sh`；
- GitHub Actions：`.github` 目录存在，具体 CI 能力以后续 workflow 为准。

Phase 0 仅调整文档，不修改测试、launch、参数、源码或 CI 配置。

## Roadmap

完整路线图见 [docs/roadmap.md](docs/roadmap.md)。

| Phase | 优先级 | 目标 | 状态 |
| --- | --- | --- | --- |
| Phase 0 | P0 | 项目定位统一与文档重构 | current |
| Phase 1 | P0 | Nav2 与 costmap 能力补强 | planned |
| Phase 2 | P0 | 控制算法实验体系 | planned |
| Phase 3 | P1 | 底盘通信与 odom / TF 工程化 | Phase 3B current: protocol v2 + odom covariance / calibration support |
| Phase 4 | P1 | 定位、重定位与安全状态机 | Phase 4A current: localization health monitor |
| Phase 5 | P1 | 厂区巡检仿真场景 | planned |
| Phase 6 | P2 | 实验报告、CI、最终展示 | planned |

## 面试可讲亮点

面试讲解口径见 [docs/interview_notes.md](docs/interview_notes.md)。建议主线：

1. 应用场景：半封闭厂区 / 园区低速巡检；
2. 系统闭环：目标点到 `/cmd_vel` 再到底盘反馈；
3. 算法选型：AMCL、Navfn / Smac、RPP / MPPI、Pure Pursuit / Stanley；
4. 工程落地：ROS2 package 拆分、launch、接口、Mock / Serial / UDP 后端；
5. 安全保护：速度仲裁、安全门控、故障监督、异常停车；
6. 实验指标：只讲模板和后续计划，不伪造数据；
7. 项目边界：当前是导航控制工程展示系统，不声称真实量产部署。

## 文档索引

- [System Architecture](docs/architecture.md)
- [Navigation](docs/navigation.md)
- [Localization](docs/localization.md)
- [Control](docs/control.md)
- [Chassis Protocol](docs/chassis_protocol.md)
- [Safety State Machine](docs/safety_state_machine.md)
- [Simulation Scenarios](docs/simulation_scenarios.md)
- [Experiment Report Template](docs/experiment_report.md)
- [Calibration](docs/calibration.md)
- [Interview Notes](docs/interview_notes.md)
- [Roadmap](docs/roadmap.md)

## Phase 4B Safety

Phase 4B adds a minimal unified safety state integration in
`cmd_vel_safety_gate_node`. It publishes `/safety/state` and `/safety/reason`,
monitors `/localization/health`, `/scan`, `/odom`, and `/chassis/state`, and
keeps the existing emergency-stop, watchdog, manual takeover, and dynamic
speed-limit behavior.

Safety checks:

```bash
bash scripts/check_safety_state_machine.sh
bash scripts/check_safety_runtime_topics.sh
```

Details: [docs/safety_state_machine.md](docs/safety_state_machine.md).

## Factory Patrol Demo

Phase 5A adds factory patrol simulation assets:

- `src/robot_simulation/worlds/factory_patrol.sdf`
- `src/robot_simulation/config/factory_patrol_stations.yaml`
- `src/robot_simulation/config/factory_patrol_zones.yaml`
- `src/robot_simulation/config/factory_patrol_route.yaml`
- `src/robot_bringup/launch/factory_patrol_demo.launch.py`
- `scripts/run_factory_patrol_demo.sh`
- `scripts/check_factory_patrol_assets.sh`

Run helper:

```bash
bash scripts/run_factory_patrol_demo.sh
```

Static asset check:

```bash
bash scripts/check_factory_patrol_assets.sh
```

Details: [docs/simulation_scenarios.md](docs/simulation_scenarios.md).

## Phase 5B Factory Patrol Workflows

Factory Patrol demo workflow entries:

- Multipoint Patrol: `bash scripts/run_factory_patrol_multipoint_demo.sh`
- Print route goals: `python3 scripts/print_factory_patrol_goals.py`
- Temporary Obstacle: `bash scripts/run_factory_patrol_obstacle_demo.sh`
- Localization Recovery: `bash scripts/run_factory_patrol_localization_recovery_demo.sh`
- Static workflow check: `bash scripts/check_factory_patrol_demo_workflows.sh`
- Runtime topic check: `bash scripts/check_factory_patrol_demo_runtime.sh`

Docs:

- [Simulation Scenarios](docs/simulation_scenarios.md)
- [Experiment Report Template](docs/experiment_report.md)
