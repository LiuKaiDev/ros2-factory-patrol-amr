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
