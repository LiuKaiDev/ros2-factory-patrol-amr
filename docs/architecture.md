# System Architecture

本项目定位为面向厂区 / 园区半封闭场景的低速巡检 AMR 导航控制系统。架构重点不是展示大量业务入口，而是展示一条可解释、可调试、可扩展的规控闭环：目标点输入、地图加载、AMCL 定位、Nav2 规划与控制、速度仲裁、安全门控、底盘适配和状态反馈。

## Closed Loop

```mermaid
flowchart LR
  Goal[巡检点 / 目标点] --> Task[robot_tasks<br/>mission_runner / station task]
  Task --> Map[map_server<br/>indoor_room map]
  Map --> AMCL[AMCL localization<br/>/amcl_pose, map->odom]
  AMCL --> LocHealth[localization_health_monitor<br/>/localization/health]
  AMCL --> Planner[Nav2 global planner<br/>Navfn / SmacPlanner2D]
  Planner --> Controller[Nav2 local controller<br/>RPP / MPPI]
  Controller --> Mux[cmd_vel_mux / twist_mux<br/>speed arbitration]
  Tracking[Pure Pursuit / Stanley<br/>standalone experiment] --> Mux
  Teleop[teleop / virtual RC] --> Mux
  Mux --> Gate[cmd_vel_safety_gate<br/>watchdog / estop / speed limit]
  Gate --> Driver[chassis_driver_node]
  Driver --> Backend[Mock / Serial / UDP backend<br/>text / text_v2 / Mick binary]
  Backend --> Feedback[/odom / wheel odom + covariance<br/>/chassis/state / TF<br/>heartbeat / fault_code]
  Feedback --> AMCL
  LocHealth --> Monitor
  LocHealth --> Task
  Feedback --> Monitor[system_monitor]
  Monitor --> Fault[fault_supervisor]
  Fault --> Gate
```

## Package Responsibilities

| Package | Current responsibility |
| --- | --- |
| `robot_bringup` | 顶层 launch 入口，组合 display、bringup、tracking、amr_demo 等流程。 |
| `robot_description` | URDF / Xacro、RViz display 配置和机器人模型资源。 |
| `robot_hardware` | 底盘协议、Mock / Serial / UDP 后端、`chassis_driver_node`、仿真底盘节点、ros2_control SystemInterface、运动学参数和 odom covariance。 |
| `robot_sensors` | 激光、IMU 标准化，fake sensor source，基础诊断。 |
| `robot_navigation` | Nav2、AMCL、SLAM、地图、costmap、map manager、语义区配置和 localization health monitor。 |
| `robot_path_tracking` | 参考路径发布、Pure Pursuit、Stanley standalone 路径跟踪控制器。 |
| `robot_teleop` | cmd_vel 仲裁、安全门控、急停、键盘 / 虚拟遥控。 |
| `robot_tasks` | mission runner、站点任务、队列、恢复、充电、定位健康和重定位服务等任务层逻辑。 |
| `robot_simulation` | Gazebo world、仿真桥接、RViz marker、自动演示编排。 |
| `robot_utils` | system monitor、fault supervisor，连接诊断和急停闭环。 |
| `robot_experiments` | benchmark runner、参考路径和指标输出基础设施。 |
| `robot_interfaces*` | msg / srv / action 接口，按 core、navigation、mission、facility、fleet、business、site 等领域拆分。 |

## Current / Planned Boundary

| Area | Current | Planned / TBD |
| --- | --- | --- |
| Low-speed patrol positioning | 项目文档已统一定位；现有 indoor_room 仿真和站点任务可承载演示。 | 厂区 / 园区专用 `factory_patrol` 场景、地图、stations、zones。 |
| Navigation loop | Nav2 + AMCL + Navfn / Smac + RPP / MPPI 配置存在。 | Phase 1 系统化调参、costmap 能力验证和巡检场景参数收敛。 |
| Localization health | Phase 4A 已补充 `/amcl_pose` covariance、AMCL timeout 和 TF 检查，并发布 `/localization/health`。 | Phase 4B 将 `LOCALIZATION_LOST` 接入安全停车和任务暂停策略。 |
| Chassis adapter | Mock / Serial / UDP 后端和行文本协议存在；Phase 3A 已补充 `text_v2` 的 seq、timestamp、heartbeat、fault_code 和命令超时停车 hook；Phase 3B 已补充运动学参数统一、odom covariance 和标定流程文档。 | 真实底盘联调、完整故障码表、实车 covariance 估计和接口字段化。 |
| Safety | safety gate、watchdog、dynamic speed limit、manual takeover、fault supervisor 存在。 | 统一安全状态机、状态事件记录、任务暂停 / 恢复策略完整化。 |
| Experiment report | benchmark 基础设施和部分历史结果文件存在。 | 不复用为正式结论；后续按模板重新跑实验并记录。 |

## Phase 4B Safety Integration

The final `/cmd_vel` gate now resolves a unified safety state before publishing
commands to the chassis adapter:

```text
/localization/health + /scan + /odom + /chassis/state
  + /manual_takeover/state + legacy /safety_state
  -> cmd_vel_safety_gate_node
  -> /safety/state + /safety/reason
  -> gated /cmd_vel
```

High-risk states (`SENSOR_STALE`, `LOCALIZATION_LOST`, `CHASSIS_FAULT`,
`COMMUNICATION_LOST`, `EMERGENCY_STOP`) publish zero speed. `SPEED_LIMITED`
keeps the command path alive but clamps it to the low-speed policy. This phase
does not change mission-runner recovery behavior or add new factory-patrol
scenarios.

## Phase 5A Factory Patrol Assets

Simulation now includes a factory patrol asset skeleton alongside the existing
`indoor_room` demo:

```text
robot_simulation/worlds/factory_patrol.sdf
robot_simulation/config/factory_patrol_stations.yaml
robot_simulation/config/factory_patrol_zones.yaml
robot_simulation/config/factory_patrol_route.yaml
robot_bringup/launch/factory_patrol_demo.launch.py
```

The Phase 5A assets add a main patrol road, equipment area, narrow corridor,
turning area, static obstacles, `station_A/B/C`, and `dock` markers. The route
config is a multi-point patrol asset, but it is not claimed as a closed-loop
mission execution result yet.
