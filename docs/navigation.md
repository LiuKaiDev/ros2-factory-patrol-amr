# 导航系统

## 系统概览

导航链路以 Nav2 为唯一规划与局部控制栈，面向半封闭室内或厂区低速巡检。AMCL 提供
`map -> odom`，Nav2 根据地图、激光和机器人 footprint 生成路径与速度候选，候选命令再
经过速度仲裁和 Safety Gate 才能到达底盘：

```text
map_server + /scan -> AMCL -> planner_server -> controller_server
                                      -> /nav2_cmd_vel
                                      -> cmd_vel mux -> Safety Gate -> /cmd_vel
```

## 导航与控制数据流

- `map_server` 加载静态地图，供 AMCL 与 global costmap 使用。
- `AMCL` 订阅 `/scan`，发布 `/amcl_pose` 和 `map -> odom`。
- `planner_server` 在 global costmap 上生成全局路径。
- `controller_server` 结合 local costmap 和全局路径生成 `/nav2_cmd_vel`。
- `behavior_server` 提供 spin、backup、wait 等行为插件。
- `cmd_vel_mux_node`（Factory Patrol 仿真）或 `twist_mux`（实体底盘 bringup）选择速度源。
- `cmd_vel_safety_gate_node` 解析 watchdog、急停、定位、底盘、传感器和感知安全输入，发布
  唯一的最终 `/cmd_vel`。

## Nav2 组件

| 配置 | Global planner | Local controller | Costmap 组合 |
| --- | --- | --- | --- |
| `src/robot_navigation/config/nav2_basic.yaml` | `nav2_navfn_planner::NavfnPlanner` | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | global: static + inflation + footprint；local: obstacle + inflation + footprint |
| `src/robot_navigation/config/nav2_advanced.yaml` | `nav2_smac_planner::SmacPlanner2D` | `nav2_mppi_controller::MPPIController` | global/local: voxel + inflation + footprint |

basic 配置强调低速巡检闭环的可解释性；advanced 配置提供 SmacPlanner2D、MPPI 和 voxel
layer 的工程化组合。两者都使用相同的 `map`、`odom`、`base_footprint` frame 约定。

## Localization / AMCL

AMCL 使用 `/scan` 和静态地图估计 `map -> odom`；底盘或 EKF 提供 `odom -> base_footprint`，
URDF 提供 `base_footprint -> base_link`。定位健康由
`localization_health_monitor_node` 汇总为 `/localization/health`，并由 Safety Gate 映射为
限速、停车或恢复状态。详细阈值见 [定位系统](localization.md)。

## Global Planner

basic 使用 Navfn：

| Parameter | Value |
| --- | --- |
| Plugin | `nav2_navfn_planner::NavfnPlanner` |
| `tolerance` | `0.5 m` |
| `use_astar` | `false` |
| Global costmap | `map` frame，static layer + inflation layer + footprint |
| Global inflation | `inflation_radius: 0.55`，`cost_scaling_factor: 3.0` |

advanced 使用 SmacPlanner2D：

| Parameter | Value |
| --- | --- |
| Plugin | `nav2_smac_planner::SmacPlanner2D` |
| `tolerance` | `0.25 m` |
| `downsample_costmap` | `false` |
| Global costmap | static layer + voxel layer + inflation layer |
| Voxel range | `z_resolution: 0.05`，`z_voxels: 12`，`max_obstacle_height: 1.2` |
| Inflation | `inflation_radius: 0.65` |

basic global costmap 保持静态地图为主要全局障碍来源，近场动态障碍由 local costmap 处理。
更复杂的全局动态障碍策略属于 [项目路线图](roadmap.md) 中的后续工作。

## Local Controller

basic 的 RPP 参数：

| Parameter group | Settings |
| --- | --- |
| Lookahead | `lookahead_dist: 0.8`，`min_lookahead_dist: 0.35`，`max_lookahead_dist: 1.0`，`lookahead_time: 1.5`，`use_velocity_scaled_lookahead_dist: true` |
| Heading | `rotate_to_heading_angular_vel: 0.8` |
| Curvature scaling | `regulated_linear_scaling_min_radius: 0.7`，`regulated_linear_scaling_min_speed: 0.08` |
| Cost scaling | `cost_scaling_dist: 0.6`，`cost_scaling_gain: 1.0` |
| Collision check | `use_collision_detection: true`，`max_allowed_time_to_collision_up_to_carrot: 1.5` |

advanced 的 MPPI 参数为 `time_steps: 48`、`model_dt: 0.05`、`batch_size: 1000`、
`vx_max: 0.55`、`wz_max: 1.5`。RPP/MPPI 的独立性能比较尚未形成统一的实测结论，不能仅凭
配置文件宣称某一控制器更优。

## Global / Local Costmap

两种配置都使用矩形 footprint：

```text
[[-0.32, -0.18], [-0.32, 0.18], [0.32, 0.18], [0.32, -0.18]]
```

basic local costmap 使用 `odom` frame 的 `5 m x 5 m` rolling window，分辨率 `0.05 m`，
`nav2_costmap_2d::ObstacleLayer` 订阅 `/scan`，同时开启 marking 和 clearing；障碍最大
范围为 `3.0 m`，raytrace 最大范围为 `4.0 m`。Inflation 为 `0.45 m`、cost scaling `3.0`。

advanced local costmap 使用 voxel layer：`z_resolution: 0.05`、`z_voxels: 12`、
`max_obstacle_height: 1.2 m`、inflation `0.5 m`。global costmap 分别使用 static + inflation
或 static + voxel + inflation。机器人 frame 均为 `base_footprint`。

## 当前关键参数

| Item | Basic | Advanced |
| --- | --- | --- |
| Controller frequency | `20 Hz` | `30 Hz` |
| Goal tolerance | `0.15 m / 0.2 rad` | `0.18 m / 0.18 rad` |
| Local costmap update/publish | `8 / 4 Hz` | `8 / 4 Hz` |
| Global costmap update/publish | `1 / 1 Hz` | `2 / 1 Hz` |
| Planner frequency | `5 Hz` | `10 Hz` |

参数源分别是 `nav2_basic.yaml` 和 `nav2_advanced.yaml`；文档中的值与配置保持同步。

## RViz 调试

`src/robot_simulation/rviz/nav2_basic_debug.rviz` 是当前可用的 Nav2 调试视图，包含：

- Robot Model、TF、`/scan`、`/odom`、`/map` 和 `/amcl_pose`；
- `/global_costmap/costmap`、`/local_costmap/costmap` 和 `/plan`；
- 2D Pose Estimate (`/initialpose`) 与 2D Goal Pose (`/goal_pose`) 工具。

Factory Patrol 的 `factory_patrol_showcase.rviz` 用于非 Nav2 场景展示；RGB-D 图像显示已在
Factory Patrol 视图中提供，Depth 默认关闭。

## 与视觉巡检任务的集成

视觉任务不会绕过 Nav2：

```text
CONFIRMED Target
  -> PerceptionEvent
  -> robot_tasks
  -> Observation Pose
  -> /navigate_sequence
  -> Nav2 NavigateToPose
  -> /nav2_cmd_vel
  -> cmd_vel mux -> Safety Gate -> /cmd_vel
```

`robot_tasks` 拥有事件校验、Observation Pose、action lifecycle、重试和完成/失败决定。
Perception 只提供 map-frame 目标和语义事件，不发布任何速度命令。

## 验证方法

静态配置检查：

```bash
bash scripts/check_nav2_costmap_obstacle_layer.sh
```

分步启动提示与运行时 topic 检查：

```bash
bash scripts/run_nav2_basic_demo.sh
bash scripts/check_nav2_runtime_topics.sh
```

完整 Factory Patrol 运行还可使用 `scripts/check_factory_patrol_runtime_topics.sh`，并在
RViz 中观察 `/scan`、costmap、路径、TF 与最终 `/cmd_vel`。WSL2 全量验证结果和边界见
[实验与 Benchmark 报告](experiment_report.md)。上述 basic/advanced 参数的独立专项运行
覆盖范围不同，未单独记录的配置项应标记为“尚未进行独立运行时专项验证”。

## 已知限制

- 定量结果来自 WSL2/Gazebo，不代表实体硬件导航性能。
- Factory Patrol 没有提交经过审阅的 occupancy map；默认展示 profile 可不启用 Nav2。
- Gazebo settling 与积分 odometry 可能有不同 origin，影响几何和路径误差解释。
- RPP 与 MPPI 的统一对比、全局动态障碍层和现场调参属于未来工作。
