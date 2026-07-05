# Navigation

本项目的导航链路基于 Nav2。Phase 1A 已补强 basic Nav2 配置，使其更适合低速巡检 AMR 的规控岗位展示：local costmap 增加 LaserScan obstacle layer，补充 footprint、inflation scaling 和 RPP regulated 参数；advanced 配置继续保留为更复杂的 Smac + MPPI + voxel layer 组合。

## Nav2 Components

```text
map_server -> AMCL -> planner_server -> controller_server -> /cmd_vel candidate
```

- `map_server`：加载静态地图，例如 `src/robot_navigation/maps/indoor_room.yaml`。
- `AMCL`：基于 `/scan` 和地图估计 `map -> odom`。
- `planner_server`：根据全局 costmap 生成全局路径。
- `controller_server`：根据局部 costmap 和全局路径生成局部速度指令。
- `behavior_server`：提供 spin、backup、wait 等恢复 / 行为插件。
- `global_costmap`：面向全局路径搜索，使用 map frame。
- `local_costmap`：面向局部避障与控制，使用 odom frame 和 rolling window。

## Current Configurations

| File | Global planner | Local controller | Costmap status |
| --- | --- | --- | --- |
| `src/robot_navigation/config/nav2_basic.yaml` | `nav2_navfn_planner::NavfnPlanner` | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | global: static + inflation + footprint；local: obstacle + inflation + footprint |
| `src/robot_navigation/config/nav2_advanced.yaml` | `nav2_smac_planner::SmacPlanner2D` | `nav2_mppi_controller::MPPIController` | global/local: voxel + inflation，含 footprint |

basic 配置适合展示清晰、低复杂度的导航闭环：Navfn 做全局规划，RPP 做低速局部跟随，local costmap 使用 2D LaserScan obstacle layer 表达近场动态障碍。advanced 配置适合展示更接近工程调参的组合：SmacPlanner2D、MPPI 和 voxel layer。

## Map and Costmap Relationship

- 静态地图由 map_server 加载，提供给 AMCL 和 global costmap。
- AMCL 输出 `map -> odom`，让全局规划与机器人局部里程计链路对齐。
- global costmap 负责路径可行区域，local costmap 负责机器人附近动态环境表达。
- controller_server 使用 local costmap 和全局路径输出速度候选值，后续仍需经过 cmd_vel 仲裁和 safety gate。

## Phase 1A Basic Costmap Update

`src/robot_navigation/config/nav2_basic.yaml` 当前补强内容：

| Area | Current setting | Purpose |
| --- | --- | --- |
| robot frame | `base_footprint` | 与现有 AMCL、Nav2 和 TF 主链保持一致。 |
| footprint | `[[-0.32, -0.18], [-0.32, 0.18], [0.32, 0.18], [0.32, -0.18]]` | 用矩形 footprint 表达低速 AMR 外形，供 collision checking 和 costmap 膨胀参考。 |
| local costmap | `global_frame: odom`、`rolling_window: true`、`5m x 5m`、`0.05m` resolution | 用机器人附近滚动窗口处理局部障碍和局部控制。 |
| local obstacle layer | `nav2_costmap_2d::ObstacleLayer` | 将近场激光障碍写入 local costmap。 |
| LaserScan source | `observation_sources: scan`，topic `/scan`，`marking: true`，`clearing: true` | 使用项目标准化后的 `/scan` 作为障碍标记和清除来源。 |
| local inflation layer | `inflation_radius: 0.45`，`cost_scaling_factor: 3.0` | 在障碍附近形成代价梯度，使 RPP 避免贴边通过。 |
| global costmap | static + inflation + footprint | 保持全局规划主要基于静态地图，动态障碍优先交给 local costmap 处理。 |

本阶段没有在 basic global costmap 中加入 obstacle layer。原因是 basic 配置的定位是清晰、可解释的低速巡检导航闭环：全局路径由静态地图和 footprint 约束给出，近场动态障碍由 local obstacle layer 处理。若后续需要让动态障碍触发全局绕行，可在 Phase 1B / 后续调参中增加 global obstacle layer 并配套验证。

## Phase 1A RPP Update

basic `FollowPath` 继续使用 `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`，并补充以下低速巡检参数：

| Parameter group | Settings | Purpose |
| --- | --- | --- |
| lookahead | `lookahead_dist: 0.8`、`min_lookahead_dist: 0.35`、`max_lookahead_dist: 1.0`、`lookahead_time: 1.5`、`use_velocity_scaled_lookahead_dist: true` | 低速时缩短前视距离，高速时适度拉长，兼顾巡检通道跟随和轨迹平滑。 |
| rotate to heading | `rotate_to_heading_angular_vel: 0.8` | 限制转向角速度，避免低速平台原地调整过猛。 |
| regulated velocity scaling | `use_regulated_linear_velocity_scaling: true`、`regulated_linear_scaling_min_radius: 0.7`、`regulated_linear_scaling_min_speed: 0.08` | 曲率较大时降低线速度，保留最小爬行速度。 |
| cost regulated scaling | `use_cost_regulated_linear_velocity_scaling: true`、`cost_scaling_dist: 0.6`、`cost_scaling_gain: 1.0` | 靠近高代价区域时降低速度。 |
| collision detection | `use_collision_detection: true`、`max_allowed_time_to_collision_up_to_carrot: 1.5` | 沿 carrot 方向做短时碰撞预测，发现风险时约束控制输出。 |

这些参数仍需在 ROS2 Jazzy + 具体 Nav2 版本中通过运行验证；Phase 1A 只提交配置补强和静态检查脚本，不声称实验结果。

## RViz Review

已有 RViz 配置：

- `src/robot_simulation/rviz/amr_sim.rviz`：已包含 `/scan`、TF、`/odom` 和 odom path 等显示项。
- `src/robot_description/rviz/display.rviz`：偏机器人模型 / TF 显示。

当前未修改 RViz 文件。global / local costmap、global path、local plan 或 cmd_vel 可视化可在后续阶段按实际运行 topic 补充，避免在未确认 topic 命名前硬改配置。

## Remaining Phase 1 / Later Plan

| Item | Status |
| --- | --- |
| 启动 Nav2 basic 并验证 obstacle layer 订阅 `/scan` | planned |
| RViz 中显示 local / global costmap 与路径 topic | planned |
| 对比 basic global costmap 是否需要加入 obstacle layer | planned |
| inflation radius 与 footprint 对低速巡检通道的实测适配 | planned |
| RPP 参数在真实 / 仿真巡检路径中的调参收敛 | planned |
| MPPI 参数：采样规模、速度上下限、角速度约束、critic 配置 | planned |
| Navfn / Smac 路径质量对比 | planned |
| costmap 可视化与验收脚本沉淀 | planned |

所有实验数据必须在真实运行后写入报告，不能在文档中预设结果。
