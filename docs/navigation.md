# Navigation

本项目的导航链路基于 Nav2。Phase 1A 已补强 basic Nav2 配置，使其更适合低速巡检 AMR 的规控岗位展示：local costmap 增加 LaserScan obstacle layer，补充 footprint、inflation scaling 和 RPP regulated 参数。Phase 1B 增加 Nav2 basic 调试 RViz 配置和最小验收脚本，方便检查 `/scan`、TF、odom、costmap、global path 和 cmd_vel 链路。

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

本阶段没有在 basic global costmap 中加入 obstacle layer。原因是 basic 配置的定位是清晰、可解释的低速巡检导航闭环：全局路径由静态地图和 footprint 约束给出，近场动态障碍由 local obstacle layer 处理。若后续需要让动态障碍触发全局绕行，可在后续调参阶段增加 global obstacle layer 并配套验证。

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

## Phase 1B RViz Debug View

已有 RViz 配置审查：

- `src/robot_simulation/rviz/amr_sim.rviz`：已包含 `/scan`、TF、`/odom` 和 odom path 等显示项。
- `src/robot_description/rviz/display.rviz`：偏机器人模型 / TF 显示。

Phase 1B 新增专用调试配置：

- `src/robot_simulation/rviz/nav2_basic_debug.rviz`

该配置面向 Nav2 basic 调试，包含：

| Display | Topic / frame | Purpose |
| --- | --- | --- |
| Robot Model | `/robot_description` | 检查机器人模型和 footprint 近似外形。 |
| TF | `/tf`、`/tf_static` | 检查 `map -> odom -> base_footprint -> base_link` 链路。 |
| Laser Scan | `/scan` | 检查标准激光输入是否存在。 |
| Odometry | `/odom` | 检查底盘或 mock 后端反馈。 |
| Wheel Odometry | `/wheel/odom`，默认关闭 | 需要底盘链路发布时可打开。 |
| Map | `/map` | 检查 map_server 输出。 |
| Global Costmap | `/global_costmap/costmap` | 检查静态地图、footprint 和 inflation 组成的全局代价图。 |
| Local Costmap | `/local_costmap/costmap` | 检查 `/scan` obstacle layer、clearing 和 inflation 是否进入局部代价图。 |
| Global Path | `/plan` | 检查 planner_server 输出的全局路径。 |
| AMCL Pose | `/amcl_pose` | 检查定位输出。 |
| Goal Pose | `/goal_pose` | 配合 2D Goal Pose 工具使用。 |

RViz 工具栏配置了 2D Pose Estimate (`/initialpose`) 和 2D Goal Pose (`/goal_pose`)。local plan / trajectory topic 在当前仓库中没有稳定确认的 topic 名称，本阶段不硬写，后续运行验证后再补充。

## Phase 1B Validation Scripts

静态配置检查：

```bash
bash scripts/check_nav2_costmap_obstacle_layer.sh
```

该脚本不依赖完整 ROS2 运行环境，只检查 `src/robot_navigation/config/nav2_basic.yaml` 是否包含 local obstacle layer、`/scan` observation source、footprint、inflation layer 和 RPP collision detection。

Nav2 basic 分步启动提示：

```bash
bash scripts/run_nav2_basic_demo.sh
```

脚本会检查 `ros2` 是否存在、可选 source `install/setup.bash`，并输出三步命令：mock bringup、Nav2 basic、RViz debug。若只想在当前终端直接启动 Nav2，可运行：

```bash
bash scripts/run_nav2_basic_demo.sh --run-nav2
```

runtime topic 检查，需要 Nav2 basic demo 已经运行：

```bash
bash scripts/check_nav2_runtime_topics.sh
```

该脚本检查 `/scan`、`/tf`、`/tf_static`、`/odom`、`/map`、`/plan`、`/cmd_vel`、`/local_costmap/costmap`、`/global_costmap/costmap` 是否存在。它需要真实 ROS2 graph，不能用静态文件检查替代。

## Runtime Verification Notes

- 验证 `/scan` 是否进入 local costmap：启动 Nav2 basic 后，在 RViz 打开 Laser Scan 和 Local Costmap；移动或模拟障碍应改变 `/local_costmap/costmap`。该结论需要 Gazebo / mock sensor 运行验证。
- 验证 footprint / inflation 是否生效：在 RViz 中叠加 Robot Model、Global Costmap 和 Local Costmap，观察机器人外形附近是否形成合理代价边界。具体半径和通道通过性需要后续调参。
- 验证 global path：发送 2D Goal Pose 后观察 `/plan` 的 Global Path 是否生成，并检查 planner_server 日志。
- 验证 cmd_vel 链路：Nav2 controller 输出经 cmd_vel 仲裁和 safety gate 后，应在完整 bringup 中出现 `/cmd_vel`。单独启动 Nav2 时可能只有 `/nav2_cmd_vel`，因此 runtime topic 检查应在完整 basic demo 链路运行后执行。
- 本文档不记录实验成功率、到达时间或误差指标；这些属于后续真实运行报告。

## Remaining Phase 1 / Later Plan

| Item | Status |
| --- | --- |
| 启动 Nav2 basic 并验证 obstacle layer 订阅 `/scan` | planned runtime verification |
| RViz 中显示 local / global costmap 与路径 topic | current debug config / runtime verification planned |
| 对比 basic global costmap 是否需要加入 obstacle layer | planned |
| inflation radius 与 footprint 对低速巡检通道的实测适配 | planned |
| RPP 参数在真实 / 仿真巡检路径中的调参收敛 | planned |
| MPPI 参数：采样规模、速度上下限、角速度约束、critic 配置 | planned |
| Navfn / Smac 路径质量对比 | planned |
| costmap 可视化与验收脚本沉淀 | planned |

所有实验数据必须在真实运行后写入报告，不能在文档中预设结果。
