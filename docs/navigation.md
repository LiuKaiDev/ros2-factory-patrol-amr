# Navigation

本项目的导航链路基于 Nav2。Phase 0 不修改任何 Nav2 yaml，只整理当前配置现状和后续补强方向。

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
| `src/robot_navigation/config/nav2_basic.yaml` | `nav2_navfn_planner::NavfnPlanner` | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | global: static + inflation；local: inflation |
| `src/robot_navigation/config/nav2_advanced.yaml` | `nav2_smac_planner::SmacPlanner2D` | `nav2_mppi_controller::MPPIController` | global/local: voxel + inflation，含 footprint |

basic 配置适合展示清晰、低复杂度的导航闭环；advanced 配置适合展示更接近工程调参的 planner / controller / costmap 组合。

## Map and Costmap Relationship

- 静态地图由 map_server 加载，提供给 AMCL 和 global costmap。
- AMCL 输出 `map -> odom`，让全局规划与机器人局部里程计链路对齐。
- global costmap 负责路径可行区域，local costmap 负责机器人附近动态环境表达。
- controller_server 使用 local costmap 和全局路径输出速度候选值，后续仍需经过 cmd_vel 仲裁和 safety gate。

## Phase 1 Plan

Phase 1 计划补强但本阶段不修改参数：

| Item | Status |
| --- | --- |
| obstacle / voxel layer 的输入源、清障行为和仿真障碍验证 | planned |
| inflation radius 与 footprint 对低速巡检通道的适配 | planned |
| RPP 参数：lookahead、速度限制、goal tolerance、曲率约束 | planned |
| MPPI 参数：采样规模、速度上下限、角速度约束、critic 配置 | planned |
| Navfn / Smac 路径质量对比 | planned |
| costmap 可视化与验收脚本沉淀 | planned |

所有实验数据必须在真实运行后写入报告，不能在文档中预设结果。
