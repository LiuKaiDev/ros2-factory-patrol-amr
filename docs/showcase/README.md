# Showcase 证据规范

本目录保存审阅过的 Showcase 证据和 media 管理规则。定量 JSON/CSV artifact 位于
`src/robot_experiments/results/`；当前未提交经过审阅的 screenshot、GIF 或 video。

只有当 screenshot、video、CSV-derived figure 或 Demo claim 关联真实 command、commit、
parameter set、map/world 和 log/rosbag path 时，才可以加入。

## Media Artifact 索引

| Artifact | Directory | 状态 | 说明 |
| --- | --- | --- | --- |
| RViz visual-perception screenshot | `screenshots/` | 未记录 | 只在 runtime capture 被审阅后加入。 |
| Gazebo Factory Patrol screenshot | `screenshots/` | 未记录 | 只在 runtime capture 被审阅后加入。 |
| Benchmark-derived figure | `figures/` | 未生成 | JSON/CSV 是当前定量证据。 |
| Demo video | external | 未记录 | 没有 command/commit/run 证据时不添加链接。 |

## Validation 总结

以下简短总结记录 build、test 和 static-check 证据，不声称实体机器人部署或真实工厂运行：

- [WSL2 full validation summary](wsl2_full_validation_summary.md)：历史 pre-perception
  WSL2 Ubuntu 24.04 + ROS2 Jazzy desktop 验证；package-level retry 处理一次瞬时 Nav2
  `/map` timeout 后，共 514 个测试。
- [Server Docker validation summary](server_docker_validation_summary.md)：Alibaba Cloud Linux 3
  + `ros:jazzy-ros-base` 部分验证；17 packages 完成、104 tests 通过。低内存 server 在编译
  `robot_tasks` 时终止 `cc1plus`，因此该 package 被排除。

视觉感知升级后的当前 full workspace baseline 为 21 packages、648 tests、0 errors、
0 failures、0 skipped。正式 perception/mission 测量记录在：

- [Benchmark JSON](../../src/robot_experiments/results/factory_patrol_phase8_20260815_011022.json)
- [Benchmark CSV](../../src/robot_experiments/results/factory_patrol_phase8_20260815_011022.csv)
- [实验与 Benchmark 报告](../experiment_report.md)

## Gazebo + RViz 仿真调试

默认 Factory Patrol simulation launch：

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true
```

可选 Scene V2 industrial layout preview：

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  world_file:=$(ros2 pkg prefix robot_simulation)/share/robot_simulation/worlds/factory_patrol_industrial.sdf \
  gui:=true use_rviz:=true
```

Headless launch：

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=false use_rviz:=false
```

Runtime topic check：

```bash
bash scripts/check_factory_patrol_runtime_topics.sh
```

默认 RViz config 为 `src/robot_simulation/rviz/factory_patrol_showcase.rviz`。它使用 `odom`
作为 fixed frame，重点显示 robot model、laser scan、odometry、odom path 和
`/amr_simulation/markers`，适合 non-Nav2 Demo 截图。需要更详细的 TF view 时，可使用
`src/robot_simulation/rviz/factory_patrol_debug.rviz`。

Showcase layout 包含 Factory Semantics marker layer，但默认关闭，避免红色 debug marker
overlay 影响画面。需要检查 semantic zone、reservation 或 state marker 时，可在 Displays
panel 中启用。

Gazebo world 使用 procedural lightweight asset：16 m x 12 m Factory floor、加宽 AMR aisle、
receiving buffer、后部 storage rack、packing workcell prop、dock guidance、safety rail、
landmark plate、低饱和 station sign、floor seam、scuff mark 和正交 inspection-route marking
均由 SDF primitive 建模，不依赖下载的第三方资产。

独立 `factory_patrol_industrial.sdf` Scene V2 preview 保持原 robot topic、frame 和 mission
seed pose，同时把视觉 Factory footprint 扩展到 24 m x 16 m。它增加更清晰的 receiving、
storage、packing、dock、slow-zone、guardrail 和 closed-loop inspection-route visual layer；
只有经过真实 WSL2 视觉审阅后才用于 portfolio screenshot。

world config 保留标准 Scene Manager、Camera Tracking 和 Interactive view control plugin，
使 Gazebo 可以渲染 3D scene 并提供 camera control service。截图前可手动折叠右侧 panel。

Live simulation 的 motion smoke-test command：

```bash
ros2 topic pub --rate 10 /virtual_rc/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 topic pub --once /virtual_rc/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

手动 input 使用 `/virtual_rc/cmd_vel`。`/teleop_cmd_vel` 是 `virtual_rc_node` 进入
`cmd_vel_mux_node` 的 output；`/cmd_vel` 是 bridge 到 Gazebo 的 final mux/safety output，
不属于普通手动输入接口。

如果 AMR 不移动，在修改 navigation、safety、chassis 或 task code 前，先检查 `/cmd_vel`、
`/odom`、`/safety_state`、`/cmd_vel_mux/active_source`、`ros2 topic info -v /cmd_vel` 和
`gz model --model mobile_robot --pose`。

预期 topic 包括 `/clock`、`/tf`、`/joint_states`、`/odom`、`/scan`、`/cmd_vel`、
`/mission_runner/state`、`/safety_state`、`/localization/health`、
`/amr_simulation/markers` 和 `/amr_simulation/demo_timeline`。

Windows 侧编辑完成后，应在 WSL2 Ubuntu 24.04 + ROS2 Jazzy 中完成最终视觉审阅：启动 Demo、
检查 runtime topic，并只在 scene 经过审阅后截图。

## Future Artifact 检查清单

每个未来 artifact 都需要记录：

- commit hash
- 启动运行所用 command
- map/world 与 parameter file
- 相关 log 或 rosbag path
- 运行属于 simulation、mock backend 还是实体 hardware

仅创建这些目录不代表 Showcase screenshot 和 video 已验证（not yet validated）。文档不会把
缺失的 image 当作 runtime proof。
