# 验证脚本

脚本按用途分组。Static check 不需要运行中的 ROS2 graph；runtime check 要求对应的
ROS2 topic 已经存在。

## Nav2

```bash
bash scripts/run_nav2_basic_demo.sh
bash scripts/check_nav2_costmap_obstacle_layer.sh
bash scripts/check_nav2_runtime_topics.sh
```

`run_nav2_basic_demo.sh` 输出 basic Nav2 和 RViz debug 的启动提示；costmap 脚本检查
`/scan` obstacle layer 等静态配置；runtime 脚本检查 `/scan`、TF、odom、map、path、
costmap 和 cmd_vel topic。

## Tracking 实验

```bash
bash scripts/run_tracking_experiment_demo.sh
bash scripts/run_tracking_analysis_workflow.sh <tracking.csv>
python3 scripts/analyze_tracking_result.py <tracking.csv>
python3 scripts/compare_tracking_results.py --format markdown <a.csv> <b.csv>
python3 scripts/plot_tracking_result.py <tracking.csv> --output-dir src/robot_experiments/results/figures
```

这些脚本对真实运行生成的 tracking CSV 做统计、绘图和 Pure Pursuit/Stanley 对比，
不会凭空生成实验数据。

## 底盘与标定

```bash
bash scripts/check_chassis_protocol_v2.sh
bash scripts/check_chassis_odom_calibration.sh
```

前者检查 protocol v2 的结构、driver hook 和单测锚点；后者检查底盘 odom 参数与
calibration 文档入口。二者都不代表真实串口/UDP 或实体底盘验收。

## 定位与安全

```bash
bash scripts/check_localization_health.sh
bash scripts/check_localization_runtime_topics.sh
bash scripts/check_safety_state_machine.sh
bash scripts/check_safety_runtime_topics.sh
```

静态脚本检查源码和配置；runtime 脚本只检查真实 graph 中的 topic、TF 和状态入口。

## Factory Patrol

```bash
bash scripts/run_factory_patrol_demo.sh
bash scripts/check_factory_patrol_assets.sh
bash scripts/run_factory_patrol_multipoint_demo.sh
python3 scripts/print_factory_patrol_goals.py
bash scripts/run_factory_patrol_obstacle_demo.sh
bash scripts/run_factory_patrol_localization_recovery_demo.sh
bash scripts/check_factory_patrol_demo_workflows.sh
bash scripts/check_factory_patrol_runtime_topics.sh
bash scripts/prepare_phase3_detector_model.sh
bash scripts/check_factory_patrol_detector_runtime.sh
bash scripts/check_factory_patrol_target_manager_runtime.sh
bash scripts/check_factory_patrol_visual_inspection_runtime.sh
bash scripts/check_factory_patrol_perception_safety_runtime.sh
bash scripts/check_factory_patrol_perception_diagnostics_runtime.sh
bash scripts/run_factory_patrol_benchmarks.sh
bash scripts/check_factory_patrol_demo_runtime.sh
```

`run_factory_patrol_demo.sh` 在 non-Nav2 Gazebo/RViz showcase 中使用默认
`src/robot_simulation/rviz/factory_patrol_showcase.rviz`。`check_factory_patrol_runtime_topics.sh`
要求 ROS2 graph 正在运行，并输出 topic 数量、`/scan` QoS 提示、采样 frame ID 和 odom TF
连通性诊断。

`prepare_phase3_detector_model.sh` 会显式下载并校验官方 OpenCV Zoo YOLOX-S 模型到用户
cache；普通 ROS launch 不会下载权重。Detector-mode Demo 运行时，
`check_factory_patrol_detector_runtime.sh` 检查 2D 到 3D 的真实链路。

`check_factory_patrol_target_manager_runtime.sh` 复用 live detector、Depth、CameraInfo
和 TF graph，验证 stable ID、lifecycle transition、duplicate suppression、marker 以及
raw/filter position statistics。

加载 Phase 5 validation profile 后，`check_factory_patrol_visual_inspection_runtime.sh`
验证一次 task-owned Nav2 approach、Observation standoff 和 yaw、机器人运动、完成反馈、
target 的 `PROCESSED` 状态，以及不变的 mux/Safety Gate velocity path。

`run_factory_patrol_demo.sh --phase6` 启动 live detector、managed person safety policy、
Nav2 和既有 combined mux/Safety Gate。对应 runtime check 让 person fixture 经过距离与
map-zone 场景，比较真实 `/nav2_cmd_vel` intent 与最终 `/cmd_vel`，测量 STOP 响应并验证
恢复；Perception 不发送任何 Twist。

`run_factory_patrol_demo.sh --phase7` 增加标准 perception diagnostics、system health 和
fault supervision。Phase 7 check 注入 camera、depth-quality、observation-time TF 和
detector fault，并验证恢复，不发布 velocity 或 synthetic safety event。

`run_factory_patrol_benchmarks.sh` 执行隔离的 headless Phase 8 detector、geometry、mission、
safety 和 invalid-depth profile。成功后在 `src/robot_experiments/results/` 写入带时间戳的
JSON 和 summary CSV。已审核的结果对记录在 `docs/experiment_report.md`；运行时间会随主机
负载变化。

预览独立的 Factory Patrol Scene V2 industrial world：

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  world_file:=$(ros2 pkg prefix robot_simulation)/share/robot_simulation/worlds/factory_patrol_industrial.sdf \
  gui:=true use_rviz:=true
```

手动 motion smoke test 应发布到 `/virtual_rc/cmd_vel`，不要直接发布到 `/cmd_vel`：

```bash
ros2 topic pub --rate 10 /virtual_rc/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 topic pub --once /virtual_rc/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

`/teleop_cmd_vel` 是 `virtual_rc_node` 进入 mux 的输出，`/cmd_vel` 是 Gazebo bridge
消费的最终 mux/safety 输出。常用检查项包括 `/cmd_vel`、`/odom`、`/safety_state`、
`/cmd_vel_mux/active_source` 和 `gz model --model mobile_robot --pose`。

## 最终就绪检查

```bash
bash scripts/check_project_showcase_readiness.sh
```

该脚本检查 Phase 9 landing 文档、local Markdown path、提交的 Phase 8 JSON/CSV 一致性、
CI、evidence policy 和主要 validation 入口；它不能替代 ROS runtime check。
