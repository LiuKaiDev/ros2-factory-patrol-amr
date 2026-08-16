# Factory Patrol 演示手册

本文档说明当前 Factory Patrol 仿真入口、可观察状态和 Benchmark 复现方式。所有运行结果
仍以 [实验与 Benchmark 报告](experiment_report.md) 和提交的 JSON/CSV 产物为准。

## 启动仿真

前置条件：Ubuntu 24.04、ROS 2 Jazzy、已安装依赖并完成 workspace build。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/run_factory_patrol_demo.sh
```

启动提示会列出 world、站点、区域、路线和 RViz 配置。实际启动：

```bash
bash scripts/run_factory_patrol_demo.sh --launch
```

也可以直接启动 launch：

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  gui:=true use_rviz:=true
```

工业布局预览使用 `factory_patrol_industrial.sdf`：

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  world_file:=$(ros2 pkg prefix robot_simulation)/share/robot_simulation/worlds/factory_patrol_industrial.sdf \
  gui:=true use_rviz:=true
```

## 视觉巡检

准备本地 Detector 模型，然后启动视觉巡检任务：

```bash
bash scripts/prepare_detector_model.sh
bash scripts/run_factory_patrol_demo.sh --visual-inspection
```

观察：

- `/perception/detections_2d`
- `/perception/objects_3d`
- `/perception/events`
- `/inspection/observation_pose`
- `/inspection/status`
- `/navigate_sequence/current_goal`
- `/nav2_cmd_vel` 和最终 `/cmd_vel`

确认链路为 `CONFIRMED target -> INSPECTION_REQUIRED -> Observation Pose -> Nav2 ->
INSPECTION_COMPLETED -> PROCESSED`。Perception 不发布任何速度命令。

## 人员安全

```bash
bash scripts/run_factory_patrol_demo.sh --perception-safety
bash scripts/check_factory_patrol_perception_safety_runtime.sh
```

观察 `/perception/safety_event`、`/safety/state`、`/safety/reason`、`/nav2_cmd_vel` 和
`/cmd_vel`。安全策略由 Safety Gate 按最高优先级解析，阈值和 hysteresis 见
[Safety 状态机](safety_state_machine.md)。

## 感知诊断

```bash
bash scripts/run_factory_patrol_demo.sh --perception-diagnostics
bash scripts/check_factory_patrol_perception_diagnostics_runtime.sh
```

诊断 topic 为 `/perception/diagnostics`，状态覆盖 RGB、Depth、CameraInfo、Detector、TF、
depth quality 和 pipeline，并沿 `system_monitor -> fault_supervisor -> Safety Gate` 传播。

## 场景验收

```bash
bash scripts/run_factory_patrol_multipoint_demo.sh
bash scripts/run_factory_patrol_obstacle_demo.sh
bash scripts/run_factory_patrol_localization_recovery_demo.sh
bash scripts/check_factory_patrol_demo_runtime.sh
```

常用观察 topic：`/scan`、`/local_costmap/costmap`、`/localization/health`、`/safety/state`、
`/mission_runner/state`、`/odom`、`/tf` 和 `/cmd_vel`。这些入口是可重复的检查工具，未统一
记录的专项结果不会被当作 Benchmark 指标。

## Benchmark

```bash
bash scripts/run_factory_patrol_benchmarks.sh
```

运行器会验证 ROS2、workspace、Factory Patrol 资产、Detector 模型和 Benchmark 配置，然后
按隔离 DDS/Gazebo profile 生成 JSON/CSV。运行时间受 CPU 与 WSL 负载影响；固定已审阅产物
和全部限制见 [实验与 Benchmark 报告](experiment_report.md)。
