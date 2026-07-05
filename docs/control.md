# Control

本项目涉及两类控制链路：Nav2 controller 下的局部控制，以及 standalone path tracking experiment 下的路径跟踪控制器。两者用途不同，文档和实验报告中需要分开描述。

## Nav2 Controller

| Controller | Location | Status | Role |
| --- | --- | --- | --- |
| Regulated Pure Pursuit (RPP) | `nav2_basic.yaml` | current | Nav2 局部控制器，根据全局路径输出速度候选。 |
| MPPI | `nav2_advanced.yaml` | current | Nav2 高级局部控制器，用采样优化方式输出速度候选。 |

Nav2 controller 的输出不是直接发到底盘，而是进入 cmd_vel 仲裁与 safety gate：

```text
Nav2 controller -> /nav2_cmd_vel or mux input -> cmd_vel_mux / twist_mux -> cmd_vel_safety_gate -> /cmd_vel
```

## Standalone Path Tracking Experiment

| Controller | Package | Status | Role |
| --- | --- | --- | --- |
| Pure Pursuit | `robot_path_tracking` | current + Phase 2A CSV metrics | 独立路径跟踪实验，用于参考路径追踪与控制器对比。 |
| Stanley | `robot_path_tracking` | current + Phase 2A CSV metrics | 独立路径跟踪实验，用于横向误差相关控制对比。 |

该链路适合做算法讲解和指标对比，不等同于 Nav2 controller_server 的插件实现。

数据流：

```text
/reference_path + /odom -> pure_pursuit_controller_node / stanley_controller_node
  -> /tracking_cmd_vel
  -> /tracking_error
  -> optional tracking CSV
```

`path_publisher_node` 发布 `/reference_path`，两个 standalone controller 订阅 `/odom` 和 `/reference_path`，输出 `/tracking_cmd_vel`，并发布统一字段的 `/tracking_error`。Phase 2A 增加可选 CSV logging，默认关闭，不会在普通运行中自动生成结果文件。

## Phase 2A CSV Metrics

Pure Pursuit 和 Stanley 支持相同参数：

| Parameter | Default | Meaning |
| --- | --- | --- |
| `enable_csv_logging` | `false` | 只有显式开启时才写 CSV。 |
| `csv_output_path` | `""` | 为空时默认写入 `src/robot_experiments/results/tracking_<controller>_<timestamp>.csv`。 |
| `path_name` | `default_path` | 写入 CSV 的路径名称标签。 |

CSV 字段：

| Field | Meaning |
| --- | --- |
| `timestamp_sec` | 当前 ROS 时间秒。 |
| `controller` | `pure_pursuit` 或 `stanley`。 |
| `path_name` | 运行参数中的路径名称。 |
| `sample_index` | 控制器本次运行内的采样序号。 |
| `x`, `y`, `yaw` | 当前机器人位姿。 |
| `ref_x`, `ref_y`, `ref_yaw` | 最近参考路径点位姿；`ref_yaw` 来自路径点 orientation。 |
| `linear_velocity`, `angular_velocity` | 本 tick 输出的速度命令。 |
| `lateral_error` | 机器人相对最近参考点切线方向的带符号横向误差，单位 m。 |
| `heading_error` | `ref_yaw - yaw` 归一化到 `[-pi, pi]`，单位 rad。 |
| `distance_to_goal` | 当前位姿到路径终点的距离。 |
| `goal_reached` | 是否到达终点容差。 |

开启 CSV logging 示例：

```bash
ros2 launch robot_bringup tracking.launch.py \
  controller:=pure_pursuit \
  use_mock_chassis:=true \
  enable_csv_logging:=true \
  csv_output_path:=src/robot_experiments/results/tracking_pure_pursuit_demo.csv \
  path_name:=demo_path
```

Stanley 只需要把 `controller:=stanley`。不要提交运行生成的 CSV 结果。

分析 CSV：

```bash
python3 scripts/analyze_tracking_result.py <csv_file>
```

提示型运行脚本：

```bash
bash scripts/run_tracking_experiment_demo.sh pure_pursuit
bash scripts/run_tracking_experiment_demo.sh stanley
```

该脚本检查 `ros2` 环境并打印推荐 launch / analyze 命令，不会自动修改文件，也不会自动提交生成的 CSV。

分析脚本只使用 Python 标准库，输出：

- `controller`
- `path_name`
- `sample_count`
- `goal_reached`
- `mean_lateral_error`
- `rms_lateral_error`
- `max_lateral_error`
- `mean_abs_heading_error`
- `max_abs_heading_error`
- `mean_linear_velocity`
- `max_linear_velocity`
- `max_abs_angular_velocity`
- `final_distance_to_goal`

## Metric Meaning

Phase 2A 只建立基础指标输出，不声称任何控制器优劣。

| Metric | Meaning | Status |
| --- | --- | --- |
| `lateral_error` | 横向误差时间序列 | current CSV field |
| `heading_error` | 航向误差时间序列 | current CSV field |
| `rms_lateral_error` | 横向误差 RMS | current analysis script |
| `max_lateral_error` | 最大横向误差绝对值 | current analysis script |
| `mean_abs_heading_error` | 平均航向误差绝对值 | current analysis script |
| `trajectory_compare` | 参考轨迹与实际轨迹对比 | planned Phase 2B |
| `cmd_vel_curve` | 线速度 / 角速度曲线 | planned Phase 2B |
| `RPP / MPPI comparison` | Nav2 controller 对比 | planned Phase 2B 或后续 |

## Experiment Boundary

- 可以说明项目中存在 Pure Pursuit / Stanley / RPP / MPPI 配置和代码。
- 可以说明 Pure Pursuit / Stanley 已有 standalone CSV 指标输出基础。
- 图表绘制、轨迹对比、RPP / MPPI 全量对比留到 Phase 2B 或后续。
- 不应在没有重新跑实验的情况下声称某控制器更优、收敛更快或误差更小。
