# 控制系统

项目包含两类用途不同的控制链路：Nav2 controller 负责正式导航中的局部控制；
`robot_path_tracking` 提供 Pure Pursuit 和 Stanley 的独立路径跟踪实验。两者不共享实现，
实验控制器也不替代 Nav2。

## 最终控制架构

```text
Nav2 controller -> /nav2_cmd_vel --+
teleop / virtual RC ---------------+-> cmd_vel mux -> Safety Gate -> /cmd_vel
standalone tracking ---------------+
```

所有候选命令都经过速度源仲裁和 Safety Gate。Perception 不发布 `/cmd_vel` 或
`/nav2_cmd_vel`。

## Nav2 Controller

| Controller | 配置 | 作用 |
| --- | --- | --- |
| Regulated Pure Pursuit (RPP) | `src/robot_navigation/config/nav2_basic.yaml` | basic profile 的低速局部路径跟随。 |
| MPPI | `src/robot_navigation/config/nav2_advanced.yaml` | advanced profile 的采样优化局部控制。 |

RPP 和 MPPI 都由 `controller_server` 调用并生成 Nav2 速度候选。当前参数见
[导航系统](navigation.md)。仓库没有提交统一条件下的 RPP/MPPI 定量对比，因此不据此判断
控制器优劣。

## 独立 Path Tracking 实验

| Controller | Package | 作用 |
| --- | --- | --- |
| Pure Pursuit | `robot_path_tracking` | 参考路径追踪和前视控制实验。 |
| Stanley | `robot_path_tracking` | 基于横向与航向误差的控制实验。 |

```text
/reference_path + /odom
  -> pure_pursuit_controller_node / stanley_controller_node
  -> /tracking_cmd_vel + /tracking_error
  -> optional tracking CSV
```

`path_publisher_node` 发布 `/reference_path`；两个控制器输出相同字段的 `/tracking_error`。
CSV logging 默认关闭，不会在普通运行中自动生成结果。

## Tracking CSV 与指标

| Parameter | Default | 含义 |
| --- | --- | --- |
| `enable_csv_logging` | `false` | 显式开启后才写 CSV。 |
| `csv_output_path` | `""` | 为空时写入 `src/robot_experiments/results/tracking_<controller>_<timestamp>.csv`。 |
| `path_name` | `default_path` | 当前参考路径标签。 |

CSV 字段包括：

| Field | 含义 |
| --- | --- |
| `timestamp_sec`, `sample_index` | ROS 时间与采样序号。 |
| `controller`, `path_name` | 控制器和路径标签。 |
| `x`, `y`, `yaw` | 当前机器人位姿。 |
| `ref_x`, `ref_y`, `ref_yaw` | 最近参考点位姿。 |
| `linear_velocity`, `angular_velocity` | 当前速度候选。 |
| `lateral_error` | 相对参考点切线的带符号横向误差，单位 m。 |
| `heading_error` | 归一化到 `[-pi, pi]` 的航向误差，单位 rad。 |
| `distance_to_goal`, `goal_reached` | 终点距离和到达标志。 |

示例：

```bash
ros2 launch robot_bringup tracking.launch.py \
  controller:=pure_pursuit \
  use_mock_chassis:=true \
  enable_csv_logging:=true \
  csv_output_path:=src/robot_experiments/results/tracking_pure_pursuit_demo.csv \
  path_name:=demo_path
```

使用 Stanley 时改为 `controller:=stanley`。运行生成的 CSV 默认被忽略，只有附带可追溯
配置并经过审阅的结果才适合作为项目证据。

## 分析与可视化

分析单个结果：

```bash
python3 scripts/analyze_tracking_result.py <csv_file>
```

输出包括 sample count、goal status、横向误差 RMS/最大值、航向误差均值/最大值、速度统计
和最终 goal distance。

生成图表：

```bash
python3 scripts/plot_tracking_result.py <csv_file> \
  --output-dir src/robot_experiments/results/figures
```

| Figure | 含义 |
| --- | --- |
| `trajectory.png` | 参考路径与实际轨迹。 |
| `lateral_error.png` | 横向误差随采样变化。 |
| `heading_error.png` | 航向误差随采样变化。 |
| `cmd_vel.png` | 线速度和角速度候选曲线。 |

比较两个结果：

```bash
python3 scripts/compare_tracking_results.py <pure_pursuit.csv> <stanley.csv>
python3 scripts/compare_tracking_results.py --format markdown \
  <pure_pursuit.csv> <stanley.csv>
```

完整分析 workflow：

```bash
bash scripts/run_tracking_analysis_workflow.sh <tracking.csv>
bash scripts/run_tracking_analysis_workflow.sh <pure_pursuit.csv> <stanley.csv>
```

这些脚本不依赖 ROS2；绘图需要 `matplotlib`。

## 指标解释

| Metric | 解读 |
| --- | --- |
| `rms_lateral_error` | 全程横向偏差的整体量级。 |
| `max_lateral_error` | 局部最大偏差。 |
| `mean_abs_heading_error` | 车身朝向与参考方向的平均差异。 |
| `max_abs_heading_error` | 急弯、起步或末端的最大朝向误差。 |
| `max_abs_angular_velocity` | 控制输出的最大转向强度。 |
| `final_distance_to_goal` | 最后一帧到终点的距离，不能单独代表全程质量。 |

公平比较需要使用相同路径、初始位姿、odom source、速度与角速度上限、goal tolerance、采样
条件和运行窗口，并记录 commit、launch 参数与 CSV 路径。

## 验证边界

- Pure Pursuit/Stanley 已具备统一 CSV、分析、绘图和 Markdown 对比工具。
- RPP/MPPI 是当前 Nav2 配置中的正式控制器，但尚无统一专项对比产物。
- 仿真 tracking 结果不等于实体底盘控制性能；未附带真实 CSV 的比较不构成实验结论。
- 未来的 RPP/MPPI 调参与硬件验证统一列在 [项目路线图](roadmap.md)。
