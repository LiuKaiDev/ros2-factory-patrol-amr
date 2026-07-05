# Experiment Report Template

本文件是实验报告模板。Phase 0 不新增、不伪造任何实验数据。所有表格中的 TBD 必须在真实运行、保存日志和复核后替换。

## Environment

| Item | Value |
| --- | --- |
| ROS distro | Jazzy |
| Build command | `colcon build --symlink-install` |
| Map / world | TBD |
| Robot model | TBD |
| Controller config | TBD |
| Date | TBD |
| Commit | TBD |

## 1. Navigation Experiment

Purpose: 验证 AMCL + Nav2 在巡检点任务中的导航能力。

| Run | Planner | Controller | success_rate | arrival_time | final_error | stop_count | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| TBD | Navfn | RPP | TBD | TBD | TBD | TBD | TBD |
| TBD | SmacPlanner2D | MPPI | TBD | TBD | TBD | TBD | TBD |

Required artifacts:

- rosbag or topic logs: TBD
- RViz / Gazebo screenshot: TBD
- map and params version: TBD

## 2. Standalone Tracking Experiment

Purpose: 记录 Pure Pursuit / Stanley standalone 路径跟踪基础指标。Phase 2A 只提供 CSV logging 和分析脚本，表格结果必须由真实运行后的 CSV 填写。

| Run | controller | path_name | goal_success | sample_count | rms_lateral_error | max_lateral_error | mean_abs_heading_error | max_abs_heading_error | mean_linear_velocity | max_abs_angular_velocity | final_distance_to_goal | result_csv |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| TBD | pure_pursuit | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| TBD | stanley | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

Analysis command:

```bash
python3 scripts/analyze_tracking_result.py <csv_file>
```

Plot command:

```bash
python3 scripts/plot_tracking_result.py <csv_file> --output-dir src/robot_experiments/results/figures
```

Comparison command:

```bash
python3 scripts/compare_tracking_results.py --format markdown <pure_pursuit.csv> <stanley.csv>
```

Controller comparison table:

| controller | path_name | goal_success | sample_count | rms_lateral_error | max_lateral_error | mean_abs_heading_error | max_abs_heading_error | mean_linear_velocity | max_abs_angular_velocity | final_distance_to_goal | result_csv | figures |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| pure_pursuit | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| stanley | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

Figure references:

| Figure | Status |
| --- | --- |
| `figures/trajectory.png` | TBD |
| `figures/lateral_error.png` | TBD |
| `figures/heading_error.png` | TBD |
| `figures/cmd_vel.png` | TBD |

Required artifacts:

- tracking CSV: TBD
- generated figures directory: TBD
- launch command and parameters: TBD
- map / path file version: TBD
- commit: TBD

## 3. Controller Comparison Experiment

Purpose: 对比 standalone Pure Pursuit / Stanley 或 Nav2 RPP / MPPI 的轨迹跟踪表现。

| Run | Controller | rms_lateral_error | max_lateral_error | mean_heading_error | max_angular_velocity | stop_count | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| TBD | Pure Pursuit | TBD | TBD | TBD | TBD | TBD | TBD |
| TBD | Stanley | TBD | TBD | TBD | TBD | TBD | TBD |
| TBD | RPP | TBD | TBD | TBD | TBD | TBD | TBD |
| TBD | MPPI | TBD | TBD | TBD | TBD | TBD | TBD |

Required plots:

- trajectory compare: TBD
- lateral error curve: TBD
- heading error curve: TBD
- cmd_vel curve: TBD

Do not fill plots or tables with generated-looking placeholders. Only paste metrics produced from real CSV files and keep the corresponding CSV / figure paths in the report.

## 4. Safety Experiment

Purpose: 验证安全门控、急停、通信超时、定位丢失和故障监督。

| Case | Trigger | Expected behavior | Observed behavior | success_rate | stop_count | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| Emergency stop | `/enable_emergency_stop` | `/cmd_vel` becomes zero | TBD | TBD | TBD | TBD |
| Cmd watchdog | mux input timeout | `/cmd_vel` becomes zero | TBD | TBD | TBD | TBD |
| Dynamic speed limit | `/safety_state` speed limit | output velocity clipped | TBD | TBD | TBD | TBD |
| Localization lost | high AMCL covariance | mission pauses / recovery requested | TBD | TBD | TBD | TBD |
| Chassis communication lost | heartbeat timeout | zero velocity / fault state | TBD | TBD | TBD | planned |

## Result Policy

- 不填写未运行的数据。
- 不用单次成功截图替代完整实验记录。
- 所有数值指标需标明 commit、地图、参数文件、启动命令和日志路径。
- 对失败实验也保留记录，注明失败原因和下一步处理。
