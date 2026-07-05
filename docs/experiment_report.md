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

## 2. Controller Comparison Experiment

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

## 3. Safety Experiment

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
