# 实验与 Benchmark 报告

本文包含两个证据层级：尚未记录运行的旧验收模板保留 `TBD`；Phase 8 包含正式审阅过的
perception/mission benchmark。已提交的 Phase 8 JSON/CSV 对是定量结果的主要来源。不要
编造指标、截图或 pass/fail 结论。

## 环境

| 项目 | 值 |
| --- | --- |
| ROS distro | Jazzy |
| OS | Ubuntu 24.04 / WSL / other: TBD |
| Commit | TBD |
| Build command | `colcon build --symlink-install` |
| Map / world | TBD |
| Robot model | TBD |
| Nav2 parameter file | TBD |
| Date | TBD |
| Operator | TBD |

## 1. Nav2 导航验收

目的：验证 AMCL + Nav2 执行巡检目标的行为。

| Run | Map/world | Planner | Controller | success_rate | arrival_time | final_error | stop_count | result_log |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| TBD | TBD | Navfn | RPP | TBD | TBD | TBD | TBD | TBD |
| TBD | TBD | SmacPlanner2D | MPPI | TBD | TBD | TBD | TBD | TBD |

所需证据：

- Launch command 和 parameter file。
- RViz/Gazebo 截图或视频路径。
- Topic log 或 rosbag 路径。
- Commit hash 和 map 版本。

## 2. 独立 Tracking 实验

目的：使用记录的 CSV 文件比较 Pure Pursuit 和 Stanley 的 standalone tracking output。

运行辅助脚本：

```bash
bash scripts/run_tracking_experiment_demo.sh
```

分析命令：

```bash
python3 scripts/analyze_tracking_result.py <csv_file>
python3 scripts/compare_tracking_results.py --format markdown <pure_pursuit.csv> <stanley.csv>
python3 scripts/plot_tracking_result.py <csv_file> --output-dir src/robot_experiments/results/figures
```

| Run | controller | path_name | goal_success | sample_count | rms_lateral_error | max_lateral_error | mean_abs_heading_error | max_abs_heading_error | mean_linear_velocity | max_abs_angular_velocity | final_distance_to_goal | result_csv |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| TBD | pure_pursuit | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| TBD | stanley | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

生成图表：

| Figure | 状态 |
| --- | --- |
| trajectory | TBD |
| lateral error | TBD |
| heading error | TBD |
| cmd_vel | TBD |

## 3. Safety 验收

目的：验证 safety input 按预期影响最终 `/cmd_vel` 和 safety state topic。

| Case | Trigger | Expected behavior | Observed behavior | result_log | Notes |
| --- | --- | --- | --- | --- | --- |
| Emergency stop | `/enable_emergency_stop` | final `/cmd_vel` 变为零 | TBD | TBD | TBD |
| Cmd watchdog | mux input timeout | final `/cmd_vel` 变为零 | TBD | TBD | TBD |
| Dynamic speed limit | safety speed limit | velocity 被裁剪 | TBD | TBD | TBD |
| Localization lost | high AMCL covariance / TF timeout | safety state 改变且 command 被 gate | TBD | TBD | TBD |
| Chassis fault | heartbeat/fault code issue | command 被 gate 或报告 fault | TBD | TBD | TBD |

## 4. Factory Patrol Demo 验收（Factory Patrol Demo Acceptance）

以下 navigation-only Factory Patrol workflow 仍是验收模板，与下面记录的 visual-perception
benchmark 分开；在每个确切 workflow 被记录和审阅前保持 `TBD`。

| demo_name | scenario | launch_or_script | expected_topics | success_criteria | runtime_result | notes |
| --- | --- | --- | --- | --- | --- | --- |
| Factory Patrol Multipoint | start -> station_A -> station_B -> station_C -> dock | `bash scripts/run_factory_patrol_multipoint_demo.sh` | `/navigate_sequence/current_goal`, `/mission_runner/state`, `/cmd_vel`, `/safety/state` | waypoints 按顺序完成且无未处理 safety fault | TBD | TBD |
| Temporary Obstacle | obstacle on patrol route | `bash scripts/run_factory_patrol_obstacle_demo.sh` | `/scan`, `/local_costmap/costmap`, `/cmd_vel`, `/safety/state` | 障碍可见且记录机器人响应 | TBD | TBD |
| Localization Recovery | bad pose followed by recovery pose | `bash scripts/run_factory_patrol_localization_recovery_demo.sh` | `/localization/health`, `/safety/state`, `/amcl_pose`, `/tf` | 观察到 LOST 与 RECOVERED transition | TBD | TBD |

## 5. Showcase 证据

只有在真实运行产生后，才应把截图、视频和生成图表列到
[docs/showcase/README.md](showcase/README.md)。

| Artifact | Path | Source command | Status |
| --- | --- | --- | --- |
| RViz Nav2 debug screenshot | TBD | TBD | TBD |
| Gazebo factory patrol screenshot | TBD | TBD | TBD |
| Tracking comparison figure | TBD | TBD | TBD |
| Demo video | TBD | TBD | TBD |

## WSL2 全量验证

本节记录 WSL2 上实际完成的 ROS2 Jazzy 验证，不代表实体机器人部署或真实工厂运行。

| 项目 | 结果 |
| --- | --- |
| Platform | WSL2 |
| OS | Ubuntu 24.04 LTS |
| ROS distribution | Jazzy |
| ROS package set | `ros-jazzy-desktop`，项目依赖通过 `rosdep` 安装 |
| Build result | workspace build 在本地完成 |
| Latest full test result | 648 tests, 0 errors, 0 failures, 0 skipped |
| robot_navigation retry | 6 个测试全部通过，失败 0 个 |
| RViz launch | WSL2 validation 通过 |
| Gazebo Factory Patrol launch | 添加轻量 label mesh asset 后通过 |

此前一次 full test run 在 `robot_navigation/test_nav2_mock_runtime.launch.py` 出现过一次
瞬时 `/map` message timeout。单独重跑 `robot_navigation` 后通过，最终聚合的
`colcon test-result` 全部通过。可能原因是 WSL2/Nav2 runtime 或 DDS topic timing，而不是
语法级 build 问题。

Factory Patrol 仿真 validation 中观察到的 runtime topic 包括 `/map`、`/scan`、`/odom`、
`/cmd_vel`、`/tf`、`/joint_states`、`/mission_runner/state`、`/safety_state`、
`/localization/health`、`/global_costmap/costmap`、`/local_costmap/costmap`、
`/amr_simulation/markers` 和 `/amr_simulation/demo_timeline`。

## Phase 8：Perception 与 Mission 评估

Phase 8 为既有 perception、inspection、Nav2 和 Safety Gate chain 增加定量 probe，不改变
机器人行为，也不新增 perception algorithm。所有结果必须标注为 **Gazebo / WSL simulation
results**，不能当作实体机器人性能保证。

构建并 source workspace 后运行标准 headless profile：

```bash
bash scripts/run_factory_patrol_benchmarks.sh
```

命令会验证 asset 和 detector model，然后在 `src/robot_experiments/results/` 生成带时间戳的
JSON artifact 与紧凑性能表 CSV。前置条件或 profile 失败时 suite 停止，不把不完整 profile
合并为最终结果。

runner 默认 DDS domain base 为 `140`，每次 profile 启动递增 domain，避免 WSL 中残留的 ROS
进程和 Fast DDS participant 污染后续 trial；可用 `FACTORY_PATROL_BENCHMARK_DOMAIN_ID` 选择
其他未占用范围。每个 profile 还使用唯一 Gazebo Transport partition；仅在确实需要跨进程
Gazebo 通信时覆盖 `FACTORY_PATROL_BENCHMARK_GZ_PARTITION`。

### 方法

| Metric | Definition | Clock |
| --- | --- | --- |
| Detector processing | OpenCV-DNN `infer()` 调用时长 | detector 内部 monotonic wall duration |
| Detector message age | source image stamp 到 detection callback receipt | ROS simulation clock |
| 3D localization | near/medium/far center-ray fixture 中的欧氏 `P_est - P_gt` | observation data |
| Position stability | 共享 observation stamp 的 raw 与 TargetManager EMA 点的 population x/y/z standard deviation | observation data |
| Detection to action | 首次 person detection、confirmation、inspection event 和 navigation goal 的 benchmark receipt | ROS simulation clock |
| STOP response | perception safety condition header stamp 到第一个有上游 Nav2 intent 且被 gate 为零的 `/cmd_vel` | ROS simulation clock |
| Mission success | detected、confirmed、event、goal、Nav2 success、completion event 和 `PROCESSED` | 每个 clean launch trial |
| Invalid depth | controlled invalid depth input，且同 stamp 不产生 valid map projection | ROS simulation clock |

百分位数使用 **nearest rank**：排序样本后选择 `ceil(percentile / 100 * count)`。标准差为
population standard deviation。保留 outlier。前五个 detector diagnostic sample 明确作为
warmup exclusion 记录；diagnostic sampling gap 和 fixture-settle exclusion 在 JSON 中计数，
不会静默丢弃。

标准 WSL CPU profile 在 warmup 后采集 30 个 detector sample、三个距离各 10 个 geometry
sample、5 个独立 reset 的 visual inspection mission、10 个 STOP transition 和 20 个
invalid-depth case。每个 mission 都重启 launch，使 TargetManager、cooldown 和 task state
干净开始。Safety trial 在保持 Nav2 goal 活动的同时交替使用既有 Gazebo fixture。完成后保留
20 秒 observation window，用于计数迟到的 duplicate task trigger，而不是第一个 mission 成功
后立即停止。Mission start 定义为 task node 的 `REQUESTED: observation pose planned` status。
active task 忽略的额外 perception event 和 Nav2 retry 作为 raw count 保留，但不误记为新的
mission start。GUI 和 RViz 保持关闭，以减少而不是掩盖资源竞争。

### 结果策略

JSON 记录 git revision 与 dirty-state fingerprint、branch、ROS distro、world、camera、
detector backend/model/device、tracking 和 safety parameter、sample count、exclusion、
failure category，以及观察到的 CameraInfo/TF diagnostic fault。CSV 从 JSON 派生。只提交
小规模审阅过的 JSON/CSV 对；临时 launch log 仅在设置
`KEEP_FACTORY_PATROL_BENCHMARK_LOGS=true` 时保留在 `/tmp`。

### WSL Simulation Result（2026-08-15）

固定产物：

- `src/robot_experiments/results/factory_patrol_phase8_20260815_011022.json`
- `src/robot_experiments/results/factory_patrol_phase8_20260815_011022.csv`

环境：ROS 2 Jazzy、`factory_patrol.sdf`、headless Gazebo、无 RViz、simulation time、实际
CPU device 上的 OpenCV YOLOX、640x640 detector input、640x480 RGB-D（15 Hz）、confidence
threshold `0.45`。JSON 记录 commit `bfb59f9` 和 dirty-tree fingerprint，因为该结果随 Phase 8
实现一并提交。

| Metric | Count | Mean | P50 | P95 | Max | Unit |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Detector processing | 30 | 526.189 | 519.714 | 568.830 | 656.778 | ms |
| 3D localization error | 30 | 0.02045 | 0.01645 | 0.05239 | 0.07075 | m |
| Detection to confirmation | 5 | 1.970 | 2.023 | 2.405 | 2.405 | s |
| Confirmation to inspection event | 5 | 0.000 | 0.000 | 0.000 | 0.000 | s |
| Inspection event to Nav2 goal | 5 | 0.080 | 0.002 | 0.385 | 0.385 | s |
| Detection to Nav2 goal | 5 | 2.050 | 2.034 | 2.790 | 2.790 | s |
| Safety STOP response | 10 | 0.1806 | 0.173 | 0.214 | 0.214 | s |

Localization RMSE 为 `0.02351 m`。medium 和 far fixture mean error 分别为 `0.01645 m` 和
`0.01612 m`；near fixture 在预定义的三样本 settle exclusion 后仍保留 RGB-D render-transition
sample，mean 为 `0.02879 m`，max 为 `0.07075 m`。策略排除了 9 个 settle sample；检查后没有
删除任何 measured sample。

primary stationary target 的 `32` 组 paired observation 中，raw x/y/z standard deviation 为
`0.00161 / 0.00167 / 0.00540 m`，EMA 为 `0.00181 / 0.00197 / 0.00607 m`。本轮 EMA 三个轴
均略差，因此不宣称 stability improvement。连续 detector profile 中观察到两个 target ID，
ID stability 并不完美。

五个端到端 inspection trial 全部成功（`5/5`，success rate `1.0`），分类 failure 与 false
mission start 均为零。五个 intended trial 恰好启动五个 mission。活动 task 期间，新分配
target ID 产生了三个额外 inspection-required event，被 task node 忽略，未启动 mission。
Inspection event 到 completion 平均 `6.339 s`（P50 `6.364 s`，P95/max `6.682 s`）。本
benchmark 没有采集最终 physical standoff error，因此不声称该值。

重复 STOP test 保持非零 Nav2 request，并要求最终 `/cmd_vel` 为零。单独的 speed-limit sample
把上游 `0.35 m/s` request 在 `0.226 s` 内限制为 `0.15 m/s`。Invalid-depth rejection 为
`20/20`，false-valid projection 为零。

三个 mission trial 中记录了 13 个 non-OK diagnostic sample：
`perception/tf: observation-time TF intermittently unavailable`。全部恢复并成功；没有观察到
CameraInfo fault sample。Detector warmup 排除 5 个记录 sample，计数 6 个 detector diagnostic
sampling gap，并在 Safety trial 开始前排除 1 个 Nav2-inactive setup goal。runner 用 DDS domain
和 Gazebo partition 隔离每个 profile，最终没有残留 Phase 8 partition process。

解释：在该 WSL/OpenCV-DNN 设置下 detector time 受 CPU 限制；geometry 包含 RGB-D rendering
和 simulated odom/map alignment；action latency 包含 detector frame cadence 与 ROS event
propagation；safety time 包含 perception propagation 和既有 final Safety Gate。历史 Phase
2/3/6 smoke observation 仅作背景，不汇总为 aggregate sample。

## 结果记录规则

- 只有在运行真实且可重复时才移除 `TBD`。
- 每个表格行记录确切 commit、command、map、parameters 和 logs。
- 失败运行也是有效数据，只要记录 trigger、observed behavior 和 next action。
- Generated CSV、PNG、SVG、PDF 和 video 默认不提交。
