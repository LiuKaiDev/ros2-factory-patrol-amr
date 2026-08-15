# 实验与 Benchmark 报告

本报告区分完整软件基线、场景验收工具和正式 Perception/Mission Benchmark。所有定量结果
均标明运行环境；Gazebo/WSL 结果不代表实体机器人性能。

## 实验方法

正式指标来自可重复执行的 Gazebo/WSL 实验。统计结果以提交在
`src/robot_experiments/results/` 下的 JSON/CSV 原始产物为准，并记录配置、样本数量、clock
policy、排除项和源码状态。

失败样本不会被静默删除。实验前置条件失效、warmup 或 fixture settling 导致的排除均在
JSON 中记录原因和数量；保留进入统计窗口的 outlier。所有结果仅代表当前 Gazebo/WSL
仿真环境，不代表实体硬件标定、实时性或安全性能。

## WSL2 全量验证

| 项目 | 结果 |
| --- | --- |
| Platform | WSL2 |
| OS | Ubuntu 24.04 LTS |
| ROS distribution | Jazzy |
| ROS package set | `ros-jazzy-desktop`，项目依赖通过 `rosdep` 安装 |
| Build result | workspace build 在本地完成 |
| Latest full test result | 648 tests, 0 errors, 0 failures, 0 skipped |
| `robot_navigation` retry | 6 个测试全部通过 |
| RViz launch | WSL2 validation 通过 |
| Gazebo Factory Patrol launch | 添加轻量 label mesh asset 后通过 |

一次 full test run 曾在 `robot_navigation/test_nav2_mock_runtime.launch.py` 出现瞬时 `/map`
message timeout；单独重跑 `robot_navigation` 后通过，最终聚合的 `colcon test-result` 全部
通过。该现象更符合 WSL2/Nav2/DDS 启动时序波动，而非编译或语法错误。

Factory Patrol 验证中观察到 `/map`、`/scan`、`/odom`、`/cmd_vel`、`/tf`、
`/joint_states`、`/mission_runner/state`、`/safety_state`、`/localization/health`、
`/global_costmap/costmap`、`/local_costmap/costmap`、`/amr_simulation/markers` 和
`/amr_simulation/demo_timeline`。

## Factory Patrol Demo Acceptance

仓库提供三类 navigation-only 场景验收入口：

| Workflow | 入口 | 观察项 | 验收边界 |
| --- | --- | --- | --- |
| Multipoint patrol | `bash scripts/run_factory_patrol_multipoint_demo.sh` | `/navigate_sequence/current_goal`、`/mission_runner/state`、`/cmd_vel`、`/safety/state` | 按记录确认 waypoint 顺序和未处理故障。 |
| Temporary obstacle | `bash scripts/run_factory_patrol_obstacle_demo.sh` | `/scan`、local costmap、`/cmd_vel`、`/safety/state` | 按记录确认障碍表达和机器人响应。 |
| Localization recovery | `bash scripts/run_factory_patrol_localization_recovery_demo.sh` | `/localization/health`、`/safety/state`、`/amcl_pose`、`/tf` | 按记录确认 LOST/RECOVERED transition。 |

这些入口是可重复的验收工具，但未提交统一条件下的专项结果表。它们不与下述正式
Perception/Mission Benchmark 混合统计。

## Perception 与 Mission Benchmark

构建并 source workspace 后运行标准 headless profile：

```bash
bash scripts/run_factory_patrol_benchmarks.sh
```

运行器验证 asset 和 detector model，在 `src/robot_experiments/results/` 生成带时间戳的 JSON
和紧凑 CSV。任一 profile 的前置条件失败时，suite 停止且不会把不完整 profile 合并为结果。

默认 DDS domain base 为 `140`，每个 profile 使用递增 domain 和独立 Gazebo Transport
partition，避免 WSL 中残留进程污染 trial。可用 `FACTORY_PATROL_BENCHMARK_DOMAIN_ID` 和
`FACTORY_PATROL_BENCHMARK_GZ_PARTITION` 选择其他隔离范围。

### 指标定义

| Metric | Definition | Clock |
| --- | --- | --- |
| Detector processing | OpenCV-DNN `infer()` 调用时长 | detector 内部 monotonic wall duration |
| Detector message age | source image stamp 到 detection callback receipt | ROS simulation clock |
| 3D localization | near/medium/far center-ray fixture 中的欧氏 `P_est - P_gt` | observation data |
| Position stability | 共享 observation stamp 的 raw 与 TargetManager EMA 点的 population x/y/z standard deviation | observation data |
| Detection to action | 首次 person detection、confirmation、inspection event 和 navigation goal 的 benchmark receipt | ROS simulation clock |
| STOP response | safety condition header stamp 到首个有上游 Nav2 intent 且被 gate 为零的 `/cmd_vel` | ROS simulation clock |
| Mission success | detected、confirmed、event、goal、Nav2 success、completion event 和 `PROCESSED` | 每个 clean launch trial |
| Invalid depth | controlled invalid depth input，且同 stamp 不产生 valid map projection | ROS simulation clock |

百分位数使用 **nearest rank**：排序后选择 `ceil(percentile / 100 * count)`。标准差使用
population standard deviation。前五个 detector diagnostic sample 作为 warmup exclusion；
diagnostic sampling gap 和 fixture-settle exclusion 均在 JSON 中计数。

标准 CPU profile 采集 30 个 detector sample、三个距离各 10 个 geometry sample、5 个独立
reset 的 visual inspection mission、10 个 STOP transition 和 20 个 invalid-depth case。
每个 mission 重新启动 launch，确保 TargetManager、cooldown 和 task state 独立。Safety trial
保持 Nav2 goal 活动并交替使用 Gazebo fixture；mission 完成后保留 20 秒 observation window
统计迟到的重复 trigger。

Mission start 定义为 task node 的 `REQUESTED: observation pose planned` status。活动任务忽略
的额外 perception event 和 Nav2 retry 保留为 raw count，但不计为新 mission。GUI 和 RViz
关闭，以减少资源竞争。

### 结果产物与环境

固定产物：

- `src/robot_experiments/results/factory_patrol_phase8_20260815_011022.json`
- `src/robot_experiments/results/factory_patrol_phase8_20260815_011022.csv`

环境：ROS 2 Jazzy、`factory_patrol.sdf`、headless Gazebo、无 RViz、simulation time、CPU
OpenCV-DNN YOLOX-S、640x640 detector input、640x480 RGB-D（15 Hz）、confidence threshold
`0.45`。JSON 记录 commit `bfb59f9` 和 dirty-tree fingerprint，因为结果与对应实现变更一起
采集和提交。

JSON 还记录 branch、ROS distro、world、camera、detector backend/model/device、tracking 和
safety parameter、sample count、exclusion、failure category 以及 CameraInfo/TF diagnostic
fault。CSV 从 JSON 派生；临时 launch log 仅在设置
`KEEP_FACTORY_PATROL_BENCHMARK_LOGS=true` 时保留在 `/tmp`。

### WSL Simulation Result（2026-08-15）

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
`0.01612 m`；near fixture 在预定义的三样本 settle exclusion 后仍保留 RGB-D
render-transition sample，mean 为 `0.02879 m`，max 为 `0.07075 m`。共排除 9 个 settle
sample；没有删除进入统计窗口的 measured sample。

primary stationary target 的 `32` 组 paired observation 中，raw x/y/z standard deviation
为 `0.00161 / 0.00167 / 0.00540 m`，EMA 为 `0.00181 / 0.00197 / 0.00607 m`。EMA 三个轴
均略差，因此该次运行没有证明 stability improvement。连续 detector profile 中一个静态
物理目标出现两个 target ID，ID stability 并不完美。

五个 inspection trial 全部成功（`5/5`，success rate `1.0`），分类 failure 和 false mission
start 均为零。五个 intended trial 恰好启动五个 mission。活动 task 期间，新 ID 产生三个
额外 inspection-required event，均被 task node 忽略。Inspection event 到 completion 平均
`6.339 s`（P50 `6.364 s`，P95/max `6.682 s`）。该 Benchmark 没有采集最终 physical
standoff error。

STOP test 保持非零 Nav2 request，并要求最终 `/cmd_vel` 为零。单独的 speed-limit sample
把上游 `0.35 m/s` request 在 `0.226 s` 内限制为 `0.15 m/s`。Invalid-depth rejection 为
`20/20`，false-valid projection 为零。

三个 mission trial 中记录 13 个 non-OK diagnostic sample：
`perception/tf: observation-time TF intermittently unavailable`。全部恢复并成功；没有
CameraInfo fault sample。Detector warmup 排除 5 个 sample，记录 6 个 diagnostic sampling
gap，并在 Safety trial 前排除 1 个 Nav2-inactive setup goal。运行结束后没有残留对应的
Gazebo partition process。

### 结果解释

- Detector latency 在该环境中主要受 CPU OpenCV-DNN 限制。
- Geometry error 同时包含 RGB-D rendering 与 simulated odom/map alignment 影响。
- Action latency 包含 detector frame cadence 和 ROS event propagation。
- Safety latency 包含 perception propagation 与既有最终 Safety Gate。
- 早期 smoke observation 只用于链路调试，不汇总到本表。

## 已知限制

- 所有定量结果仅来自 WSL2/Gazebo，未覆盖实体硬件。
- EMA 在当前静态样本中没有改善稳定性。
- 一个静态目标在长时运行中出现两个 ID；当前没有 appearance ReID。
- Mission 负载下存在可恢复的瞬时 observation-time TF warning。
- Gazebo settling 与 wheel odometry origin 对齐会影响几何误差解释。
- Navigation-only workflow 具有验收入口，但没有统一条件下的专项结果表。
