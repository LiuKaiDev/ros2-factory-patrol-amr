# 工程项目总结

## 项目定位

**ROS2 Factory Patrol AMR with Visual Perception, Navigation and Safety Integration**

本项目在既有 ROS 2 Jazzy 巡检机器人上增加模块化 RGB-D perception loop，同时保留 Nav2、
AMCL/localization、velocity arbitration 和最终 Safety Gate。当前验证平台是 WSL2、
Ubuntu 24.04 与 Gazebo simulation，不是实体机器人部署。

## 要解决的问题

原始 AMR 可以执行规划巡检/导航目标、仲裁速度源，并通过 Safety Gate 停车或限速；但不能
把相机观察转换为稳定的 map-frame target、task-owned inspection goal 或语义化人员限制。

升级必须在不创建平行 navigation stack、不允许 perception 控制运动、也不把系统耦合到单一
detector model 的前提下补齐这些能力。

## 已实现架构

```text
RGB-D -> Detection2D -> robust depth -> optical-frame 3D
      -> observation-time TF2 -> map-frame TargetManager
                                  /                 \
                     inspection event          safety event
                            |                       |
                       robot_tasks              Safety Gate
                            |                       |
                          Nav2 -> mux --------------+
                                  |
                             final /cmd_vel
```

关键 ownership boundary：

- `robot_perception` 产生 detection、geometry、managed target、semantic mission/safety event、
  marker 和 diagnostics。
- `robot_tasks` 负责 Observation Pose 规划、action lifecycle，以及既有
  `/navigate_sequence` 到 Nav2 的路径。
- Nav2 仍负责 planning 和 control，把 `/nav2_cmd_vel` 写入既有 mux。
- Safety Gate 仍是解析所有 safety source 并输出最终 `/cmd_vel` 的唯一组件。
- Perception 既不发布 `/cmd_vel`，也不发布 `/nav2_cmd_vel`。

## 工程工作内容

### Sensor 与 Geometry 集成

- 在 Xacro 中加入唯一 camera extrinsic，并在两个 Factory Patrol world 中保持 Gazebo RGB-D
  sensor pose 一致。
- 加入 conventional `camera_color_optical_frame`、ROS-Gazebo topic bridge 和 RGB/Depth/CameraInfo validation。
- 实现同步的 2D/Depth/CameraInfo 处理、bbox-center ROI median depth、invalid-depth rejection、
  pinhole projection，以及通过 observation-time TF2 转换到 `map`。

### 可替换 Detector 边界

- 使用 `DetectorBackend` 封装 CPU OpenCV-DNN YOLOX-S。
- 发布标准 `vision_msgs/msg/Detection2DArray`，使 geometry、tracking、mission 和 safety
  code 不依赖 YOLO 专用细节。
- 增加带 checksum 校验的 model preparation script；权重不提交，也不会在 launch 时隐式下载。

### 有状态目标与决策

- 增加 class-aware spatial association 和 `TENTATIVE`、`CONFIRMED`、`LOST`、`PROCESSED`
  lifecycle。
- 增加多帧确认、丢失/reacquisition、EMA、cooldown 和 semantic inspection event。
- duplicate suppression 保持在 `robot_tasks`，包括 mission 活动时抑制新 target-ID event。
  长时 benchmark 记录了三个额外 event，但 false mission starts 为零。

### Navigation 集成

- 在配置的 `1.2 m` standoff 处规划 Observation Pose，并让机器人朝向目标，而不是驶向物体中心。
- 复用既有 `NavigateSequence` adapter 与 Nav2；没有 visual servoing，也没有 perception velocity publisher。
- 一次 smoke run 返回 Nav2 `SUCCEEDED`，最终机器人与目标距离 `1.342032 m`。Nav2 goal
  tolerance 解释了部分差异；该值不属于正式 Benchmark 的 standoff metrics。

### Safety 与故障处理

- 将 confirmed person observation 转为 semantic `CLEAR`、`SPEED_LIMITED`、`STOP` 和 danger-zone `STOP` event。
- 这些 event 接入两种既有 Safety Gate，并按最严格活动 source 解析。
- 增加 camera stream、CameraInfo、detector、TF、depth quality 和 pipeline 的标准 diagnostics，
  再连接既有 system monitor 和 fault supervisor。
- 验证中断或无效 perception 会抑制下游输出，而不是产生虚假坐标或任务。

### 可复现评估

- 增加 detector、geometry、mission、safety 和 invalid-depth 的隔离 headless profile。
- 在 JSON/CSV 中记录 clock policy、warmup/exclusion、source commit/tree state、configuration、
  raw sample 和 nearest-rank statistic。
- 早期 smoke observation 与正式 benchmark 分开保存。

## 简历可用的已验证指标（Resume-ready Verified Metrics）

以下全部是已提交 Benchmark artifact 的 Gazebo/WSL simulation 结果，不是实体机器人的保证：

- 3D localization RMSE：`0.02351 m`，30 个样本，距离 `1.7-3.7 m`；P95 error `0.05239 m`。
- CPU OpenCV-DNN YOLOX-S inference：30 个样本，Mean `526.189 ms`，P95 `568.830 ms`。
- Detection-to-Nav2-goal：5 次，Mean `2.050 s`，P95 `2.790 s`。
- Safety STOP response：10 次，Mean `0.1806 s`，P95 `0.214 s`。
- Visual inspection：`5/5` successful，分类失败为零，false mission starts 为零。
- Invalid-depth rejection：`20/20`，false-valid 3D output 为零。
- Speed limiting：上游 `0.35 m/s` 在 `0.226 s` 内被限制为 `0.15 m/s`。
- Full ROS 2 baseline：21 packages，648 tests，0 errors，0 failures，0 skipped。

证据：

- [Benchmark JSON](../src/robot_experiments/results/factory_patrol_phase8_20260815_011022.json)
- [Benchmark CSV](../src/robot_experiments/results/factory_patrol_phase8_20260815_011022.csv)
- [实验方法与解释](experiment_report.md)

## 不能夸大的结果

32 组配对静态观测中，原始 x/y/z 标准差为 `0.00161 / 0.00167 / 0.00540 m`，EMA 为
`0.00181 / 0.00197 / 0.00607 m`。本样本中 EMA 三个轴都略差，因此不宣称 filtering improvement。

一次长时运行中，一个静态 physical target 产生了 `two IDs`。短时测试展示了同 ID reacquisition，
任务层 suppression 防止额外 event 启动重复 mission，但 persistent appearance identity 尚未实现。

三个 mission trial 中观察到 `Thirteen transient` non-OK `perception/tf` sample；它们均恢复且任务
成功，失败 TF lookup 不产生 map coordinate。

## 主要设计决策

| 决策 | 原因 |
| --- | --- |
| RGB-D 而非 RGB-only | 提供 metric geometry，不依赖单目尺度假设。 |
| 标准 detector output | 让 inference backend 可替换。 |
| 先验证 Geometry | 将 camera/TF 正确性与 model variability 分开。 |
| Confirm 后再 action | 防止单帧 detection 触发 mission 或 safety transition。 |
| Observation Pose | 保留 standoff 和朝向，不驶向 target center。 |
| Semantic safety event | 保持运动权限在既有 final gate。 |
| 独立 health channel | 防止 perception failure 被当作空场景且安全。 |
| 延后 TensorRT 与 ReID | 避免不支持的部署声明，保持 benchmark scope 一致。 |

## 已知限制

- 指标仅来自 WSL2/Gazebo；hardware calibration、timing 和 safety 尚未验证。
- CPU inference 较慢，是当前 perception latency 的主要来源。
- Tracking 是短时空间关联，没有 appearance ReID。
- EMA 没有改善收集到的静态样本稳定性。
- Mission 负载下出现过瞬时 TF warning，随后恢复。
- Gazebo settling 与积分 wheel odometry 可能有不同 origin，影响 geometry error。
- Inspection 假设目标静止且使用固定 Observation Pose，不是 pursuit 或 visual servoing。

## 当前状态

系统已发布 `visual-perception-v1`。当前仓库包含完整仿真闭环、可复现验证入口与固定
Benchmark 证据；实体硬件部署仍属于未来工作。
