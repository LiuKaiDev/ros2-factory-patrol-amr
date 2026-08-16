# 项目技术总结

## 项目定位

本项目是面向工厂低速巡检场景的 ROS2 Jazzy AMR 软件栈，覆盖传感器输入、定位、导航、
RGB-D 感知、目标管理、任务执行、速度仲裁、安全约束、底盘通信、状态反馈和工程评估。
实现以模块边界和可验证性为核心：Nav2 保持导航职责，Perception 只产生环境语义，任务层
负责把目标转换为导航动作，Safety Gate 保持最终速度权限。

当前定量结论来自 WSL2/Gazebo 仿真，不代表实体机器人性能或安全认证。

## 系统架构

```text
LiDAR / IMU / Encoders / RGB-D
            |
            +--> AMCL / Odometry / TF ------------------+
            |                                            |
            +--> Detector -> DepthProjector -> TargetManager
                                                    |     |
                                             robot_tasks  safety event
                                                    |     |
                                                  Nav2    |
                                                    |     |
                                              cmd_vel mux |
                                                    |     |
                           diagnostics / faults --> Safety Gate
                                                          |
                                                    final /cmd_vel
                                                          |
                                             Chassis / Gazebo
                                                          |
                                             odometry / state feedback
```

运行时权限边界：

- Nav2 负责规划、局部控制和恢复行为。
- `robot_tasks` 拥有视觉巡检导航流程，通过 action adapter 调用 Nav2。
- Perception 不发布 `/cmd_vel` 或 `/nav2_cmd_vel`。
- cmd_vel mux 负责候选速度源仲裁。
- `cmd_vel_safety_gate` 综合安全状态并发布最终 `/cmd_vel`。
- 底盘层只消费最终速度，反馈 odometry 和 chassis state。

## 定位与 TF

AMCL 结合 `/scan` 和静态地图维护 `map -> odom`。底盘或 Gazebo 提供
`odom -> base_footprint`，robot description 提供 `base_footprint -> base_link` 以及 LiDAR、
IMU、RGB-D camera 固定变换。

定位健康监控结合 `/amcl_pose` 新鲜度、xy/yaw covariance 和 TF 可用性输出：

```text
LOCALIZATION_UNKNOWN
LOCALIZATION_OK
LOCALIZATION_UNSTABLE
LOCALIZATION_LOST
LOCALIZATION_RECOVERING
LOCALIZATION_RECOVERED
```

LOST 后由 operator 或上层流程发布 `/initialpose`，监控器等待 covariance 与 TF 恢复，再回到
`LOCALIZATION_OK`。仿真默认 covariance 与实体标定值在文档中明确区分。

相机 TF 为：

```text
base_link -> camera_link -> camera_color_optical_frame
```

Xacro 与 Gazebo 使用同一相机平移外参，optical frame 遵循 ROS 约定。三维投影使用图像观测
时间戳查询 TF，避免用最新变换替代历史观测。

## Nav2 导航

导航子系统包括 AMCL、planner server、controller server、behavior server、BT navigator、
global/local costmap 和 lifecycle 管理。仓库提供 basic/advanced 参数配置以及 RPP/MPPI controller
配置。任务、巡检和恢复流程通过 Nav2 action 提交目标，不实现并行视觉速度控制器。

独立 `robot_path_tracking` package 用于 Pure Pursuit 与 Stanley 算法实验，其输出仍需经过速度
仲裁与 Safety Gate，且不替代发布系统中的 Nav2。

## 底盘与运动反馈

`robot_hardware` 将 ROS2 速度与具体后端解耦：

- Mock backend 用于无硬件闭环、测试和演示。
- Serial/UDP backend 提供外部底盘控制器接入点。
- 文本协议 v2 包含 seq、timestamp、heartbeat、fault code 与 command timeout。
- ros2_control adapter 提供标准硬件接口。
- `/odom`、`/wheel/odom` 和 `/chassis/state` 构成运动与状态反馈。

轮径、轮距、速度上限和 odom covariance 有明确参数入口。仓库值用于仿真/Mock；实体机器人
需要通过直线、原地旋转和重复轨迹完成现场标定。

## RGB-D 环境感知

Gazebo RGB-D sensor 发布：

- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/color/camera_info`

Detector 默认 backend 为 CPU OpenCV-DNN YOLOX-S，输出标准
`vision_msgs/msg/Detection2DArray`。模型准备脚本下载并校验官方 OpenCV Zoo ONNX 文件，模型
不进入 Git 仓库，Detector backend 可替换而不改变下游接口。

DepthProjector 支持 `16UC1` 与 `32FC1`，在 detection ROI 内执行边界裁剪、无效深度拒绝和
鲁棒统计，再结合 CameraInfo 内参得到 camera-frame 三维点。geometry node 将其按 observation-time
TF2 转换到 `map` frame。

## TargetManager

TargetManager 对 map-frame 三维观测执行空间关联和生命周期管理：

```text
TENTATIVE -> CONFIRMED -> LOST -> retired
                  |
               PROCESSED
```

目标状态包含稳定 ID、类别、置信度、位置、观测次数和时间戳。配置支持 association distance、
confirmation hits、lost retirement 和 EMA。目标只在满足状态与类别策略时产生巡检或安全事件。

## 巡检任务

`robot_tasks` 订阅 `INSPECTION_REQUIRED`，根据目标位置、机器人姿态和
`standoff_distance` 计算 Observation Pose，经既有 navigation adapter 提交 Nav2 goal。成功到达
后发布检查状态，并通过 `MarkProcessed` 回写 TargetManager。任务锁和完成状态抑制重复触发。

设施操作、多点巡检、返回充电点和恢复流程继续由任务层编排，不把导航或速度控制职责下沉到
Perception。

## Safety Gate

安全链分为语义判定与最终执行：

1. 感知安全策略依据人员距离、危险区域、target state 和 hysteresis 输出
   `CLEAR / SPEED_LIMIT / STOP`。
2. 定位、底盘、watchdog、急停和 fault supervisor 产生各自安全状态。
3. Safety Gate 按优先级组合状态，对 mux 输出执行放行、限速或归零。
4. 只有 Safety Gate 发布最终 `/cmd_vel`。

该结构确保单个感知或任务节点失效时不能绕开最终安全控制点。

## Diagnostics

Perception diagnostics 监控 RGB、Depth、CameraInfo、Detector、depth quality、TF 和 pipeline
新鲜度，输出 `diagnostic_msgs/msg/DiagnosticArray`。system monitor 汇总感知、定位、导航、
底盘等状态，fault supervisor 将故障语义接入既有安全链。诊断只影响状态与既有安全入口，不
直接发布速度。

## Benchmark

固定 Benchmark 产物：

- [JSON](../src/robot_experiments/results/factory_patrol_phase8_20260815_011022.json)
- [CSV](../src/robot_experiments/results/factory_patrol_phase8_20260815_011022.csv)

核心结果：

| 指标 | 结果 |
| --- | --- |
| Detector processing latency | mean `526.189 ms`，P95 `568.830 ms` |
| 3D localization error | RMSE `0.02351 m`，P95 `0.05239 m` |
| Detection -> Nav2 goal | mean `2.050 s` |
| Safety STOP response | mean `0.1806 s`，P95 `0.214 s` |
| Visual inspection mission | `5/5` successful，`0` false mission start |
| Invalid depth | `20/20` rejected |
| Speed limiting | `0.35 -> 0.15 m/s`，response `0.226 s` |

运行器通过隔离 DDS domain 和 Gazebo partition 分别执行 detector、geometry、mission、safety
和 invalid-depth profile。统计使用 nearest-rank 百分位；采样、warmup、排除项与原始数组见
[实验与 Benchmark 报告](experiment_report.md)。

## 工程验证

仓库包含 21 个 ROS package，提交时软件测试基线为 648 tests、0 errors、0 failures、0 skipped。
验证层次包括：

- `colcon build --symlink-install` 与 `colcon test`。
- C++ 单元测试、Python 测试和 launch/config 解析。
- package XML、shell/Python syntax 和接口 package 拆分检查。
- Nav2、localization、chassis、Safety Gate、Factory Patrol assets 静态检查。
- RGB-D、Detector、TargetManager、巡检、安全与 diagnostics 运行时 smoke test。
- Benchmark JSON/CSV 一致性和 Markdown 本地链接检查。

## Known Limitations

- 定量验证限定在 WSL2/Gazebo，没有实体机器人 benchmark 结论。
- Detector 当前使用 CPU OpenCV-DNN YOLOX-S，推理延迟受主机负载影响。
- 静态目标 Benchmark 中 EMA 没有改善稳定性。
- 长时间运行观察到偶发 target re-identification，关联尚未使用 appearance feature。
- 运行中存在过可恢复的 perception/TF health 瞬态告警。
- Gazebo world pose 与 bridged odometry 存在同步限制，验证流程需等待 fixture/pose 稳定。
- 视觉巡检假设目标与 Observation Pose 静态，不实现动态目标跟随。
- 实体底盘通信、相机外参、运动学参数和 covariance 仍需现场标定与安全验证。
