# Localization

当前定位链路以 AMCL 为主，适用于半封闭室内 / 园区低速巡检场景中的 2D 激光定位演示。Phase 4A 新增 `localization_health_monitor_node`，用于把 AMCL pose、covariance、TF 可用性和 timeout 转换为可展示的定位健康状态。该阶段只发布 health，不直接控制 safety gate 或任务暂停。

## AMCL Chain

```text
/scan + static map -> AMCL -> /amcl_pose + map -> odom
odom source -> odom -> base_footprint -> base_link
/amcl_pose + TF checks -> localization_health_monitor_node -> /localization/health
```

- `/scan`：来自 `robot_sensors` 标准化后的激光数据。
- `/amcl_pose`：AMCL 输出的机器人在 `map` frame 下位姿估计。
- `/initialpose`：RViz 2D Pose Estimate 或外部重定位流程发布的初始位姿。
- `map -> odom`：AMCL 维护的全局定位修正。
- `odom -> base_footprint`：由底盘里程计或 EKF 链路提供。
- `base_footprint -> base_link`：机器人模型 TF 链路的一部分。

AMCL / Nav2 配置位于：

- `src/robot_navigation/config/nav2_basic.yaml`
- `src/robot_navigation/config/nav2_advanced.yaml`
- `src/robot_navigation/launch/nav.launch.py`

当前 frame 命名保持一致：AMCL、Nav2 basic、EKF 和底盘 odom 主链均使用 `map`、`odom`、`base_footprint`；URDF 中 `base_footprint -> base_link` 是固定关节。

## Health Monitor

节点路径：

```text
src/robot_navigation/src/localization_health_monitor_node.cpp
```

启动入口：

```bash
ros2 launch robot_navigation localization_health.launch.py use_sim_time:=true
```

订阅：

| Topic | Type | Purpose |
| --- | --- | --- |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 读取 AMCL pose 和 covariance。 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | LOST 后收到初始位姿时进入 `LOCALIZATION_RECOVERING`。 |

发布：

| Topic | Type | Purpose |
| --- | --- | --- |
| `/localization/health` | `std_msgs/msg/String` | Phase 4A 主 health topic，包含状态、covariance、timeout 和 TF 摘要。 |
| `/localization_health` | `robot_interfaces/msg/LocalizationHealth` | 兼容现有可视化 / mission topic 的旧路径。 |

## States

| State | Meaning |
| --- | --- |
| `LOCALIZATION_UNKNOWN` | 启动后尚未收到 `/amcl_pose`，或 AMCL 尚未发布。 |
| `LOCALIZATION_OK` | `/amcl_pose` 新鲜，covariance 低于 warn 阈值，`map -> odom` 和 `odom -> base_footprint` TF 可用。 |
| `LOCALIZATION_UNSTABLE` | covariance 超过 warn 阈值但尚未持续到 lost hold time，或 TF 短暂不可用。 |
| `LOCALIZATION_LOST` | `/amcl_pose` 超时，或 covariance 持续超过 lost 阈值，或必要 TF 长时间不可用。 |
| `LOCALIZATION_RECOVERING` | LOST 后收到 `/initialpose`，正在等待 AMCL covariance 和 TF 收敛。 |
| `LOCALIZATION_RECOVERED` | 从 LOST / RECOVERING 回到稳定 OK，短暂发布后转为 `LOCALIZATION_OK`。 |

## Covariance Rules

从 `/amcl_pose.pose.covariance` 读取：

- x covariance：`covariance[0]`
- y covariance：`covariance[7]`
- yaw covariance：`covariance[35]`
- xy covariance：`max(covariance[0], covariance[7])`

默认阈值：

| Parameter | Default | Meaning |
| --- | --- | --- |
| `covariance_warn_xy` | `0.25` | xy 超过后进入 `LOCALIZATION_UNSTABLE`。 |
| `covariance_lost_xy` | `0.8` | xy 持续超过后进入 `LOCALIZATION_LOST`。 |
| `covariance_warn_yaw` | `0.25` | yaw 超过后进入 `LOCALIZATION_UNSTABLE`。 |
| `covariance_lost_yaw` | `0.8` | yaw 持续超过后进入 `LOCALIZATION_LOST`。 |
| `lost_hold_time_sec` | `2.0` | covariance lost 条件需要持续的时间。 |

这些阈值是仿真 / mock 默认值，不来自真实机器人标定。真实机器人应结合场地、传感器噪声和重定位实验重新估计。

## Timeout And TF Rules

| Parameter | Default | Meaning |
| --- | --- | --- |
| `amcl_timeout_sec` | `1.0` | 超过该时间没有新的 `/amcl_pose` 后进入 `LOCALIZATION_LOST`。 |
| `tf_timeout_sec` | `0.5` | 必要 TF 不可用持续超过该时间后进入 `LOCALIZATION_LOST`。 |
| `recovered_hold_time_sec` | `1.0` | 从 LOST / RECOVERING 稳定后短暂发布 `LOCALIZATION_RECOVERED`。 |
| `publish_period_ms` | `200` | health 发布周期。 |

TF 检查使用 `tf2_ros::Buffer::canTransform`：

- `map -> odom`
- `odom -> base_footprint`

单次 TF 查询失败先进入 `LOCALIZATION_UNSTABLE`；持续超过 `tf_timeout_sec` 才进入 `LOCALIZATION_LOST`。

## Relocalization Flow

```text
AMCL covariance high / AMCL timeout / TF unavailable
  -> localization_health_monitor_node publishes LOCALIZATION_LOST
  -> Phase 4A stops here for safety integration
  -> operator or higher-level flow publishes /initialpose
  -> monitor publishes LOCALIZATION_RECOVERING
  -> AMCL pose covariance falls below warn threshold and TF is available
  -> monitor publishes LOCALIZATION_RECOVERED briefly
  -> monitor returns to LOCALIZATION_OK
```

任务暂停、停车和恢复策略留到 Phase 4B 接入 `cmd_vel_safety_gate` / mission pause。本阶段不声明完成完整安全闭环，也不声明完成真实机器人重定位验证。

## Checks

静态检查：

```bash
bash scripts/check_localization_health.sh
```

运行时 topic 检查，需要 Nav2 / AMCL / localization health monitor 已运行：

```bash
bash scripts/check_localization_runtime_topics.sh
```

该 runtime 脚本只检查 `/amcl_pose`、`/initialpose`、`/localization/health`、`/tf`、`/tf_static`、`/map`、`/odom` 是否存在，不伪造任何定位恢复结果。

## Existing Task-Layer Hooks

仓库中仍保留 mission 层已有的重定位接口和演示链路：

- `robot_interfaces_navigation/srv/RequestRelocalization.srv`
- `robot_interfaces/msg/LocalizationHealth.msg`
- `robot_tasks` 中 `mission_localization_workflow` 和 `mission_runner_node` 相关逻辑
- `scripts/check_localization_auto_recovery.sh`
- `scripts/check_relocalization_resume.sh`

Phase 4A 新 monitor 与这些入口并行存在，不重构 mission runner 的任务暂停 / 恢复逻辑。

## Phase 4B Safety Hook

Phase 4B connects the Phase 4A `/localization/health` string topic to the final
`cmd_vel_safety_gate_node`.

Safety mapping:

| Localization health | Safety state |
| --- | --- |
| `LOCALIZATION_OK` | `NORMAL` |
| `LOCALIZATION_UNSTABLE` | `SPEED_LIMITED` |
| `LOCALIZATION_LOST` | `LOCALIZATION_LOST` |
| `LOCALIZATION_RECOVERING` / `LOCALIZATION_RECOVERED` | `RECOVERY` |

`LOCALIZATION_LOST` publishes zero `/cmd_vel` through the safety gate when
`localization_lost_stop=true`. `LOCALIZATION_UNSTABLE` uses the low-speed policy
with `speed_limited_max_linear_mps=0.15` and
`speed_limited_max_angular_radps=0.4`.

Checks:

```bash
bash scripts/check_safety_state_machine.sh
bash scripts/check_safety_runtime_topics.sh
```

The runtime check requires ROS2 nodes to be running and only verifies topic
presence. It does not claim a real localization recovery test.

## Phase 5B Factory Patrol Recovery Demo

Factory patrol localization recovery demo entry:

```bash
bash scripts/run_factory_patrol_localization_recovery_demo.sh
```

The script prints bad and recovery `/initialpose` examples from
`src/robot_simulation/config/factory_patrol_localization_recovery.yaml` and lists
topics to observe:

- `/localization/health`
- `/safety/state`
- `/safety/reason`
- `/amcl_pose`
- `/tf`

It does not claim the LOST/RECOVERED transitions have been observed until a real
ROS2/Nav2 run is captured.
