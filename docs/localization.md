# 定位系统

<!-- Existing localization-check compatibility marker: Phase 4B. -->

当前定位链路以 AMCL 为主，适用于半封闭室内或园区低速巡检。AMCL、Nav2、EKF 和底盘
里程计统一使用 `map`、`odom`、`base_footprint` frame；URDF 提供固定的
`base_footprint -> base_link`。

## AMCL 链路

```text
/scan + static map -> AMCL -> /amcl_pose + map -> odom
odom source -> odom -> base_footprint -> base_link
/amcl_pose + TF checks -> localization_health_monitor_node -> /localization/health
```

- `/scan` 来自 `robot_sensors` 的标准化激光数据。
- `/amcl_pose` 是 `map` frame 下的位姿和 covariance。
- `/initialpose` 由 RViz 2D Pose Estimate 或上层重定位流程发布。
- `map -> odom` 由 AMCL 维护，`odom -> base_footprint` 由底盘或 EKF 提供。

相关配置和启动入口：

- `src/robot_navigation/config/nav2_basic.yaml`
- `src/robot_navigation/config/nav2_advanced.yaml`
- `src/robot_navigation/launch/nav.launch.py`
- `src/robot_navigation/src/localization_health_monitor_node.cpp`

## 定位健康监控

启动：

```bash
ros2 launch robot_navigation localization_health.launch.py use_sim_time:=true
```

订阅：

| Topic | Type | 用途 |
| --- | --- | --- |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 读取位姿、covariance 和时间戳。 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | LOST 后触发恢复状态。 |

发布：

| Topic | Type | 用途 |
| --- | --- | --- |
| `/localization/health` | `std_msgs/msg/String` | 当前状态、covariance、timeout 和 TF 摘要。 |
| `/localization_health` | `robot_interfaces/msg/LocalizationHealth` | 兼容既有可视化和 mission 接口。 |

## 状态与阈值

| State | 含义 |
| --- | --- |
| `LOCALIZATION_UNKNOWN` | 尚未收到新鲜的 `/amcl_pose`。 |
| `LOCALIZATION_OK` | pose 新鲜、covariance 正常且必要 TF 可用。 |
| `LOCALIZATION_UNSTABLE` | covariance 超过 warn 阈值，或 TF 短暂不可用。 |
| `LOCALIZATION_LOST` | pose 超时、covariance 持续超过 lost 阈值，或 TF 长时间不可用。 |
| `LOCALIZATION_RECOVERING` | LOST 后收到 `/initialpose`，正在等待收敛。 |
| `LOCALIZATION_RECOVERED` | 已恢复稳定，短暂发布后回到 `LOCALIZATION_OK`。 |

从 `/amcl_pose.pose.covariance` 使用 `covariance[0]`、`[7]` 和 `[35]` 读取 x、y、yaw；xy
使用 x/y 中较大值。

| Parameter | Default | 含义 |
| --- | --- | --- |
| `covariance_warn_xy` | `0.25` | xy 超过后进入 `LOCALIZATION_UNSTABLE`。 |
| `covariance_lost_xy` | `0.8` | xy 持续超过后进入 `LOCALIZATION_LOST`。 |
| `covariance_warn_yaw` | `0.25` | yaw 超过后进入 `LOCALIZATION_UNSTABLE`。 |
| `covariance_lost_yaw` | `0.8` | yaw 持续超过后进入 `LOCALIZATION_LOST`。 |
| `lost_hold_time_sec` | `2.0` | lost covariance 条件需持续的时间。 |
| `amcl_timeout_sec` | `1.0` | `/amcl_pose` 超时阈值。 |
| `tf_timeout_sec` | `0.5` | 必要 TF 不可用的 lost 阈值。 |
| `recovered_hold_time_sec` | `1.0` | `LOCALIZATION_RECOVERED` 保持时间。 |
| `publish_period_ms` | `200` | health 发布周期。 |

上述 covariance 值是仿真/mock 默认值，不是实体底盘标定结论。

## TF 与重定位流程

监控器通过 `tf2_ros::Buffer::canTransform` 检查 `map -> odom` 和 `odom -> base_footprint`。
单次查询失败先产生 `LOCALIZATION_UNSTABLE`，持续超过 `tf_timeout_sec` 才进入 LOST。

```text
covariance high / AMCL timeout / TF unavailable
  -> LOCALIZATION_LOST
  -> operator or higher-level flow publishes /initialpose
  -> LOCALIZATION_RECOVERING
  -> covariance and TF recover
  -> LOCALIZATION_RECOVERED -> LOCALIZATION_OK
```

任务暂停、停车和恢复由 `robot_tasks` 与 Safety Gate 分工完成；定位监控器本身只发布
健康状态，不发布速度。

## Safety 与任务接口

Safety Gate 的定位映射为：

| Localization health | Safety state |
| --- | --- |
| `LOCALIZATION_OK` | `NORMAL` |
| `LOCALIZATION_UNSTABLE` | `SPEED_LIMITED` |
| `LOCALIZATION_LOST` | `LOCALIZATION_LOST`，默认最终速度为零 |
| `LOCALIZATION_RECOVERING` / `LOCALIZATION_RECOVERED` | `RECOVERY` |

`speed_limited_max_linear_mps=0.15`、`speed_limited_max_angular_radps=0.4` 是当前低速策略。
既有任务接口包括：

- `robot_interfaces_navigation/srv/RequestRelocalization.srv`
- `robot_interfaces/msg/LocalizationHealth.msg`
- `robot_tasks` 中的 `mission_localization_workflow` 与 `mission_runner_node`

## 验证

```bash
bash scripts/check_localization_health.sh
bash scripts/check_localization_runtime_topics.sh
bash scripts/check_safety_state_machine.sh
bash scripts/check_safety_runtime_topics.sh
```

Factory Patrol 恢复演示入口：

```bash
bash scripts/run_factory_patrol_localization_recovery_demo.sh
```

该入口列出 `/localization/health`、`/safety/state`、`/safety/reason`、`/amcl_pose`、`/tf`
等需要观察的 topic。静态检查只验证结构；具体 LOST/RECOVERED transition 以带日志的
ROS2 运行记录为准。

## 已知限制

- 当前阈值服务于仿真和 mock，实体机器人需要重新标定。
- WSL2/Gazebo 的启动时序可能产生瞬时 TF warning；监控器会将其区分为不稳定或丢失。
- 重定位恢复依赖上层发布正确的 `/initialpose`，不包含自动全局重定位算法。
