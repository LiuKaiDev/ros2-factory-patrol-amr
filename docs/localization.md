# Localization

当前定位链路以 AMCL 为主，适用于半封闭室内 / 园区低速巡检场景中的 2D 激光定位演示。

## Current AMCL Chain

```text
/scan + static map -> AMCL -> /amcl_pose + map->odom
odom source -> odom->base_footprint/base_link
```

- `/scan`：来自 `robot_sensors` 标准化后的激光数据。
- `/amcl_pose`：AMCL 输出的机器人在 map frame 下位姿估计。
- `map -> odom`：AMCL 维护的全局定位修正。
- `odom -> base_footprint`：由底盘里程计或 EKF 链路提供。
- `base_footprint -> base_link`：机器人模型 TF 链路的一部分。

当前 Nav2 配置中 AMCL 参数位于：

- `src/robot_navigation/config/nav2_basic.yaml`
- `src/robot_navigation/config/nav2_advanced.yaml`

## Localization Health

仓库中已有定位健康相关接口和逻辑：

- `robot_interfaces_navigation/srv/RequestRelocalization.srv`
- `robot_interfaces_navigation/msg/LocalizationHealth.msg`
- `robot_tasks` 中 `mission_localization_workflow` 和 `mission_runner_node` 相关逻辑；
- `scripts/check_localization_health.sh`
- `scripts/check_localization_auto_recovery.sh`
- `scripts/check_relocalization_resume.sh`

这些内容说明当前已有 LOST / RECOVERED / RELOCALIZING 相关工程入口。文档中仍将厂区巡检完整重定位演示场景标为 planned，因为 `factory_patrol` 场景和面向最终展示的脚本尚未建立。

## State Plan

| State | Current / planned | Meaning |
| --- | --- | --- |
| `UNKNOWN` | current | 等待定位输入或健康状态尚未建立。 |
| `OK` | current | 协方差等健康指标在阈值内。 |
| `LOST` | current | 定位协方差超过阈值，可触发任务暂停。 |
| `RELOCALIZING` | current | 收到重定位请求，等待初始位姿或恢复动作。 |
| `RECOVERED` | current | 定位恢复，可请求任务恢复。 |
| 巡检场景级自动恢复策略 | planned | 结合站点、巡检点、RViz/Gazebo 演示脚本进行最终展示。 |

## Relocalization Flow

```text
AMCL covariance abnormal
  -> localization health enters LOST
  -> mission may pause
  -> /v2/request_relocalization
  -> set initial pose from known station / operator input
  -> localization health enters RECOVERED
  -> mission resume requested
```

边界说明：当前项目已有上述流程的代码和检查脚本入口，但真实场景下的鲁棒性、恢复成功率和人工操作 SOP 需要后续实验验证后再写结论。
