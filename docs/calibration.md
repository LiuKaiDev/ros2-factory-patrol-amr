# Calibration

本文档给出低速巡检 AMR 底盘里程计标定流程。当前仓库没有真实底盘标定结论，所有记录表中的数值保持 `TBD`，不得当作实车结果引用。

## Calibration Goals

| Parameter | Current default | Goal |
| --- | --- | --- |
| `wheel_diameter_m` | `0.15` | 标定轮径或等效编码器比例，降低直线里程误差。 |
| `track_width_m` | `0.43` | 标定左右轮距或等效角速度比例，降低原地旋转 yaw 误差。 |
| `max_linear_speed_mps` | `0.6` | 约束低速调试和标定时的最大线速度。 |
| `max_angular_speed_radps` | `1.5` | 约束低速调试和标定时的最大角速度。 |
| `odom_pose_covariance_x` / `y` / `yaw` | `0.02` / `0.02` / `0.05` | 给 EKF / localization 提供里程计位姿不确定性。 |
| `odom_twist_covariance_vx` / `wz` | `0.03` / `0.05` | 给 EKF / localization 提供里程计速度不确定性。 |

`wheel_radius_m` 若在控制器或 URDF 中出现，应满足：

```text
wheel_radius_m = wheel_diameter_m / 2.0
```

当前 URDF / ros2_control 使用 `wheel_radius=0.075` 与 `wheel_diameter_m=0.15` 对齐。

## Straight-Line Wheel Diameter Calibration

目标：通过已知直线距离修正 `wheel_diameter_m` 或等效 encoder scale。

流程：

1. 在平整、防滑、可重复的地面上标出 5 m 直线。
2. 将机器人初始姿态对齐直线方向，记录初始 `/odom` 或 `/wheel/odom`。
3. 使用低速命令让机器人直行，例如 `0.2 m/s` 到 `0.3 m/s`。
4. 机器人停止后，记录实际行驶距离和 odom 行驶距离。
5. 计算修正比例：

```text
scale = actual_distance_m / odom_distance_m
new_wheel_diameter_m = old_wheel_diameter_m * scale
```

6. 更新 `wheel_diameter_m`，重新构建 / 启动后重复测试。
7. 正向和反向各重复多次，直到误差低于项目阈值。

记录模板：

| Run | Direction | Command speed | Actual distance | Odom distance | Scale factor | New wheel_diameter_m | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

## In-Place Track Width Calibration

目标：通过原地旋转修正 `track_width_m` 或等效角速度比例。

流程：

1. 在地面标记机器人初始朝向。
2. 使用低速角速度命令机器人原地旋转 360 deg。
3. 使用地面标记、外部测量或高可信姿态估计记录实际 yaw 变化。
4. 读取 `/odom` yaw 变化。
5. 计算修正比例：

```text
scale = odom_yaw_rad / actual_yaw_rad
new_track_width_m = old_track_width_m * scale
```

对于差速底盘，`track_width_m` 变大通常会降低同一左右轮速差推导出的 yaw 变化；`track_width_m` 变小则会放大 yaw 变化。

6. 更新 `track_width_m`，重复旋转测试。
7. 顺时针和逆时针各重复多次，直到角度误差低于项目阈值。

记录模板：

| Run | Direction | Command angular velocity | Actual yaw | Odom yaw | Scale factor | New track_width_m | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

## Odom Covariance Estimation

Phase 3B 在 `chassis_driver_node` 中为 `/odom` 和 `/wheel/odom` 填充默认 covariance。默认值是 mock / 仿真可用的工程占位，不来自真实标定：

| Field | Default |
| --- | --- |
| `pose.covariance[0]` x | `0.02` |
| `pose.covariance[7]` y | `0.02` |
| `pose.covariance[35]` yaw | `0.05` |
| `twist.covariance[0]` vx | `0.03` |
| `twist.covariance[35]` wz | `0.05` |

建议估计流程：

1. 固定 `wheel_diameter_m` 和 `track_width_m` 后，再估计 covariance。
2. 多次运行 5 m 直线、360 deg 原地旋转和短距离组合路径。
3. 每次记录 ground truth 与 `/odom` 的 x、y、yaw、vx、wz 误差。
4. 分别计算误差方差，更新 `odom_pose_covariance_*` 和 `odom_twist_covariance_*`。
5. 若存在 EKF，观察融合后定位是否过度信任或过度忽略 odom。

记录模板：

| Run | Scenario | x error variance | y error variance | yaw error variance | vx error variance | wz error variance | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

## Parameter Locations

| Parameter | Primary runtime location |
| --- | --- |
| `wheel_diameter_m` | `chassis_driver_node` ROS parameter, launch passthrough |
| `track_width_m` | `chassis_driver_node` ROS parameter, launch passthrough |
| `wheel_base_m` | `chassis_driver_node` ROS parameter, launch passthrough |
| `max_linear_speed_mps` | `chassis_driver_node` ROS parameter, launch passthrough |
| `max_angular_speed_radps` | `chassis_driver_node` ROS parameter, launch passthrough |
| `odom_pose_covariance_*` | `chassis_driver_node` ROS parameter, launch passthrough |
| `odom_twist_covariance_*` | `chassis_driver_node` ROS parameter, launch passthrough |

URDF / ros2_control 仍保留几何参数，用于机器人模型和 diff_drive_controller。Phase 3B 已将默认 `wheel_diameter_m=0.15`、`wheel_radius=0.075`、`track_width_m=0.43` / `wheel_separation=0.43` 对齐；真实底盘标定后应同步更新这些位置。

## Notes

- 标定地面材质会影响轮胎打滑，湿滑或粉尘地面应单独记录。
- 标定速度应保持低速，避免加速度过大导致打滑。
- 每次实验前确认初始位姿、轮胎压力 / 磨损状态和电池电压。
- 手工测量会引入误差，建议至少重复 5 次并记录方差。
- 标定结果必须注明底盘型号、轮胎状态、地面材质、ROS 参数版本和 commit。
- 当前文档只提供流程，不声明已经完成真实机器人标定。
