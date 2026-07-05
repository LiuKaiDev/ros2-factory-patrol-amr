# Calibration

本文档说明 wheel diameter、track width 和 odom 的标定思路。当前仓库没有真实底盘标定结论，本文不声称已完成真实标定。

## Parameters

| Parameter | Meaning | Impact |
| --- | --- | --- |
| `wheel_diameter` | 轮径 | 影响线速度、直线里程计距离。 |
| `track_width` | 左右轮距 | 影响角速度、原地旋转里程计角度。 |
| encoder scale | 编码器脉冲到轮速 / 距离换算 | 影响 `/wheel/odom` 精度。 |
| odom covariance | 里程计不确定性 | 影响 EKF / 定位融合权重。 |

## Straight-line Calibration

目标：修正轮径或线速度比例。

流程：

1. 在平整地面标出已知距离，例如 2 m 或 5 m。
2. 机器人低速直行，记录 `/wheel/odom` 或 `/odom` 的位移。
3. 重复多次，正向和反向各采样。
4. 计算实际距离 / 里程计距离的比例。
5. 调整 `wheel_diameter` 或 encoder scale。
6. 重新运行直到误差进入项目设定阈值。

记录模板：

| Run | Command speed | Ground truth distance | Odom distance | Scale factor | Notes |
| --- | --- | --- | --- | --- | --- |
| TBD | TBD | TBD | TBD | TBD | TBD |

## In-place Rotation Calibration

目标：修正 track width 或角速度比例。

流程：

1. 让机器人原地旋转 360 度或 720 度。
2. 使用地面标记、外部测量或高可信姿态估计记录真实旋转角。
3. 对比 `/odom` yaw 变化。
4. 若 odom yaw 偏大或偏小，调整 `track_width` 或角速度比例。
5. 多次重复并取稳定结果。

记录模板：

| Run | Command angular velocity | Ground truth angle | Odom angle | Track width scale | Notes |
| --- | --- | --- | --- | --- | --- |
| TBD | TBD | TBD | TBD | TBD | TBD |

## Odom Validation

标定后建议做组合路径验证：

- 直线往返；
- 方形轨迹；
- S 弯轨迹；
- 停止后 odom 漂移检查；
- 不同速度下的误差趋势。

真实标定结果必须注明底盘型号、轮胎状态、地面材质、电池电压、ROS 参数版本和 commit。
