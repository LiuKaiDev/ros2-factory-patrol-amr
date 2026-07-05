# Control

本项目涉及两类控制链路：Nav2 controller 下的局部控制，以及 standalone path tracking experiment 下的路径跟踪控制器。两者用途不同，文档和实验报告中需要分开描述。

## Nav2 Controller

| Controller | Location | Status | Role |
| --- | --- | --- | --- |
| Regulated Pure Pursuit (RPP) | `nav2_basic.yaml` | current | Nav2 局部控制器，根据全局路径输出速度候选。 |
| MPPI | `nav2_advanced.yaml` | current | Nav2 高级局部控制器，用采样优化方式输出速度候选。 |

Nav2 controller 的输出不是直接发到底盘，而是进入 cmd_vel 仲裁与 safety gate：

```text
Nav2 controller -> /nav2_cmd_vel or mux input -> cmd_vel_mux / twist_mux -> cmd_vel_safety_gate -> /cmd_vel
```

## Standalone Path Tracking Experiment

| Controller | Package | Status | Role |
| --- | --- | --- | --- |
| Pure Pursuit | `robot_path_tracking` | current | 独立路径跟踪实验，用于参考路径追踪与控制器对比。 |
| Stanley | `robot_path_tracking` | current | 独立路径跟踪实验，用于横向误差相关控制对比。 |

该链路适合做算法讲解和指标对比，不等同于 Nav2 controller_server 的插件实现。

## Metrics Plan

后续 Phase 2 建立控制实验体系时，建议输出以下指标。Phase 0 不伪造任何结果。

| Metric | Meaning | Status |
| --- | --- | --- |
| `lateral_error` | 横向误差时间序列 | planned |
| `heading_error` | 航向误差时间序列 | planned |
| `rms_lateral_error` | 横向误差 RMS | planned |
| `max_lateral_error` | 最大横向误差 | planned |
| `mean_heading_error` | 平均航向误差 | planned |
| `trajectory_compare` | 参考轨迹与实际轨迹对比 | planned |
| `cmd_vel_curve` | 线速度 / 角速度曲线 | planned |
| `stop_count` | 安全停车或控制中断次数 | planned |

## Experiment Boundary

- 可以说明项目中存在 Pure Pursuit / Stanley / RPP / MPPI 配置和代码。
- 可以说明后续会做 controller 对比。
- 不应在没有重新跑实验的情况下声称某控制器更优、收敛更快或误差更小。
