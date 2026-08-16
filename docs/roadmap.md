# 项目路线图

本文区分当前版本已经实现的能力与尚未实现的扩展方向。当前发布版本保持 Nav2、速度仲裁和
Safety Gate 的既有权限边界，Perception 没有速度发布权。

## 已实现（Implemented）

| 领域 | 当前能力 |
| --- | --- |
| 导航与定位 | Nav2 basic/advanced 配置、AMCL、costmap、定位健康与恢复接口。 |
| 控制与安全 | RPP/MPPI 配置、独立 Pure Pursuit/Stanley 实验、速度仲裁、watchdog、急停和统一 Safety Gate。 |
| Factory Patrol 仿真 | 主/工业预览 world、RGB-D、激光、IMU、odom、TF、语义 fixture、RViz 和演示入口。 |
| 视觉感知 | 可替换 OpenCV-DNN YOLOX-S detector、鲁棒深度投影、observation-time TF2。 |
| 目标与任务 | map-frame TargetManager、生命周期、Observation Pose、`robot_tasks` 和 Nav2 action 集成。 |
| 人员安全 | 人员距离/危险区域策略、hysteresis、语义事件与最终 Safety Gate 集成。 |
| Diagnostics | Camera、CameraInfo、Detector、Depth、TF 和 Pipeline diagnostics，接入 system monitor/fault supervisor。 |
| 验证与评估 | 静态/运行时脚本、648-test 软件基线、可重复 WSL2/Gazebo Benchmark 和已提交 JSON/CSV。 |

当前闭环为：

```text
RGB-D -> 2D detection -> depth/TF -> managed map target
                                  /                 \
                         robot_tasks             Safety Gate
                              |                       |
                            Nav2 -> cmd_vel mux ------+
                                      |
                                 final /cmd_vel
```

## Future Work（尚未实现）

- 在目标硬件上评估 C++ inference、ONNX Runtime 和 TensorRT。
- 在实体机器人上完成 camera/chassis calibration、部署、时序和安全验证。
- 增加 appearance-aware target ReID 与 dynamic-target evaluation。
- 使用 rosbag replay 和非静态场景评估 EMA 或其他 filtering 策略。
- 扩展工厂目标数据集、场景覆盖和 Benchmark 重复次数。
- 在相同地图、速度和采样条件下专项比较并调优 RPP/MPPI。
- 评估 basic global costmap 的动态 obstacle layer 和全局重规划策略。
- 提交带 command、commit、parameter 和日志的审核后截图与视频。

以上项目不属于当前版本的已验证能力。
