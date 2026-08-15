# 项目路线图

本文档区分已经完成的仓库历史与可选的未来工作。视觉感知升级已经完成并发布到
`main`；`feature/visual-perception-upgrade` 仅保留为开发历史。

## 导航与控制基线

原始 Phase 0-6 roadmap 已完成，建立了 Nav2/AMCL 闭环、独立 tracking 实验、底盘协议
与里程计准备、定位健康、统一 Safety Gate 状态、Factory Patrol 仿真资产、Demo workflow、
CI/static check 和证据模板。

| 基线阶段 | 状态 | 结果 |
| --- | --- | --- |
| Phase 0 | 已完成 | 项目结构与证据边界 |
| Phase 1A/1B | 已完成 | Nav2 costmap/controller 与 RViz/runtime 检查 |
| Phase 2A/2B | 已完成 | Pure Pursuit/Stanley logging 与对比 workflow |
| Phase 3A/3B | 已完成 | 底盘 protocol v2、odom 和 calibration 准备 |
| Phase 4A/4B | 已完成 | Localization health 与统一 Safety Gate 状态 |
| Phase 5A/5B | 已完成 | Factory world/assets 与可复现 Demo workflow |
| Phase 6 | 已完成 | 文档、CI、报告和 showcase readiness 基线 |

## 视觉感知升级

| Phase | 状态 | 结果 |
| --- | --- | --- |
| Phase 0 | 已完成 | 范围与架构审计 |
| Phase 1 | 已完成 | RGB-D sensor、optical TF、topics、bridge、RViz、validation |
| Phase 2 | 已完成 | Robust depth projection 与 observation-time TF geometry |
| Phase 3 | 已完成 | 可替换的 OpenCV-DNN YOLOX-S detector integration |
| Phase 4 | 已完成 | map-frame TargetManager 与 lifecycle/event policy |
| Phase 5 | 已完成 | 通过既有 Nav2 执行 task-owned visual inspection |
| Phase 6 | 已完成 | 人员 safety event 接入既有 Safety Gate |
| Phase 7 | 已完成 | Perception diagnostics、monitoring、fault injection/recovery |
| Phase 8 | 已完成 | 可重复 Gazebo/WSL benchmark 与提交的 JSON/CSV |
| Phase 9 | complete（已完成） | 最终 README、文档、证据链接和 portfolio summary |

## 当前项目形态

```text
RGB-D -> 2D detection -> depth/TF -> managed map target
                                  /                 \
                         robot_tasks             Safety Gate
                              |                       |
                            Nav2 -> cmd_vel mux ------+
                                      |
                                 final /cmd_vel
```

Nav2 仍是导航栈。Perception 没有 velocity publisher，Safety Gate 仍是最终 authority。

## 未来工作（Future Work，尚未实现）

- 在目标硬件上评估 C++ inference、ONNX Runtime 和 TensorRT。
- 在实体机器人上完成整链路 calibration 与 validation。
- 增加 appearance-aware target re-identification 和 dynamic-target evaluation。
- 使用 rosbag replay 与非静态场景评估 filtering 选择。
- 扩展工厂目标数据集和 benchmark 重复次数。
- 记录包含确切 command、commit、parameters 和 logs 的审核后截图/视频。

以上条目不是当前版本已实现的能力。
