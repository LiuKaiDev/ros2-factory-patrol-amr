# 项目文档

这里是 ROS2 工厂巡检 AMR 的详细工程文档索引。项目首页用于快速了解系统和已验证指标，
本页按架构、仿真、验证和工程边界组织技术资料。

## 系统设计

- [系统架构](architecture.md)
- [导航系统](navigation.md)
- [定位系统](localization.md)
- [控制系统](control.md)
- [Safety 状态机](safety_state_machine.md)

## 仿真与硬件

- [仿真场景](simulation_scenarios.md)
- [底盘通信协议](chassis_protocol.md)
- [底盘标定说明](calibration.md)

## 实验与项目总结

- [实验与 Benchmark 报告](experiment_report.md)
- [工程项目总结](project_summary.md)
- [项目路线图](roadmap.md)
- [面试复习要点](interview_notes.md)

## Showcase 与验证

- [Showcase 证据规范](showcase/README.md)
- [验证脚本清单](../scripts/README.md)

## 文档说明

- ROS package、Topic、interface、TF frame、parameter 和 command 保留源码中的英文标识。
- 未实际验证的能力明确标记为“未验证”。
- 仿真结果不等同于实体硬件验证。
- Benchmark 数据以提交的 JSON/CSV 实验产物为准。

## 开发历史

项目早期设计与视觉感知升级规划保留在 [`upgrade/`](upgrade/)，用于记录系统演进过程。
