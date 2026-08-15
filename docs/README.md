# 项目文档

这里是 ROS2 工厂巡检 AMR 项目的中文工程文档索引。README 首页用于快速了解项目定位、
能力、已验证指标和 Demo；本页按主题组织详细设计、验证方法和工程边界。

## 系统设计

- [系统架构](architecture.md)
- [导航系统](navigation.md)
- [定位系统](localization.md)
- [控制系统](control.md)
- [安全状态机](safety_state_machine.md)

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

## 文档约定

- 文中的 package、node、ROS Topic、message/interface、TF frame、parameter、command 和
  file path 保持代码中的英文标识。
- `TBD` 表示尚未由真实、可追溯运行填充；仿真结果不会被描述为实体硬件验收。
- Phase 8 JSON/CSV 是 Benchmark 数值的 source of truth；文档中的指标不得脱离产物修改。
- 文档整理不会改变 ROS runtime code、配置、算法、Safety Gate、Nav2 或实验产物。

## 开发历史

视觉感知升级规划属于已完成 Phase 0–9 的 design history，保留在
[docs/upgrade/visual_perception_upgrade_plan.md](upgrade/visual_perception_upgrade_plan.md)，
不作为项目首页的主要阅读入口。
