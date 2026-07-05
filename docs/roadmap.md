# Roadmap

路线图按 Phase 0 到 Phase 6 整理，并区分 P0、P1、P2。当前阶段完成的是 Phase 0 文档重构与项目定位统一。

## Priority Definition

| Priority | Meaning |
| --- | --- |
| P0 | 直接影响项目定位和规控岗位展示主线。 |
| P1 | 强化工程完整度和演示可信度。 |
| P2 | 提升最终展示、CI 和报告质量。 |

## Phases

| Phase | Priority | Goal | Status | Deliverables |
| --- | --- | --- | --- | --- |
| Phase 0 | P0 | 项目定位统一与文档重构 | current | README、docs 架构、导航、定位、控制、安全、底盘、仿真、实验模板、面试口径。 |
| Phase 1 | P0 | Nav2 与 costmap 能力补强 | planned | costmap 参数审查、obstacle / voxel / inflation 验证、RPP 参数调优、basic / advanced 启动说明。 |
| Phase 2 | P0 | 控制算法实验体系 | planned | Pure Pursuit / Stanley / RPP / MPPI 指标采集，轨迹对比，cmd_vel 曲线，实验脚本。 |
| Phase 3 | P1 | 底盘通信与 odom / TF 工程化 | planned | 协议 v2、heartbeat、timeout、fault_code、odom covariance、TF 检查脚本。 |
| Phase 4 | P1 | 定位、重定位与安全状态机 | planned | localization health 接入统一 safety state，LOST / RECOVERED 演示，任务暂停 / 恢复策略。 |
| Phase 5 | P1 | 厂区巡检仿真场景 | planned | `factory_patrol.sdf`、factory map、stations、zones、多点巡检、临时障碍、定位恢复 demo。 |
| Phase 6 | P2 | 实验报告、CI、最终展示 | planned | 正式实验报告、GitHub 首页 polish、CI 验收、演示视频 / 截图索引。 |

## Phase 1 Detail

Phase 1 建议优先做：

1. 梳理 `nav2_basic.yaml` 和 `nav2_advanced.yaml` 的参数边界；
2. 验证 map_server、AMCL、planner、controller、costmap topic 是否稳定；
3. 针对低速巡检设置 RPP 的 lookahead、速度上限、goal tolerance；
4. 检查 obstacle / voxel layer 的 scan 输入、marking、clearing；
5. 建立最小导航验收脚本和 RViz 截图说明；
6. 不写性能结论，先写可复现实验步骤。
