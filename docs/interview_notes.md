# 面试复习要点

本页用于在不夸大验证结论的前提下介绍项目。推荐叙事顺序是：场景 -> 闭环 -> 算法 ->
工程实现 -> 安全 -> 验证 -> 边界。

## 30 秒版本

这是一个 ROS2 Jazzy 低速工厂巡检 AMR。系统在 AMCL、Nav2、velocity mux 和 Safety Gate
闭环上接入 RGB-D perception，通过 Detection2D、robust Depth、CameraInfo 与 observation-time
TF2 得到 map-frame target，再由 TargetManager、`robot_tasks` 和语义 Safety event 参与巡检
与人员安全。项目已在 WSL2/Gazebo 完成可重复 Benchmark；实体硬件结果仍未验证。

## 2 分钟版本

目标场景是走廊、实验室、园区或工厂通道中的半封闭巡检机器人。机器人在静态地图上执行
patrol goal，使用 AMCL 定位和 Nav2 规划。RGB-D pipeline 将 2D bbox 与 Depth/CameraInfo
同步，使用 bbox 中心 ROI median depth 恢复 3D 点，并按 observation timestamp 通过 TF2
转换到 `map`。TargetManager 对目标做多帧确认和生命周期管理；confirmed target 可触发
由 `robot_tasks` 所有的 Observation Pose mission，人员目标则只发布语义 safety event。

仓库按 navigation、localization、control、chassis、perception、task、safety、simulation 和 experiment
拆分 package，包含 Nav2 basic/advanced 参数、RViz debug layout、Factory Patrol 仿真资产、
mock/serial/UDP chassis backend、safety state topic、localization health monitoring，
以及静态和 runtime validation 脚本。

必须强调：simulation 和 mock 检查不能当作现场部署结果。当前有可追溯的 Gazebo/WSL
指标；实体硬件 calibration、timing 和 safety 仍未验证。

## 5 分钟结构

1. 场景：半封闭环境中的低速巡检。
2. 闭环：Perception -> Spatial Understanding -> Decision -> Navigation -> Control -> Safety。
3. 算法：Depth projection、observation-time TF2、TargetManager、AMCL、Navfn/SmacPlanner2D、RPP/MPPI。
4. 工程：ROS2 package、launch file、parameter、mock/serial/UDP backend 和 validation script。
5. 安全：final `/cmd_vel` gate、emergency stop、watchdog、speed limit、localization/chassis health input。
6. 验证：static/runtime check、fault injection、固定 JSON/CSV Benchmark 和 evidence boundary。
7. 边界：不伪造指标、不声称生产硬件，真实 tuning 仍属于未来工作。

## 常见问题

| 问题 | 回答要点 |
| --- | --- |
| 为什么使用 AMCL？ | 项目面向带激光输入的 2D 半封闭地图场景。AMCL 成熟、可解释，并与范围匹配。 |
| 为什么同时保留 RPP 和 MPPI？ | RPP 是 basic Nav2 中清晰的低速跟随控制器，MPPI 作为 advanced controller 选项保留。 |
| 为什么有 Pure Pursuit 和 Stanley？ | 它们是用于控制器比较的独立 tracking experiment，不替换 Nav2 plugin。 |
| 为什么需要 CameraInfo？ | `fx`、`fy`、`cx`、`cy` 是把像素与 Depth 投影成 camera-frame 3D point 的必要内参；内参无效时不产生点。 |
| 为什么不用 bbox 中心单像素 Depth？ | 单像素容易受空洞、边缘和 NaN/Inf 影响；项目对中心 ROI 过滤无效值后取 median。 |
| 为什么 TF 使用 observation timestamp？ | 目标点对应图像采样时刻；使用 latest TF 会把机器人运动引入空间误差并掩盖同步问题。 |
| TargetManager 解决什么问题？ | 它提供多帧确认、3D 空间关联、lifecycle、cooldown 和 duplicate suppression；它不是永久 ReID。 |
| `/cmd_vel` 如何受到保护？ | 候选命令先经过 mux，再进入 final Safety Gate；estop、watchdog、manual takeover、localization、scan、odom 和 chassis 状态都可以使其限速或停车。 |
| 是否已经接入实体硬件？ | 仓库有 mock、serial 和 UDP adapter layer；真实硬件 calibration 与长时间现场结果仍是未来工作。 |
| 是否有实验结果？ | 有 Gazebo/WSL Benchmark 与提交的 JSON/CSV；实体硬件结果仍未验证。 |

## 应避免的表述

- “Production-ready autonomous vehicle”。
- “Proven in a real factory”，除非有对应的具体运行证据。
- “Guaranteed safety” 或 “certified safety”。
- 任何不能追溯到真实运行的数值性能指标。
