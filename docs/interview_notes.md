# Interview Notes

面试讲解建议按“场景 -> 闭环 -> 算法 -> 工程 -> 安全 -> 实验 -> 边界”展开，避免把 planned 内容说成已完成。

## 1. 应用场景

这个项目面向厂区 / 园区半封闭低速巡检 AMR。典型任务是机器人在静态地图中按巡检点运行，途中依赖 AMCL 定位、Nav2 规划与控制、底盘反馈和安全门控完成闭环。

可以强调：

- 场景低速、半封闭，重点是稳定导航和工程链路；
- 当前以 Gazebo / RViz / Mock 后端验证为主；
- Serial / UDP 后端是上位机到底盘控制器的适配，不是底盘固件开发。

## 2. 系统闭环

主线：

```text
目标点 -> map_server -> AMCL -> Nav2 planner -> Nav2 controller
-> cmd_vel_mux -> cmd_vel_safety_gate -> chassis_driver
-> Mock / Serial / UDP -> odom / TF / chassis state -> 监控与故障监督
```

讲解重点：

- Nav2 输出的是候选速度，不直接等于最终底盘速度；
- 最终 `/cmd_vel` 由 safety gate 统一发布；
- 底盘反馈重新进入定位、监控和任务状态。

## 3. 算法选型

可以说明：

- AMCL 适合 2D 激光 + 静态地图的室内 / 半封闭场景；
- Navfn 是清晰可靠的基础全局规划器；
- SmacPlanner2D 更适合展示 advanced 配置；
- RPP 是低速巡检常用的路径跟随式局部控制器；
- MPPI 适合展示采样优化类局部控制；
- Pure Pursuit / Stanley 在项目中作为 standalone 路径跟踪实验，不混同于 Nav2 controller 插件。

## 4. 工程落地

可以讲：

- package 按导航、底盘、传感器、任务、安全、仿真、接口拆分；
- launch 文件组合不同运行模式；
- Mock 后端支持无硬件闭环；
- Serial / UDP 后端为真实底盘接入预留；
- shell 验收脚本和 GoogleTest 覆盖部分关键链路。

## 5. 安全保护

可以讲：

- cmd_vel 仲裁解决导航、跟踪、遥控等多来源速度冲突；
- safety gate 做最终速度门控；
- watchdog 在速度输入超时时输出零速度；
- emergency stop service 可触发急停；
- fault supervisor 根据 system health 请求急停或恢复；
- 完整安全状态机仍在后续 Phase 4 统一。

## 6. 实验指标

可讲 planned 指标，但不要声称已有结论：

- navigation: `success_rate`、`arrival_time`、`final_error`、`stop_count`；
- tracking: `rms_lateral_error`、`max_lateral_error`、`mean_heading_error`；
- command smoothness: `max_angular_velocity`、cmd_vel curve；
- safety: stop reason、stop_count、恢复时间。

## 7. 项目边界

实事求是：

- 当前是 ROS2 AMR 导航控制工程展示项目；
- 当前主验证环境是仿真和 Mock；
- 不声称真实工厂长期运行；
- 不伪造实验指标；
- 后续需要真实底盘标定、场景地图、参数调优和系统化实验。

## Common Questions

| Question | Answer points |
| --- | --- |
| 为什么用 AMCL？ | 半封闭室内 / 园区低速场景，已有静态地图和 2D 激光，AMCL 工程成熟、可解释。 |
| RPP 和 MPPI 怎么区分？ | RPP 更直接、参数少、适合低速路径跟随；MPPI 是采样优化控制，适合展示 advanced controller 调参。 |
| Pure Pursuit / Stanley 和 Nav2 controller 是什么关系？ | 它们是 standalone tracking experiment，用于算法对比；Nav2 controller 是导航栈内部局部控制器。 |
| 如何保证安全？ | 速度源仲裁、最终 safety gate、watchdog、急停服务、system monitor、fault supervisor。完整安全状态机 planned。 |
| 真实底盘接了吗？ | 仓库有 Mock / Serial / UDP 适配层；真实设备参数、标定和长期运行数据需要后续联调。 |
| 实验结果如何？ | Phase 0 不给虚假指标；已有模板，后续按 commit、参数、日志和地图记录结果。 |
