# ROS2 工厂巡检 AMR

这是一个基于 ROS 2 Jazzy 的工厂巡检 AMR 仿真项目，面向低速移动机器人在厂区通道、收货区、货架区和包装工位之间的巡检场景。项目集成了 Gazebo Sim 场景、RViz 可视化、Nav2 / AMCL 基础导航、速度仲裁、安全状态反馈、odom / TF 运行状态检查和脚本化启动流程，用于展示一套可运行、可检查的 ROS2 AMR 软件链路。

当前项目主要在 **WSL2 Ubuntu 24.04 + ROS 2 Jazzy** 环境下完成构建、测试和仿真验证。

## 项目亮点

- 基于 Gazebo Sim 的工厂巡检仿真场景，包含 dock、receiving、storage、packing、slow zone 和 inspection loop 等区域。
- 提供 RViz 可视化和调试配置，便于观察 robot model、laser scan、odom、TF 和 path。
- 接入 Nav2 / AMCL 基础导航配置，包含 costmap、footprint、inflation、RPP 等常用导航调试项。
- 提供 `/virtual_rc/cmd_vel` 手动控制入口，速度指令会经过 mux 和 safety gate 后再进入 `/cmd_vel`。
- 实现 `cmd_vel` mux、安全门控和 `/safety_state` 状态反馈。
- 提供 odom、TF、scan、safety state 等运行时 topic 检查脚本。
- 提供多点巡检、临时障碍物和定位恢复等 demo / validation 入口，便于验证巡检流程和运行时状态。
<!-- Phase 5B demo workflows -->

## 当前状态

| 内容                 | 状态   | 说明                                                        |
| -------------------- | ------ | ----------------------------------------------------------- |
| Gazebo / RViz 仿真   | 已验证 | 可启动 Factory Patrol 场景并观察机器人状态                  |
| Nav2 / AMCL 基础导航 | 已接入 | 提供基础导航参数和调试配置                                  |
| 手动控制链路         | 已验证 | `/virtual_rc/cmd_vel` 经 mux / safety gate 后驱动仿真机器人 |
| 运行时检查脚本       | 已提供 | 覆盖 topic、TF、odom、safety state 等检查                   |
| 真实机器人接入       | 未验证 | 当前项目不声明真实底盘或现场部署结果                        |

## 项目概览

Factory Patrol 是当前主要仿真场景。它模拟一台低速 AMR 在工厂区域内执行巡检任务，场景中包含充电停靠区、收货入库区、仓储货架区、包装工位区和中央巡检通道。

当前主要展示 world：

```text
src/robot_simulation/worlds/factory_patrol_industrial.sdf
```

默认基线 world：

```text
src/robot_simulation/worlds/factory_patrol.sdf
```

两个 world 使用相同的 ROS2 demo 启动入口。工业展示场景主要用于更清楚地展示厂区布局和巡检路线；默认基线场景用于保留稳定的回退版本。

## 工程能力展示

这个项目主要展示以下 ROS2 / 移动机器人软件工程能力：

- ROS2 workspace、package、launch、参数文件和脚本化 bringup 组织；
- Gazebo Sim 场景建模与 RViz 调试视图配置；
- Nav2 / AMCL 基础导航链路接入；
- 多来源速度输入的 mux 仲裁和 safety gate 设计；
- odom、TF、scan、safety state 等运行时状态检查；
- 面向 demo / validation 的脚本化验证流程。

## 系统链路（Closed-Loop Pipeline）

整体闭环链路如下：

```text
任务入口 / 手动控制
  -> Nav2 或 virtual RC
  -> cmd_vel_mux_node
  -> safety gate
  -> /cmd_vel
  -> Gazebo bridge / 仿真机器人
  -> /odom / TF / safety state 反馈
```

手动控制推荐链路如下：

```text
/virtual_rc/cmd_vel
  -> virtual_rc_node
  -> /teleop_cmd_vel
  -> cmd_vel_mux_node
  -> /cmd_vel
  -> Gazebo mobile_robot
```

## 快速开始

推荐环境：

- Ubuntu 24.04 / WSL2
- ROS 2 Jazzy
- Gazebo Sim
- colcon

安装依赖并构建：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

默认启动：

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true
```

## 启动 Factory Patrol Demo

默认基线场景：

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true
```

工业展示场景：

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  world_file:=$(ros2 pkg prefix robot_simulation)/share/robot_simulation/worlds/factory_patrol_industrial.sdf \
  gui:=true \
  use_rviz:=true
```

无 GUI 启动：

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=false use_rviz:=false
```

## 手动控制

推荐手动控制入口是 `/virtual_rc/cmd_vel`。

前进：

```bash
ros2 topic pub --times 100 --rate 10 /virtual_rc/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

停止：

```bash
ros2 topic pub --times 20 --rate 10 /virtual_rc/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

`/virtual_rc/cmd_vel` 会经过 `virtual_rc_node`、`cmd_vel_mux_node` 和 Gazebo bridge，最终驱动 Gazebo 中的 `mobile_robot`。手动测试建议从这个入口发布速度，不建议长期直接向 `/cmd_vel` 发布，避免和 mux / bridge 节点竞争。

## 运行时检查

Factory Patrol runtime topic 检查：

```bash
bash scripts/check_factory_patrol_runtime_topics.sh
```

Factory Patrol 静态资产检查：

```bash
bash scripts/check_factory_patrol_assets.sh
```

更多脚本说明见 [scripts/README.md](scripts/README.md)。Validation Scripts 的完整列表也放在该文档中。

## 验证结果

在 WSL2 Ubuntu 24.04 + ROS 2 Jazzy 环境下完成过一次完整验证：

```text
colcon test-result --verbose
Summary: 514 tests, 0 errors, 0 failures, 0 skipped
```

同时验证过：

- 工业展示场景可以在 Gazebo / RViz 中启动；
- `scripts/check_factory_patrol_runtime_topics.sh` 通过；
- `/safety_state` 为 `OK`；
- 通过 `/virtual_rc/cmd_vel` 发布前进命令后，Gazebo 中 `mobile_robot` 的 pose 发生明显变化，仿真运动链路已打通。

以上结果来自当前开发环境。更换系统、ROS 2 版本或 Gazebo 版本后，建议重新构建并运行测试。

## 项目结构

```text
src/
  robot_bringup/        # 启动入口
  robot_navigation/     # Nav2、AMCL 和导航参数
  robot_simulation/     # Gazebo world、RViz 和仿真节点
  robot_teleop/         # virtual RC、cmd_vel mux 和 safety gate
  robot_tasks/          # 巡检任务入口
  robot_interfaces*/    # 自定义消息和接口

docs/                   # 项目文档
scripts/                # 启动、检查和实验脚本
```

## 更多文档

- [docs/architecture.md](https://chatgpt.com/c/docs/architecture.md)
- [docs/navigation.md](https://chatgpt.com/c/docs/navigation.md)
- [docs/simulation_scenarios.md](https://chatgpt.com/c/docs/simulation_scenarios.md)
- [docs/showcase/README.md](https://chatgpt.com/c/docs/showcase/README.md)
- [scripts/README.md](https://chatgpt.com/c/scripts/README.md)
