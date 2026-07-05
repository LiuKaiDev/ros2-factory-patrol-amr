# Chassis Protocol

底盘适配层目标是把 ROS2 上位机中的速度指令、里程计、底盘状态和故障状态，与具体底盘控制器或仿真后端连接起来。它不是 VCU 固件开发，也不声称覆盖真实底盘内部电机控制。

## Current Chassis Adapter

`robot_hardware` 当前包含：

- `chassis_driver_node`：ROS2 底盘驱动节点；
- `chassis_simulator_node`：无真实硬件时的底盘模拟器；
- `mock_backend`：Mock 后端，便于本地闭环和测试；
- `serial_backend`：串口后端；
- `udp_backend`：UDP 后端；
- `chassis_packet`：协议编解码；
- `chassis_hardware_interface`：ros2_control SystemInterface；
- `chassis_system_adapter`：底盘系统适配逻辑。

## Current Line Protocol

当前头文件中存在以下数据结构和编解码入口：

- `ChassisCommand`
- `ChassisOdometryPacket`
- `ChassisStatePacket`
- `EncodeCommand`
- `EncodeOdometry`
- `EncodeState`
- `ParseCommandLine`
- `ParsePacketLine`

现有协议可概括为行文本消息：

```text
CMD   -> 上位机发送速度命令
ODOM  -> 底盘或后端反馈里程计
STATE -> 底盘或后端反馈状态
```

实际字段以 `src/robot_hardware/include/robot_hardware/chassis_packet.hpp` 和对应 `.cpp` 实现为准。

## Backends

| Backend | Status | Responsibility |
| --- | --- | --- |
| Mock | current | 本地无硬件闭环、单元测试和演示。 |
| Serial | current adapter | 通过串口连接外部底盘控制器；真实设备参数需现场配置。 |
| UDP | current adapter | 通过 UDP 连接外部底盘控制器或模拟器。 |
| Mick binary frame | partial current | 仓库中存在相关 frame 编解码入口，具体设备适配能力以代码和测试为准。 |

## Feedback Topics

底盘层需要支撑以下反馈链路：

- `/odom`
- `/wheel/odom`
- `/chassis/state`
- `odom -> base_footprint` 或 EKF 输入
- battery / status / fault message（当前字段能力以接口实现为准）

## Protocol v2 Phase 3A

Phase 3A 已在上位机适配层补充文本协议 v2 的最小工程骨架，目标是让 Mock / Serial / UDP 后端具备 seq、timestamp、heartbeat、fault_code 和命令超时停车的统一入口。该阶段不包含真实底盘硬件联调结论，也不声明某个厂商控制器已经完整适配。

协议 v2 当前以行文本为主，当 `chassis_driver_node` 参数 `protocol:=text_v2` 时，Serial / UDP 后端会发送 v2 命令和 heartbeat；Mock 后端会记录 v2 frame，便于单元测试和本地闭环检查；`chassis_simulator_node` 可解析 `CMDV2` / `HEARTBEATV2` 并回传 `ODOMV2` / `STATEV2`。已有 `text` 和 `mick_binary` 路径保持兼容。

当前 v2 行格式：

```text
CMDV2       <seq> <timestamp_sec> <mode> <vx_mps> <vy_mps> <wz_radps> <max_v_mps> <max_w_radps> <estop>
ODOMV2      <seq> <timestamp_sec> <x_m> <y_m> <yaw_rad> <vx_mps> <vy_mps> <wz_radps> <left_rpm> <right_rpm>
STATEV2     <seq> <timestamp_sec> <mode> <battery_v> <fault_code> <estop> <connected> <temperature_c>
HEARTBEATV2 <seq> <timestamp_sec> <source>
```

故障码枚举以代码为准，当前最小集合为：

| Code | Name | Meaning |
| --- | --- | --- |
| 0 | `NONE` | 无已知故障 |
| 1 | `CMD_TIMEOUT` | `/cmd_vel` 超时后已发送零速度命令 |
| 2 | `HEARTBEAT_TIMEOUT` | 超过 heartbeat / IO 超时窗口未收到后端反馈 |
| 3 | `BACKEND_DISCONNECTED` | 后端未打开或写入失败 |
| 4 | `MALFORMED_PACKET` | 收到无法解析的文本行或 Mick binary frame |
| 5 | `ESTOP_ACTIVE` | v2 状态反馈报告 emergency stop |

`/chassis/state` 当前消息没有独立 `fault_code` 字段，因此 Phase 3A 将故障码、最近接收 seq / timestamp、estop、温度和轮速摘要追加在 `status` 字符串中，例如 `fault_code=NONE(0):rx_seq=...`。后续如需机器可读字段，应在接口包中新增 msg 字段并做兼容迁移。

`chassis_driver_node` 新增参数：

| Parameter | Default | Purpose |
| --- | --- | --- |
| `heartbeat_period_ms` | `500` | 周期发送 `HEARTBEATV2`，小于等于 0 时关闭 |
| `heartbeat_timeout_ms` | `1500` | 非 Mock 后端反馈超时判定窗口 |
| `command_timeout_ms` | `500` | `/cmd_vel` 超时后发送零速度命令 |
| `max_linear_speed_mps` | `0.6` | 限制 `/cmd_vel` 线速度，并写入 `CMDV2` 的上限字段 |
| `max_angular_speed_radps` | `1.5` | 限制 `/cmd_vel` 角速度，并写入 `CMDV2` 的上限字段 |

静态检查入口：

```bash
bash scripts/check_chassis_protocol_v2.sh
```

该脚本只检查仓库中 v2 结构、驱动 hook 和单测锚点是否存在；它不是串口 / UDP 真实联通测试。

## Odom / Kinematics Phase 3B

Phase 3B 聚焦底盘里程计、运动学参数和标定文档工程化。它与 Phase 3A 的关系是：

- Phase 3A 解决通信可靠性骨架：`text_v2`、seq、timestamp、heartbeat、fault_code 和 command timeout。
- Phase 3B 解决 odom / TF / kinematics / calibration：轮径、轮距、速度上限、odom covariance 和标定流程。

底盘参数默认值：

| Parameter | Default | Used by | Notes |
| --- | --- | --- | --- |
| `wheel_diameter_m` | `0.15` | `chassis_driver_node` / `chassis_system_adapter` / ros2_control hardware interface | 影响轮速与线速度换算；URDF 中 `wheel_radius=0.075` 与其一致。 |
| `wheel_base_m` | `0.42` | mecanum / four-wheel geometry estimate | 差速底盘主要依赖 `track_width_m`。 |
| `track_width_m` | `0.43` | 差速角速度换算、轮速估计 | 与 URDF `wheel_separation=0.43` 和 diff_drive_controller `wheel_separation=0.43` 对齐。 |
| `max_linear_speed_mps` | `0.6` | `chassis_driver_node` command clamp and `CMDV2` max field | 低速巡检默认上限，不代表实车极限。 |
| `max_angular_speed_radps` | `1.5` | `chassis_driver_node` command clamp and `CMDV2` max field | 低速调试默认上限，不代表实车极限。 |

`wheel_diameter_m` 影响直线 odom 距离：轮径偏大会让同样轮速推导出的线速度 / 位移偏大，轮径偏小则相反。`track_width_m` 影响差速 yaw：轮距偏大会降低同样左右轮速度差推导出的角速度，轮距偏小则会放大角速度。

`chassis_driver_node` 现在为 `/odom` 和 `/wheel/odom` 填充平面里程计 covariance：

| Field | Parameter | Default |
| --- | --- | --- |
| `pose.covariance[0]` | `odom_pose_covariance_x` | `0.02` |
| `pose.covariance[7]` | `odom_pose_covariance_y` | `0.02` |
| `pose.covariance[35]` | `odom_pose_covariance_yaw` | `0.05` |
| `twist.covariance[0]` | `odom_twist_covariance_vx` | `0.03` |
| `twist.covariance[35]` | `odom_twist_covariance_wz` | `0.05` |

这些 covariance 是 mock / 仿真默认值，不来自真实底盘标定。未观测的 z、roll、pitch 相关自由度在发布时使用较大的 covariance，真实机器人应根据标定实验和传感器噪声估计修正。

Frame 约定当前保持一致：`chassis_driver_node` 发布 `odom -> base_footprint`，Nav2 basic 使用 `robot_base_frame: base_footprint`，URDF 提供 `base_footprint -> base_link` 固定关节。Phase 3B 不重构 TF 树。

标定流程见 `docs/calibration.md`，真实底盘标定结果仍为 TBD，不在本文档中伪造。

## Protocol v2 Remaining Plan

| Field / mechanism | Purpose | Status |
| --- | --- | --- |
| `seq` | 请求 / 响应关联，丢包检测 | Phase 3A current in text_v2 |
| `timestamp` | 延迟估计和日志对齐 | Phase 3A current in text_v2 |
| `heartbeat` | 通信存活检测 | Phase 3A current transmit / receive parsing |
| `fault_code` | 标准化底盘故障码 | Phase 3A current minimal enum |
| `timeout` | 命令超时停车策略 | Phase 3A current driver stop hook |
| `odom covariance` | EKF / localization 使用里程计不确定性 | Phase 3B current mock / simulation defaults |
| protocol version | 兼容多版本底盘控制器 | partial via `protocol:=text_v2` |

真实底盘联调后，需要补充串口波特率、UDP 端口、坐标系约定、速度单位、故障码表和安全超时策略。

## Phase 4B Safety Mapping

Phase 4B reads `/chassis/state` in `cmd_vel_safety_gate_node`. The message still
uses the existing `connected` field and the Phase 3A `status` string, so no
interface migration is required in this phase.

Mapping used by the safety gate:

| Chassis input | Safety state |
| --- | --- |
| `connected=false` | `COMMUNICATION_LOST` |
| `fault_code=NONE(...)` | `NORMAL` |
| `fault_code=CMD_TIMEOUT(...)` | `SENSOR_STALE` |
| `fault_code=HEARTBEAT_TIMEOUT(...)` | `COMMUNICATION_LOST` |
| `fault_code=BACKEND_DISCONNECTED(...)` | `COMMUNICATION_LOST` |
| `fault_code=MALFORMED_PACKET(...)` | `CHASSIS_FAULT` |
| `fault_code=ESTOP_ACTIVE(...)` or `estop=1` | `EMERGENCY_STOP` |

`CMD_TIMEOUT` is treated as `SENSOR_STALE` because the driver has already
detected a stale command stream and sent zero speed. Backend heartbeat and
disconnect faults are communication failures; malformed packets are treated as
chassis faults.

Static validation:

```bash
bash scripts/check_safety_state_machine.sh
```
