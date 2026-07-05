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

## Protocol v2 Plan

后续 Phase 3 建议定义协议 v2，不在 Phase 0 修改代码：

| Field / mechanism | Purpose | Status |
| --- | --- | --- |
| `seq` | 请求 / 响应关联，丢包检测 | planned |
| `timestamp` | 延迟估计和日志对齐 | planned |
| `heartbeat` | 通信存活检测 | planned |
| `fault_code` | 标准化底盘故障码 | planned |
| `timeout` | 命令超时停车策略 | planned |
| `odom covariance` | EKF / localization 使用里程计不确定性 | planned |
| protocol version | 兼容多版本底盘控制器 | planned |

真实底盘联调后，需要补充串口波特率、UDP 端口、坐标系约定、速度单位、故障码表和安全超时策略。
