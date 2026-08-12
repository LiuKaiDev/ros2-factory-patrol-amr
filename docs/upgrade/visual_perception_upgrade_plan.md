# ROS2 工厂巡检 AMR 自主导航与视觉感知系统
## 原项目升级规划书

> 基于现有仓库：`LiuKaiDev/ros2-factory-patrol-amr`
>
> 升级目标：在不改变“低速工厂巡检 AMR”主线的前提下，为现有导航—控制—安全闭环增加视觉感知能力，使机器人从“按照预设目标点运动”升级为“能够发现目标、估计目标空间位置、形成感知事件，并驱动任务或安全策略”的完整机器人系统。
>
> 核心原则：**主角始终是 AMR 机器人系统，视觉模型只是 Perception 的输入模块。**
>
> 建议项目名称：
>
> **ROS2 工厂巡检 AMR 自主导航与视觉感知系统**
>
> 英文可写：
>
> **ROS2 Factory Patrol AMR with Visual Perception, Navigation and Safety Integration**

---

# 1. 现有项目基础

当前仓库已经具备较完整的低速 AMR 软件链路，已有能力主要包括：

- ROS2 Jazzy workspace 与多 package 工程组织；
- Gazebo 工厂巡检仿真场景；
- RViz 可视化与调试；
- Nav2 / AMCL 导航定位链路；
- 全局规划与局部控制；
- `/cmd_vel` 多来源速度仲裁；
- Safety Gate；
- 底盘适配与 odom / TF 反馈；
- Localization Health；
- System Monitor / Fault Supervisor；
- 多点巡检任务入口；
- 临时障碍、定位恢复等 demo / validation 基础；
- 脚本化启动、检查和测试体系。

现有主闭环可以概括为：

```text
巡检任务 / 目标点
        ↓
      Nav2
        ↓
规划 / 控制器
        ↓
 cmd_vel mux
        ↓
   Safety Gate
        ↓
    Chassis
        ↓
  Odom / TF / State
        ↓
定位与系统健康反馈
```

这个项目目前主要体现：

> **机器人导航、控制、安全和软件系统工程能力。**

---

# 2. 为什么要升级视觉感知

当前系统的主要任务目标来自：

- 固定巡检点；
- 任务配置；
- 人工输入；
- 已知地图与站点。

机器人虽然能够“移动”，但缺少一层主动环境理解能力。

升级视觉感知后，机器人应该能够回答：

1. **我看到了什么？**
2. **这个目标在哪里？**
3. **它是不是可信目标？**
4. **它是否已经被处理过？**
5. **它需要触发什么机器人行为？**

因此升级后的核心闭环变为：

```text
视觉感知
   ↓
空间定位
   ↓
目标管理 / 事件判断
   ↓
任务决策 / Safety
   ↓
Nav2 / 控制
   ↓
机器人执行
   ↓
状态反馈
```

项目由原来的：

> **Navigation-Control Closed Loop**

升级为：

> **Perception-Decision-Navigation-Control Closed Loop**

---

# 3. 升级后的核心定位

## 3.1 项目不是视觉算法项目

本项目不以以下内容作为主要贡献：

- 改 YOLO Backbone；
- 修改 RT-DETR Decoder；
- 新设计 Loss；
- 追求检测 mAP 的微小提升；
- 做大规模数据集训练；
- 比较大量检测模型。

这些内容属于“视觉算法简历”的范畴。

---

## 3.2 项目真正重点

升级后的核心重点应放在：

### Perception

```text
RGB-D Camera
    ↓
Object Detector
    ↓
2D Detection
```

### Geometry

```text
2D bbox
    +
Depth
    +
Camera Intrinsics
    ↓
3D Point in Camera Frame
```

### Robot Coordinate System

```text
camera_optical_frame
        ↓
       TF2
        ↓
base_link
        ↓
       TF2
        ↓
map
```

### Target Management

```text
Detection
   ↓
Confidence Filtering
   ↓
Depth Validation
   ↓
Temporal Filtering
   ↓
Target Association
   ↓
Deduplication
```

### Robot Behavior

```text
Target / Event
     ↓
Mission Policy
     ↓
Approach / Record / Slow / Stop
     ↓
Nav2 / Safety Gate
```

---

# 4. 升级后的最终项目故事

最终项目应该能够被一句话解释清楚：

> 一台 ROS2 工厂巡检 AMR 在 Gazebo 厂区中执行多点巡检任务，通过 RGB-D 相机和目标检测模型识别人员或巡检目标，根据深度与相机内参恢复目标三维位置，通过 TF2 将目标转换到 map 坐标系，并经过目标去重、稳定性过滤和事件策略后驱动 Nav2 靠近巡检目标，或将人员进入危险区域事件接入 Safety Gate，实现感知—定位—决策—导航—控制闭环。

---

# 5. 最终 Demo 设计

建议最终至少完成 4 个 Demo。

---

## Demo 1：目标检测与三维定位

### 场景

AMR 静止或低速运动，相机观察目标。

### 流程

```text
RGB-D Camera
    ↓
Object Detector
    ↓
bbox + class + confidence
    ↓
Depth Sampling
    ↓
3D Position in camera frame
    ↓
TF2
    ↓
3D Position in map frame
    ↓
RViz Marker
```

### 最终展示

RViz 中同时显示：

- Robot；
- TF Tree；
- Camera；
- Detected Object Marker；
- Object Label；
- Object map 坐标。

### 验收重点

不是“模型检测到了”。

而是：

> **目标的机器人空间坐标正确。**

---

# 6. Demo 2：视觉引导巡检 / 靠近目标

### 场景

巡检过程中识别到一个需要复检的目标，例如：

- inspection box；
- equipment marker；
- fire extinguisher；
- abnormal object。

### 流程

```text
检测目标
   ↓
目标 map 坐标
   ↓
目标稳定确认
   ↓
计算安全观察点
   ↓
发送 Nav2 Goal
   ↓
AMR 导航至目标附近
   ↓
到达后停车
   ↓
记录 Inspection Event
```

### 注意

机器人不应该直接导航到物体中心。

需要计算：

```text
observation_pose
```

例如保持：

```text
1.0 ~ 1.5 m
```

观察距离。

---

# 7. Demo 3：人员危险区域安全联动

这是整个升级中非常重要的企业系统感展示。

### 场景

工厂场景中定义一个：

```text
danger_zone
```

AMR 检测到 person。

### 流程

```text
Person Detection
       ↓
Person 3D Position
       ↓
map Coordinate
       ↓
Zone Membership Check
       ↓
Person inside danger zone?
       ↓
Perception Safety Event
       ↓
Safety Gate
       ↓
SPEED_LIMITED / STOP
```

### 可设计策略

距离 > 3 m：

```text
NORMAL
```

距离 1.5 ~ 3 m：

```text
SPEED_LIMITED
```

距离 < 1.5 m：

```text
STOP
```

或基于地图危险区：

```text
person ∈ danger_zone
    → STOP
```

---

# 8. Demo 4：感知异常与恢复

需要证明项目不只是 Happy Path。

模拟：

- Detector 没有结果；
- RGB 图像断流；
- Depth 断流；
- 深度为 NaN / 0；
- 检测低置信度；
- TF lookup 失败；
- 目标短暂消失；
- 视觉 Node 超时。

系统要求：

- 不能崩溃；
- 不输出错误导航目标；
- Safety 策略明确；
- 发布 diagnostics；
- 恢复后继续运行。

---

# 9. 视觉算法选择

## 9.1 第一版建议

选择一个成熟检测器即可：

```text
YOLO family
或
RT-DETR
```

项目不依赖某个具体检测模型。

应该设计统一接口：

```text
DetectorBackend
```

例如：

```cpp
class DetectorBackend {
public:
    virtual std::vector<Detection2D> infer(
        const cv::Mat& image) = 0;
};
```

后续模型可以替换，而下游：

- Depth；
- TF；
- Tracking；
- Task；
- Safety；

无需修改。

---

## 9.2 推荐开发顺序

### Stage A：几何链路验证

先不用真实 detector。

可以使用：

- AprilTag；
- Synthetic Ground Truth Detection；
- 固定 bbox 测试。

目的：

> **先证明 Camera → Depth → 3D → TF 链路正确。**

---

### Stage B：真实检测器

接入：

```text
YOLO / RT-DETR
```

第一版模型不要求自行改结构。

---

### Stage C：部署优化（可选）

根据环境选择：

```text
ONNX Runtime
TensorRT
OpenCV DNN
```

如果 GPU 环境复杂，不允许部署优化阻塞主项目。

---

# 10. 为什么建议 RGB-D

普通 RGB 检测只能得到：

```text
(u, v)
```

无法直接获得目标距离。

RGB-D Camera 可以同时获得：

```text
RGB Image
Depth Image
CameraInfo
```

检测 bbox 中心：

```text
(u, v)
```

深度：

```text
Z
```

根据相机模型：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

得到：

```text
P_camera = [X, Y, Z]
```

再使用 TF2 转换到：

```text
map
```

这是本项目最关键的机器人视觉链路之一。

---

# 11. Depth 不能简单取 bbox 中心一个像素

为了避免项目过于“Demo 化”，Depth 模块需要处理：

- 0 depth；
- NaN；
- Inf；
- 背景深度；
- 物体边缘；
- bbox 中心落在空洞区域。

推荐策略：

### V1

取 bbox 中央 ROI：

```text
center 20% ~ 40%
```

过滤非法深度后：

```text
median(depth)
```

而不是单像素。

### V2

可以使用 segmentation mask 或 bbox 内鲁棒统计。

---

# 12. 坐标系设计

建议至少包含：

```text
map
 ↓
odom
 ↓
base_link
 ↓
camera_link
 ↓
camera_color_optical_frame
```

感知节点输出必须明确：

```text
frame_id
timestamp
```

目标点转换：

```text
camera optical frame
        ↓
      base_link
        ↓
       map
```

所有 TF 查询必须使用对应图像 timestamp。

禁止默认直接使用：

```text
latest TF
```

来掩盖时间同步问题。

---

# 13. 时间同步

RGB、Depth、CameraInfo 必须进行同步。

推荐：

```text
message_filters
```

使用：

```text
ApproximateTime
```

或严格时间同步。

需要处理：

- RGB 比 Depth 快；
- Depth 延迟；
- CameraInfo 时间；
- TF 时间不一致。

这是非常适合面试讨论的工程问题。

---

# 14. 目标管理

单帧检测不能直接触发机器人行为。

必须增加：

```text
Target Manager
```

核心功能：

- 多帧确认；
- 置信度过滤；
- 空间距离匹配；
- target_id；
- 生命周期；
- target lost；
- duplicate suppression；
- cooldown。

例如：

```text
连续 3 / 5 帧检测到
+
map position variance < threshold
    ↓
CONFIRMED
```

状态：

```text
TENTATIVE
CONFIRMED
LOST
PROCESSED
```

---

# 15. 目标位置稳定

检测框和 depth 会抖动。

需要做基础过滤。

第一版：

```text
Moving Average
```

或：

```text
EMA
```

例如：

```text
p_filtered =
alpha * p_new
+
(1 - alpha) * p_previous
```

后期可扩展：

```text
Kalman Filter
```

但本项目不需要将 Target Tracking 做成第二个状态估计项目。

---

# 16. 项目 Package 升级规划

保留现有 package，不破坏原架构。

新增：

```text
src/
  robot_perception/
```

建议职责划分：

```text
robot_perception/
├── include/
│   └── robot_perception/
│       ├── detector_backend.hpp
│       ├── depth_projector.hpp
│       ├── target_manager.hpp
│       ├── zone_checker.hpp
│       └── perception_types.hpp
│
├── src/
│   ├── perception_node.cpp
│   ├── detector_backend.cpp
│   ├── depth_projector.cpp
│   ├── target_manager.cpp
│   └── zone_checker.cpp
│
├── config/
│   ├── detector.yaml
│   ├── depth.yaml
│   ├── tracking.yaml
│   └── safety_policy.yaml
│
├── launch/
│   └── perception.launch.py
│
└── test/
```

---

# 17. 对现有 package 的修改

## robot_description

增加：

- RGB-D Camera link；
- optical frame；
- camera sensor model；
- camera TF。

---

## robot_simulation

增加：

- RGB-D camera Gazebo sensor；
- inspection objects；
- person / hazard test objects；
- perception demo worlds；
- visual target markers。

---

## robot_perception

新增核心视觉感知链。

负责：

```text
Image
→ Detection
→ Depth
→ 3D
→ TF
→ Target Manager
→ Event
```

---

## robot_tasks

新增：

```text
Perception Event Consumer
```

例如：

```text
INSPECTION_TARGET_FOUND
```

转换为：

```text
Approach Target Mission
```

---

## robot_teleop / Safety Gate

不要让 Perception 直接发布 `/cmd_vel`。

视觉安全事件应该成为：

```text
Safety Input
```

最终仍经过现有：

```text
cmd_vel_safety_gate
```

统一处理。

---

## robot_utils

增加：

```text
perception health
camera health
detector health
```

并接入 system monitor。

---

## robot_experiments

增加：

- perception latency；
- target localization error；
- detection-to-action latency；
- safety response time；
- mission success；
- false trigger count。

---

# 18. 推荐 ROS2 Topic

输入：

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
```

中间结果：

```text
/perception/detections_2d
/perception/objects_3d
/perception/markers
/perception/diagnostics
```

行为事件：

```text
/perception/events
/perception/safety_event
```

调试：

```text
/perception/debug_image
```

---

# 19. 推荐 Interface

可新增：

```text
DetectedObject3D.msg
```

示例：

```text
std_msgs/Header header

uint32 target_id

string class_name

float32 confidence

geometry_msgs/Point position

geometry_msgs/Vector3 size

bool depth_valid

uint8 tracking_state
```

---

## PerceptionEvent.msg

```text
std_msgs/Header header

uint32 target_id

string event_type

string class_name

geometry_msgs/PoseStamped target_pose

float32 confidence

uint8 severity
```

event_type 示例：

```text
TARGET_FOUND
TARGET_CONFIRMED
TARGET_LOST
INSPECTION_REQUIRED
PERSON_IN_DANGER_ZONE
```

---

# 20. 参数配置

示例：

```yaml
detector:
  backend: yolo
  confidence_threshold: 0.5
  input_width: 640
  input_height: 640

depth:
  min_depth: 0.2
  max_depth: 8.0
  roi_ratio: 0.3
  statistic: median

tracking:
  confirm_frames: 3
  lost_frames: 5
  max_match_distance: 0.5
  ema_alpha: 0.4
  processed_cooldown_sec: 10.0

safety:
  person_slow_distance: 3.0
  person_stop_distance: 1.5
```

---

# 21. 开发阶段总览

```text
Phase 0
冻结当前 AMR Baseline

↓

Phase 1
RGB-D Camera + TF

↓

Phase 2
Depth / 3D / TF 几何链路

↓

Phase 3
Detector Integration

↓

Phase 4
Target Manager / Filtering

↓

Phase 5
视觉引导任务

↓

Phase 6
视觉 Safety Integration

↓

Phase 7
Fault Handling / Diagnostics

↓

Phase 8
Evaluation / Benchmark

↓

Phase 9
README / Demo / Resume
```

---

# 22. Phase 0：冻结原项目 Baseline

## 目标

升级之前先证明：

```text
原 AMR 功能正常
```

记录：

- commit hash；
- colcon test 结果；
- factory patrol demo 启动方法；
- Nav2 / Safety / odom / TF 状态。

### 原则

升级不能破坏：

- Nav2；
- cmd_vel mux；
- safety gate；
- chassis；
- odom；
- localization；
- mission runner。

### 验收

原有 runtime validation 全部通过。

---

# 23. Phase 1：RGB-D Camera 与机器人模型

实现：

- camera_link；
- camera optical frame；
- RGB sensor；
- depth sensor；
- CameraInfo；
- ROS2 bridge；
- RViz image display。

### 验收

可正常：

```bash
ros2 topic echo /camera/color/camera_info
```

并确认：

```text
RGB
Depth
CameraInfo
TF
```

全部存在。

---

# 24. Phase 2：3D 几何链路

先不加入神经网络模型。

创建测试目标。

实现：

```text
pixel
+
depth
+
intrinsics
    ↓
Point Camera
    ↓
TF
    ↓
Point Map
```

推荐使用：

- AprilTag；
- synthetic bbox；
- known landmark。

### 为什么

如果一开始直接上 YOLO：

一旦位置错了，很难知道问题来自：

- Detector；
- Depth；
- Intrinsics；
- Optical Frame；
- TF；
- Timestamp。

所以必须先单独验证几何链。

### 验收

已知物体 Ground Truth：

```text
P_gt
```

估计：

```text
P_est
```

计算：

```text
3D localization error
```

---

# 25. Phase 3：目标检测模型

接入：

```text
YOLO
或
RT-DETR
```

要求：

- Detector 与机器人下游解耦；
- 输出统一 Detection2D；
- 支持置信度过滤；
- 输出 latency；
- 支持模型加载失败检测。

### 验收

Gazebo 场景中可以稳定检测至少：

```text
2 ~ 3 类目标
```

不要求复杂数据集。

---

# 26. Phase 4：目标管理与稳定性

实现：

- target association；
- position smoothing；
- confirm；
- lost；
- processed；
- duplicate suppression。

### 验收

目标连续被看到时：

```text
target_id 稳定
```

短暂漏检：

```text
不会立即产生新目标
```

同一目标：

```text
不会连续触发多个任务
```

---

# 27. Phase 5：视觉引导任务

加入：

```text
VisualInspectionTask
```

流程：

```text
Confirmed Target
    ↓
Target map pose
    ↓
Observation Pose Planner
    ↓
Nav2 Goal
    ↓
Navigate
    ↓
Inspection Complete
```

### Observation Pose

不能直接使用 target pose。

需要：

```text
target position
-
safe standoff distance
```

计算机器人观察位姿。

### 验收

机器人检测目标后：

- 自动生成观察点；
- Nav2 接收目标；
- 到达目标附近；
- 不碰撞目标；
- 发布完成事件。

---

# 28. Phase 6：视觉 Safety Integration

视觉不能直接修改速度。

使用：

```text
/perception/safety_event
```

接入现有 Safety Gate。

例如：

```text
PERSON_NEAR
→ SPEED_LIMITED
```

```text
PERSON_TOO_CLOSE
→ STOP
```

```text
PERSON_IN_DANGER_ZONE
→ STOP
```

### 验收

Nav2 正在运动时出现 person：

```text
Nav2 cmd_vel
    ↓
Safety Gate
    ↓
0 / Limited cmd_vel
```

person 离开并满足恢复策略：

```text
恢复运行
```

---

# 29. Phase 7：异常与诊断

实现：

### Camera Health

检测：

- RGB stale；
- Depth stale；
- CameraInfo missing。

### Detector Health

检测：

- model load failure；
- inference timeout；
- repeated error。

### TF Health

检测：

- missing transform；
- transform timeout；
- stale transform。

### Depth Health

检测：

- invalid ratio；
- missing depth；
- excessive noise。

发布：

```text
/perception/diagnostics
```

并接入：

```text
system_monitor
```

---

# 30. Phase 8：Evaluation

这是项目从“Demo”变成“工程项目”的关键。

至少测：

## 30.1 Detection Latency

```text
image timestamp
→ detection output
```

指标：

- Average；
- P50；
- P95。

---

## 30.2 3D Localization Error

Gazebo 中获得目标 Ground Truth：

```text
P_gt
```

感知：

```text
P_est
```

计算：

```text
error = ||P_est - P_gt||
```

---

## 30.3 Position Stability

机器人静止时：

记录连续目标位置。

计算：

- x std；
- y std；
- z std。

比较：

```text
raw
vs
filtered
```

---

## 30.4 Detection-to-Action Latency

```text
Target detected
→ Event confirmed
→ Nav2 goal / safety action
```

---

## 30.5 Safety Response Time

```text
person enters danger condition
→ final gated cmd_vel becomes zero
```

---

## 30.6 Mission Success

多轮运行：

```text
target found
→ approach
→ arrive
→ complete
```

统计成功次数。

注意：

**没有真实跑实验之前，不允许在 README 或简历虚构成功率。**

---

# 31. Phase 9：最终展示

README 首页建议展示：

1. 项目一句话介绍；
2. 系统架构图；
3. Demo GIF；
4. Factory Patrol Screenshot；
5. Perception Pipeline；
6. Target 3D Localization；
7. Nav2 Approach Demo；
8. Safety Stop Demo；
9. Performance Table；
10. Package Architecture；
11. Quick Start；
12. Known Limitations。

---

# 32. 推荐最终架构图

```text
                        ┌───────────────────┐
                        │   RGB-D Camera    │
                        └─────────┬─────────┘
                                  │
                RGB / Depth / CameraInfo
                                  │
                                  ▼
                        ┌───────────────────┐
                        │ robot_perception  │
                        │                   │
                        │ Detector          │
                        │ Depth Projection  │
                        │ TF Transform      │
                        │ Target Manager    │
                        └──────┬───────┬────┘
                               │       │
                         Object/Event  Safety Event
                               │       │
                               ▼       ▼
                      ┌────────────┐  ┌────────────┐
                      │robot_tasks │  │Safety Gate │
                      └─────┬──────┘  └──────┬─────┘
                            │                │
                         Nav2 Goal           │
                            │                │
                            ▼                │
                      ┌─────────────┐         │
                      │    Nav2     │         │
                      └──────┬──────┘         │
                             │ cmd_vel        │
                             ▼                │
                       cmd_vel mux            │
                             │                │
                             └───────┬────────┘
                                     ▼
                               Final cmd_vel
                                     │
                                     ▼
                                  Chassis
                                     │
                                     ▼
                               Odom / TF / State
```

---

# 33. 测试要求

## Unit Test

至少覆盖：

- depth median；
- invalid depth；
- pixel-to-camera projection；
- coordinate validation；
- target association；
- confirmation state；
- target lost；
- duplicate suppression；
- zone membership；
- safety distance decision。

---

## Integration Test

### Test 1

```text
Camera → Detector
```

### Test 2

```text
Detector → 3D → TF
```

### Test 3

```text
Target → Mission
```

### Test 4

```text
Person → Safety
```

### Test 5

```text
Camera failure → diagnostics
```

---

# 34. 工程质量要求

建议维持现有项目工程风格：

- ROS2 Jazzy；
- C++17；
- Python 仅用于模型或工具时使用；
- CMake / colcon；
- YAML config；
- launch；
- RViz；
- Gazebo；
- gtest；
- clang-format；
- CI；
- validation scripts；
- structured logs；
- documentation。

---

# 35. C++ 与 Python 的边界建议

如果模型部署时间有限：

## V1

Detector：

```text
Python
```

机器人系统：

```text
C++
```

这样先快速跑通闭环。

## V2

再升级 Detector：

```text
C++ + ONNX Runtime / TensorRT
```

项目价值不会因为第一版模型节点使用 Python 就消失。

但最终如果目标是：

> C++ / ROS2 机器人软件岗

建议有时间时完成 C++ inference backend。

---

# 36. 不允许视觉直接控制机器人

这是重要架构原则。

错误设计：

```text
YOLO detects person
    ↓
publish /cmd_vel = 0
```

正确设计：

```text
YOLO
 ↓
Perception Event
 ↓
Safety Policy
 ↓
Safety Gate
 ↓
Final cmd_vel
```

原因：

- 控制权统一；
- 安全状态可解释；
- 易调试；
- 易恢复；
- 避免多个节点抢 `/cmd_vel`。

---

# 37. 避免项目跑偏的边界

第一版不做：

- 自研 Detector；
- 大规模训练；
- 模型结构创新；
- 6D Pose；
- Visual SLAM；
- VIO；
- 3D Point Cloud Detector；
- 多摄像头；
- Foundation Model；
- VLM；
- Grasping；
- Reinforcement Learning；
- Semantic SLAM；
- 自研 Nav2 Planner；
- 自研 Controller。

后续这些都可以扩展，但不能影响主线闭环完成。

---

# 38. 必须完成 / 可选功能

## Must Have

- [ ] RGB-D Camera
- [ ] Camera TF
- [ ] RGB / Depth / CameraInfo
- [ ] Detector
- [ ] Detection2D
- [ ] Robust Depth Sampling
- [ ] 3D Projection
- [ ] TF2 → map
- [ ] RViz Marker
- [ ] Target Manager
- [ ] Duplicate Suppression
- [ ] Visual Inspection Event
- [ ] Nav2 Approach
- [ ] Person Safety Event
- [ ] Safety Gate Integration
- [ ] Camera / Detector Diagnostics
- [ ] Performance Evaluation
- [ ] Demo
- [ ] README

## Should Have

- [ ] approximate time synchronization
- [ ] target smoothing
- [ ] target lost state
- [ ] cooldown
- [ ] observation pose calculation
- [ ] detector latency statistics
- [ ] 3D localization error
- [ ] safety response latency

## Nice to Have

- [ ] C++ TensorRT backend
- [ ] semantic segmentation
- [ ] danger-zone segmentation
- [ ] object tracking algorithm
- [ ] recording inspection snapshots
- [ ] rosbag replay
- [ ] Docker GPU environment

---

# 39. 最终简历能力分工

升级后的 AMR 项目主要体现：

## 机器人系统

- ROS2；
- Nav2；
- AMCL；
- TF2；
- Task；
- Safety；
- Chassis；
- Diagnostics。

## 视觉感知

- Object Detection；
- RGB-D；
- Camera Model；
- 3D Localization。

## 工程能力

- 多节点架构；
- 异步数据；
- 时间同步；
- 状态管理；
- 故障处理；
- 参数化；
- 测试；
- Benchmark。

## 系统集成

```text
Perception
→ Decision
→ Navigation
→ Control
→ Safety
```

---

# 40. 与第二个“多传感器融合定位”项目的分工

## 项目一：Factory Patrol AMR + Visual Perception

回答：

> **我能不能搭建一个完整的机器人系统？**

重点：

```text
系统广度
+
感知闭环
+
工程集成
```

---

## 项目二：Multi-Sensor Fusion Localization

回答：

> **我是否理解机器人底层状态估计？**

重点：

```text
EKF
+
IMU
+
Wheel Odom
+
Noise
+
Covariance
+
Robust State Estimation
```

---

两个项目不要互相吞掉。

本项目不深入 EKF。

第二项目不做复杂视觉任务闭环。

---

# 41. 面试中必须能回答的问题

完成项目后至少能够独立回答：

### Camera

- RGB 与 Depth 如何同步？
- CameraInfo 有什么？
- fx、fy、cx、cy 是什么？
- optical frame 与 camera_link 有什么区别？

### 3D Geometry

- bbox 怎么恢复三维坐标？
- 为什么不能只取中心 depth？
- 为什么目标坐标会抖？
- invalid depth 怎么处理？

### TF

- camera → base_link → map 如何转换？
- 为什么 timestamp 很重要？
- TF lookup 失败怎么办？

### Perception

- 为什么选择 YOLO / RT-DETR？
- 为什么不自己改模型？
- 模型延迟怎么测？
- detection confidence 如何设置？

### Target Management

- 为什么单帧检测不能直接触发任务？
- 如何处理连续重复检测？
- 目标消失怎么办？
- 如何判断两个检测是同一个物体？

### Navigation

- 为什么不能直接导航到目标坐标？
- observation pose 怎么算？
- 目标在机器人运动中变化怎么办？

### Safety

- 为什么 perception 不直接发 `/cmd_vel`？
- safety event 怎么接入已有 Safety Gate？
- person 消失后什么时候恢复？

### Engineering

- Camera Node 挂了怎么办？
- 模型推理超时怎么办？
- 如何测试整个 perception pipeline？
- 如何定量评估 3D localization？

---

# 42. 后续给 Codex 的开发原则

不要一次性让 Codex 重构整个项目。

每个 Phase 都应该：

1. 读取当前仓库；
2. 识别现有 package / interface；
3. 说明本阶段为什么这样设计；
4. 列出修改文件；
5. 明确 Topic / TF / Parameter；
6. 保持现有接口兼容；
7. 实现代码；
8. `colcon build`；
9. `colcon test`；
10. 运行本阶段 smoke test；
11. 更新 docs；
12. 不提前实现下一阶段。

---

# 43. 给 Codex 时必须强调的限制

每阶段 Prompt 建议固定包含：

```text
Do not rewrite existing architecture unless necessary.

Do not bypass cmd_vel mux or safety gate.

Do not publish robot velocity directly from perception.

Do not replace Nav2.

Do not introduce SLAM/VIO/3D detection unless explicitly requested.

Do not fabricate runtime results.

Do not claim a feature has passed unless it has actually been executed.

Keep perception modular and replaceable.

Preserve existing validation scripts and tests.
```

---

# 44. 推荐 Codex 实施顺序

```text
Phase 0
Scan + Baseline Freeze

↓

Phase 1
RGB-D Camera + TF

↓

Phase 2
Depth Projection + Geometry Validation

↓

Phase 3
Detector Backend

↓

Phase 4
Target Manager

↓

Phase 5
Visual Inspection Mission

↓

Phase 6
Safety Integration

↓

Phase 7
Diagnostics + Fault Handling

↓

Phase 8
Benchmark + Evaluation

↓

Phase 9
README + Demo + Resume
```

---

# 45. 每个阶段的判断原则

每一个功能都问：

> **“它有没有让机器人更加自主，或者让系统更加可靠？”**

如果答案是：

```text
Yes
```

优先保留。

如果只是：

```text
模型 mAP 多提高 1%
```

且不会改变机器人系统能力：

```text
放到视觉简历项目，不要占用本项目时间。
```

---

# 46. 最终完成标准

只有当以下链路真实跑通，升级才算完成：

```text
Gazebo Factory Patrol
        ↓
RGB-D Camera
        ↓
Object Detection
        ↓
Robust Depth
        ↓
3D Position
        ↓
TF2 map Position
        ↓
Target Manager
        ↓
Perception Event
       / \
      /   \
Mission   Safety
  ↓         ↓
Nav2    Safety Gate
  \         /
   \       /
    Robot Motion
```

同时：

- 原导航功能不退化；
- 原 Safety 链路不破坏；
- 感知故障不会产生危险控制；
- 有定量实验；
- 有可复现 Demo；
- 有测试；
- 有 README。

---

# 47. 项目最终一句话描述

> 基于 ROS2 Jazzy 构建工厂巡检 AMR 自主导航与视觉感知系统，在原有 Nav2、AMCL、速度仲裁、Safety Gate 和底盘反馈闭环基础上集成 RGB-D 视觉感知，通过目标检测、深度恢复和 TF2 实现目标三维定位，并设计目标管理、视觉引导巡检和人员安全事件机制，使视觉结果能够驱动任务与安全策略，形成感知—决策—导航—控制闭环。

---

# 48. 简历未来可重点量化的指标

项目真实完成后，可以从实际实验中选取：

- Detector P95 latency；
- 3D target localization mean error；
- 3D target localization RMSE；
- filtered target position std；
- detection-to-action P95 latency；
- safety response latency；
- inspection mission success rate；
- false task trigger count；
- invalid depth rejection rate。

**所有数字必须来自真实实验，不提前编写。**

---

# 49. 当前优先级建议

从投入产出比看，建议严格按以下优先级：

```text
P0：
RGB-D + 3D + TF

P1：
Detector + Target Manager

P2：
Nav2 Visual Approach

P3：
Safety Integration

P4：
Diagnostics + Evaluation

P5：
C++ inference optimization

P6：
Semantic Segmentation / Advanced Perception
```

如果秋招时间紧：

做到 P0 ~ P4 已经足够形成完整项目。

---

# 50. 结论

本次升级不是把原 AMR 项目改造成一个“视觉算法项目”。

真正的升级是：

```text
原项目：
Goal → Navigation → Control

升级后：
Perception → Spatial Understanding → Decision
           → Navigation → Control → Safety
```

因此升级后的项目核心仍然是：

> **机器人系统工程。**

视觉承担的是：

> **让机器人获得环境理解入口。**

最终最有价值的能力不是：

> “我会 YOLO。”

而是：

> **“我知道如何把一个视觉模型以可靠、可解释、可测试的方式接入 ROS2 机器人系统，并让感知结果真正参与任务和安全闭环。”**
