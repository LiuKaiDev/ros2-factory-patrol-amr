# 仿真场景

当前仓库同时包含通用室内仿真和面向“厂区 / 园区半封闭低速巡检”的 Factory Patrol
专用场景、配置、launch 与验证脚本。各项 runtime 结论仍以对应的真实运行记录为准。

## 当前仿真基础

| Asset / module | Status | Notes |
| --- | --- | --- |
| `src/robot_simulation/worlds/indoor_room.sdf` | 已实现 | Gazebo 室内场景。 |
| `src/robot_navigation/maps/indoor_room.yaml` | 已实现 | 静态地图。 |
| `src/robot_simulation/config/amr_sim_stations.yaml` | 已实现 | 仿真站点配置。 |
| `src/robot_simulation/config/amr_sim_map_zones.yaml` | 已实现 | 仿真 map zones 配置。 |
| `amr_sim_visualizer_node` | 已实现 | RViz marker 可视化。 |
| `amr_sim_demo_director_node` | 已实现 | 自动演示编排器。 |
| Gazebo entity bridge | 已实现 | 将部分业务 / 安全状态映射到 Gazebo 实体。 |

## Factory Patrol 资产状态

| Asset | Status | Purpose |
| --- | --- | --- |
| `factory_patrol.sdf` | 主场景 | 厂区 / 园区巡检 Gazebo world。 |
| `factory_patrol_industrial.sdf` | 可选预览 | 独立的工业布局预览 world。 |
| Factory Patrol occupancy map | 未提交 | 仓库保留 map 生成说明，不伪造 occupancy map。 |
| patrol stations / route | 已配置 | 巡检点、充电点、等待点与路线 seed。 |
| patrol zones | 已配置 | 限速区、禁行区、临时障碍区配置；不等同于已接入 Nav2 filter。 |
| scenario launch | 已实现 | `factory_patrol_demo.launch.py` 与配套 Demo script。 |

## Demo 设计

### 1. Multi-point Patrol

状态：workflow、mission profile 和脚本已存在；逐 waypoint runtime 结果仍需按具体运行记录验收。

目标：

- 从起点出发；
- 依次到达多个巡检点；
- 每个点停留或发布到达状态；
- 完成后返回等待点或充电点。

Factory scene、巡检点、route 配置和 multipoint workflow 均已提供；success rate 与 travel
time 需要由具体运行记录支持。

### 2. Temporary Obstacle Avoidance

状态：temporary obstacle config、model 和脚本已存在；实际避障行为仍需 runtime 验收。

目标：

- 在巡检路径上放置临时障碍；
- 验证 local costmap 感知、清障和绕行；
- 记录是否停车、是否重新规划、到达时间和 stop_count。

该 workflow 尚未提交统一条件下的成功率或避障性能结果。

### 3. Localization Lost and Recovery

状态：定位逻辑、recovery 配置和 Demo 入口已存在；状态转换仍需 runtime log 证明。

目标：

- 模拟 AMCL covariance 超阈值；
- localization health 进入 `LOST`；
- 任务暂停并请求重定位；
- 设置初始位姿后进入 `RECOVERED`；
- 任务恢复。

当前代码和检查脚本已有定位健康与重定位入口；具体状态转换以 runtime log 为准。

## Factory Patrol 仿真资产

Factory Patrol 资产面向低速半封闭 AMR 巡检，包含 world、语义配置、launch、RViz 和验证
入口。是否启用 Nav2 取决于启动参数和可用地图。

当前结构扫描结果：

| Area | Existing path |
| --- | --- |
| Gazebo world | `src/robot_simulation/worlds/indoor_room.sdf` |
| Maps | `src/robot_navigation/maps/indoor_room.yaml`, `src/robot_navigation/maps/indoor_room.pgm` |
| RViz | `src/robot_simulation/rviz/amr_sim.rviz`, `src/robot_simulation/rviz/nav2_basic_debug.rviz` |
| Simulation launch | `src/robot_simulation/launch/sim.launch.py` |
| Navigation launch | `src/robot_navigation/launch/nav.launch.py`, `src/robot_navigation/launch/navigation_no_docking.launch.py` |
| Bringup launch | `src/robot_bringup/launch/amr_demo.launch.py`, `src/robot_bringup/launch/bringup.launch.py` |
| Mission launch | `src/robot_tasks/launch/mission_runner.launch.py` |
| Existing sim stations/zones | `src/robot_simulation/config/amr_sim_stations.yaml`, `src/robot_simulation/config/amr_sim_map_zones.yaml` |
| Existing task station config | `src/robot_tasks/config/stations/warehouse_stations.yaml` |

Factory Patrol 专用资产：

| Asset | Path | Status |
| --- | --- | --- |
| Factory world | `src/robot_simulation/worlds/factory_patrol.sdf` | 主场景 |
| Factory Scene V2 world | `src/robot_simulation/worlds/factory_patrol_industrial.sdf` | optional preview asset |
| Station seed config | `src/robot_simulation/config/factory_patrol_stations.yaml` | 已配置 |
| Zone seed config | `src/robot_simulation/config/factory_patrol_zones.yaml` | 已配置 |
| Patrol route config | `src/robot_simulation/config/factory_patrol_route.yaml` | 路线 seed；未直接接入 mission runner |
| Map note | `src/robot_navigation/maps/factory_patrol_map_generation.md` | 地图生成说明 |
| Demo launch | `src/robot_bringup/launch/factory_patrol_demo.launch.py` | 已实现 |
| Demo helper | `scripts/run_factory_patrol_demo.sh` | 已实现 |
| Static check | `scripts/check_factory_patrol_assets.sh` | 已实现 |

`factory_patrol.sdf` contains:

- 16 m x 12 m factory floor with widened AMR aisles
- orthogonal dock -> receiving -> storage -> packing -> dock inspection route
- main patrol road
- equipment inspection area
- narrow corridor marker
- turning area
- station markers for `station_A`, `station_B`, `station_C`, and `dock`
- static obstacles using simple boxes/cylinders
- slow-zone visual overlay
- no-go visual overlay

Station 配置：

`src/robot_simulation/config/factory_patrol_stations.yaml` defines `start`,
`station_A`, `station_B`, `station_C`, and `dock` in the `map` frame. Coordinates
这些是与 SDF layout 对齐的 simulation seed pose，不是现场测量 pose。

Zone 配置：

`src/robot_simulation/config/factory_patrol_zones.yaml` defines
`factory_slow_corridor`, `factory_no_go_equipment_service_area`, and
`factory_turning_caution_area`。这些是语义配置资产，尚未接入 Nav2 costmap filter。

Route 配置：

`src/robot_simulation/config/factory_patrol_route.yaml` defines
`factory_patrol_loop`:

```text
start -> station_A -> station_B -> station_C -> dock
```

当前 mission runner 不直接执行此 YAML；它作为路线 seed 保存。未来可转换为 package 已有的
mission format，具体工作见 [项目路线图](roadmap.md)。

Map 处理：

仓库未提交 Factory Patrol occupancy map。地图生成流程见
`src/robot_navigation/maps/factory_patrol_map_generation.md`；在提供经过审阅的地图前，Factory
Demo launch 默认保持 Nav2 disabled。

Demo 入口：

```bash
bash scripts/run_factory_patrol_demo.sh
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true
```

提供明确的 Factory map 后，如需实验 Nav2：

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  use_nav2:=true nav2_map:=/absolute/path/to/factory_patrol.yaml
```

RViz 视图：

当 `use_rviz:=true` 时，Factory Patrol Demo launch 默认使用
`src/robot_simulation/rviz/factory_patrol_showcase.rviz`。这是 non-Nav2 showcase view，
包含 `odom` fixed frame、RobotModel、Lidar Scan、Odometry、Odom Path 和 Factory Semantics
marker。为保持截图简洁，Factory Semantics 默认关闭；检查 runtime marker state 时可以打开。

更详细的 Factory debug view 使用 `src/robot_simulation/rviz/factory_patrol_debug.rviz`；
单独运行 Nav2 map/costmap debug 时使用 `src/robot_simulation/rviz/nav2_basic_debug.rviz`。

Gazebo world 使用轻量 procedural SDF primitive 表示 receiving、storage、packing、dock、
safety、低饱和 station sign、floor seam、scuff mark、landmark detail、大型 Factory layout
layer 和 route marking，使 AMR inspection loop 与 rack、rail、wall 和 workcell prop 在视觉上
分开。这些视觉资产本身不代表 runtime mission success。

### Factory Patrol RGB-D Camera

`factory_patrol.sdf` 和 `factory_patrol_industrial.sdf` 中的主机器人 `mobile_robot` 各有一个
simulated RGB-D camera。同一 camera mount 在 `robot.urdf.xacro` 中表示；Xacro property 是
权威 extrinsic，static asset check 会验证两个 world 与其一致。

Camera 参数：

| Parameter | Value |
| --- | --- |
| Parent frame | `base_link` |
| Camera body frame | `camera_link` |
| Optical frame | `camera_color_optical_frame` |
| `base_link -> camera_link` translation | `x=0.58 m, y=0.0 m, z=0.42 m` |
| `camera_link -> camera_color_optical_frame` rotation | RPY `-pi/2, 0, -pi/2` |
| Resolution | `640 x 480` |
| Update rate | `15 Hz` |
| Horizontal field of view | `1.0471975512 rad` (60 degrees) |
| Depth clip range | `0.1 m` to `10.0 m` |

optical frame 遵循 ROS camera convention：`+Z` 向前、`+X` 向右、`+Y` 向下。RGB、Depth 和
CameraInfo header 使用 `camera_color_optical_frame`。

ROS topics exposed by the existing `ros_gz_bridge` process:

| Topic | Type |
| --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |

Factory Patrol showcase RViz configuration 默认显示 RGB image，Depth visualization 默认关闭。

静态验证：

```bash
bash scripts/check_factory_patrol_assets.sh
```

启动 Factory Patrol Demo 后的 runtime validation：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=false use_rviz:=false
```

在另一个已 source 的 shell 中：

```bash
bash scripts/check_factory_patrol_runtime_topics.sh
ros2 topic info /camera/color/image_raw
ros2 topic info /camera/depth/image_raw
ros2 topic echo --once /camera/color/camera_info
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

runtime check 验证 topic 存在与类型、RGB/Depth payload 非空、CameraInfo intrinsics 有效且
非零、camera header frame ID 和 camera TF。它不验证 detection、3D projection、visual
navigation 或 perception safety behavior。

### RGB-D Geometry 验证

geometry validation 使用 synthetic bbox 独立验证 depth projection 和 timestamped TF，
不依赖 Detector 推理结果。Pipeline 为：

```text
synthetic bbox + synchronized RGB/depth/CameraInfo
  -> central-ROI median depth
  -> PointStamped in camera_color_optical_frame
  -> TF2 lookup at the depth observation timestamp
  -> PointStamped in map
  -> RViz sphere marker
```

输入：

| Topic | Type |
| --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` (`32FC1`) |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |

输出：

| Topic | Type | Frame |
| --- | --- | --- |
| `/perception/geometry/camera_point` | `geometry_msgs/msg/PointStamped` | `camera_color_optical_frame` |
| `/perception/geometry/map_point` | `geometry_msgs/msg/PointStamped` | `map` |
| `/perception/markers` | `visualization_msgs/msg/Marker` | `map` |

三个 camera stream 使用 `message_filters::ApproximateTime`，默认最大间隔为 50 ms。Depth
message timestamp 是两个 PointStamped output 和 TF lookup 的权威 observation timestamp。
节点不回退到 latest TF。缺失 observation-time transform 时抑制 map point 和 marker，但
camera point 与节点仍保持运行。

对于 bbox center `(u, v)`、median depth `Z` 和 CameraInfo intrinsics，相机坐标点为：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = median valid depth
```

Depth 从 bbox 中心区域采样。`src/robot_perception/config/depth.yaml` 默认 ROI ratio 为
`0.3`，最小 `0.2 m`、最大 `8.0 m`，使用 median statistic，至少需要五个有效样本。Zero、
NaN、Inf、低于范围和超出范围的值都会拒绝。Invalid depth 或 intrinsics 不产生点。

两个 Factory Patrol world 都包含只用于视觉、不可碰撞的
`geometry_validation_target`。目标视觉中心为 `[2.76, 0.00, 0.66]`，前表面为
`x = 2.70`。默认 bbox center ray 位于 settling 后的 camera/TF 高度 `z = 0.495`，因此
map convention 下已知表面交点为 `P_gt = [2.70, 0.00, 0.495]`。Synthetic bbox 中心为
`(320, 240)`，可通过 ROS parameter 修改，无需重新编译。

Gazebo freely settling 的 model 可能在 wheel odometry 开始跟踪运动前产生轻微位移，该被动
位移不体现在 differential-drive odometry frame 中。应从 geometry node log 记录 measured
`P_est` 与 `P_gt` 的误差。两者可能受 simulator/odometry origin 影响而不完全相等；geometry
实现没有 latest-TF fallback。

Factory Patrol 不使用 Nav2/AMCL 时，geometry validation launch 显式发布 identity
`map -> odom` static transform。`use_nav2:=true` 时关闭该 simulation transform，AMCL 保持
权威。geometry node 不存在 transform fallback。

启动集成 validation：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  gui:=false use_rviz:=false use_nav2:=false use_geometry_validation:=true
```

或者针对已有 camera/TF runtime 只启动 geometry component：

```bash
ros2 launch robot_perception geometry_validation.launch.py \
  publish_sim_map_tf:=true
```

验证命令：

```bash
colcon test --packages-select robot_perception
colcon test-result --verbose
bash scripts/check_factory_patrol_assets.sh
bash scripts/check_factory_patrol_runtime_topics.sh
ros2 topic echo --once /perception/geometry/camera_point
ros2 topic echo --once /perception/geometry/map_point
ros2 topic echo --once /perception/markers
```

已知限制：该 validation 使用一个可配置 synthetic bbox 和一个静态 target，只隔离检查
geometry/TF。定量精度来自实际仿真运行，目标摆放本身不是 accuracy result。

### 2D Object Detection

`robot_perception` 提供可替换的 Python detector adapter。默认 backend 是使用 COCO class
的 OpenCV-DNN YOLOX-S。它消费 RGB image，并发布标准
`vision_msgs/msg/Detection2DArray` contract：

```text
/camera/color/image_raw
  -> DetectorBackend (OpenCV-DNN YOLOX-S)
  -> /perception/detections_2d
  -> ApproximateTime with depth + CameraInfo
  -> existing central-ROI median DepthProjector
  -> camera PointStamped + observation-time TF
  -> map PointStamped
  -> TargetManager + managed sphere/text RViz markers
```

Detector 将 source image header 复制到 `Detection2DArray` 和每个 `Detection2D`，不替换为
inference-completion timestamp。在 `geometry_input_mode:=detector` 时，geometry node 将
该 detection header 与 Depth、CameraInfo 同步。detection/source-image timestamp 对 output
point 和 TF lookup 仍是权威时间戳。一个 frame 中的多个 detection 独立处理。Invalid depth
只抑制对应 3D result，不会产生虚假 point。

Detector 与 Geometry topics：

| Topic | Type | Contents/frame |
| --- | --- | --- |
| `/perception/detections_2d` | `vision_msgs/msg/Detection2DArray` | class, confidence, pixel bbox; `camera_color_optical_frame` |
| `/perception/debug_image` | `sensor_msgs/msg/Image` | optional RGB boxes/labels; `camera_color_optical_frame` |
| `/perception/geometry/camera_point` | `geometry_msgs/msg/PointStamped` | valid median-depth projection; optical frame |
| `/perception/geometry/map_point` | `geometry_msgs/msg/PointStamped` | observation-time TF result; `map` |
| `/perception/markers` | `visualization_msgs/msg/Marker` | managed target ID/class/state/filtered-position markers in detector mode |
| `/perception/objects_3d` | `robot_interfaces_perception/msg/DetectedObject3D` | managed map-frame target；每次更新按保留目标发布 |

Detector 参数位于 `src/robot_perception/config/detector.yaml`：

| Parameter | Default | Purpose |
| --- | --- | --- |
| `backend` | `opencv_yolox` | 可替换 backend 选择 |
| `model_path` | empty | 为空时解析到已校验的 user-cache path |
| `confidence_threshold` | `0.45` | 发布 class 的最低 confidence |
| `nms_threshold` | `0.5` | class-aware nonmaximum suppression IoU |
| `input_size` | `640` | 方形 YOLOX input size |
| `device` | `auto` | `auto`、`cpu` 或 OpenCV-DNN `cuda` |
| `allowed_classes` | `[person]` | 发布的 COCO class allowlist；为空表示全部允许 |
| `debug_image_enabled` | `true` | 启用 annotated debug image |

ONNX weight 不存储在 Git 中，普通 launch 不会下载 model。显式获取并 SHA-256 校验官方
OpenCV Zoo model：

```bash
sudo apt install ros-jazzy-cv-bridge ros-jazzy-vision-msgs \
  python3-opencv python3-numpy
bash scripts/prepare_detector_model.sh
```

脚本默认将 `object_detection_yolox_2022nov.onnx` 安装到
`${XDG_CACHE_HOME:-$HOME/.cache}/robot_perception/models`。可用
`ROBOT_PERCEPTION_MODEL_DIR` 覆盖目录，或用 `detector_model_path:=/absolute/model.onnx` 传入
明确的 launch path。下载 URL 可用 `ROBOT_PERCEPTION_MODEL_URL` 覆盖，但安装前始终要求同一
固定 SHA-256。Backend 使用 system `python3-opencv` 和 `python3-numpy`；只有在显式请求且
安装的 OpenCV build 可用时才使用 CUDA。缺失、截断或不支持的 model 会记录 error、关闭
inference，但 ROS node 仍存活并发布时间戳正确的空 detection array。

两个 Factory Patrol world 都包含只用于视觉的 `person_detection_target`，它与
center-ray target 错开，以保持 synthetic regression 不变。其 source、license 和 mechanical
texture reduction 记录在 model 旁的 `src/robot_simulation/models/person_standing/ATTRIBUTION.md`。

运行真实 detector path：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  gui:=false use_rviz:=false use_nav2:=false \
  use_detector:=true geometry_input_mode:=detector
```

保持 `use_detector:=false` 和 `geometry_input_mode:=synthetic`（两者均为默认值）即可使用
deterministic geometry regression path。这些开关也允许实现相同 `Detection2DArray` contract 的其他 node
替换 YOLOX，而不修改 depth projection。

验证命令：

```bash
colcon test --packages-select robot_perception
bash scripts/check_factory_patrol_assets.sh
bash scripts/check_factory_patrol_runtime_topics.sh
FACTORY_PATROL_DETECTOR_MODE=true bash scripts/check_factory_patrol_runtime_topics.sh
bash scripts/check_factory_patrol_detector_runtime.sh
ros2 topic echo --once /perception/detections_2d
ros2 topic echo --once /perception/geometry/map_point
ros2 topic echo --once /perception/markers
```

Inference latency 围绕真实 backend inference 测量，并记录当前与 running-average milliseconds。
runtime validator 要求真实 `person` detection 以及生成的 camera/map point 和 text marker，
不合成 detection。Detector failure 和空场景不会产生新的 3D observation。Detector 与 geometry
contract 保持不变，下游 TargetManager 消费有效的 map-frame result。

### Managed Targets

validated map-frame projection 下游使用 ROS-independent C++ `TargetManager`。
Detector 和 OpenCV type 保持在上游。其 generic input 包含 class name、confidence、finite
map-frame XYZ、source observation timestamp 和 depth validity。Invalid depth、空 class name、
超范围 confidence、non-finite position、future-dated observation 和 out-of-order update
cycle 会被拒绝，不会创建 target。

```text
Detection2DArray + depth + CameraInfo
  -> existing DepthProjector
  -> observation-time TF to map
  -> generic 3D observations (one synchronized frame)
  -> class-compatible 3D distance association
  -> confirmation + lifecycle + EMA
  -> /perception/objects_3d + /perception/markers
```

目标 lifecycle 为：

```text
               confirm_frames matched observations
TENTATIVE ----------------------------------------------> CONFIRMED
    |                                                        |
    +---------------- lost_frames misses --------------------+
                                                             |
                                                             v
                                                            LOST

TENTATIVE / CONFIRMED -- MarkProcessed() --> PROCESSED
PROCESSED -- cooldown expires and object is observed --> TENTATIVE (same ID)
```

`confirm_frames` 统计 compatible match，不要求严格连续；短于 `lost_frames` 的 dropout 会
保留 target 和 ID。错过 `lost_frames` 个同步 detector cycle 后，target 进入 LOST；在 missed
count 超过 `lost_frames` 两倍前仍可匹配，之后为限制内存而删除。匹配到 LOST target 时以
同一 ID reacquire；已确认 target 直接回到 CONFIRMED。ID 从 1 单调递增，在进程生命周期内
不回收。

每次 update 会把 `max_match_distance` 范围内同 class 的 target/observation pair 按 distance、
target ID、observation order 排序。Greedy selection 保证一个 observation 对应一个 target，
一个 target 对应一个 observation。已接受 observation 的 match radius 内其他同 class
observation 会作为 duplicate 抑制。这是小型确定性 spatial association policy，不是
appearance tracking、DeepSORT、ByteTrack 或 ReID。

managed position 使用 exponential moving average：

```text
p_filtered = ema_alpha * p_new + (1 - ema_alpha) * p_previous
```

Raw latest position 在内部保留用于 validation；公开的 managed position 和 marker 使用 filtered
value。Confidence 使用最新 matched detector confidence。Marker 使用来自 `target_id` 的
stable ID，显示 `#12 person CONFIRMED` 等 text，并用 `DELETEALL` 加 bounded marker lifetime
保持 LOST/removed-target update 清晰。

最小 domain interface 位于 `robot_interfaces_perception`，符合仓库按职责拆分 custom interface
的做法。`DetectedObject3D.msg` 包含 source-observation header、stable `target_id`、class、
latest confidence、filtered map position、depth validity，以及 `TENTATIVE`、`CONFIRMED`、
`LOST` 或 `PROCESSED` 之一。当前只有 C++ manager API 能到达 `PROCESSED`；没有增加 mission
consumer 或 ROS behavior API。在 `processed_cooldown_sec` 期间，匹配 observation 更新保留的
target，但不创建新的 actionable identity。cooldown 后，匹配 observation 以同一 ID 重新进入
TENTATIVE。

Tracking 参数从 `src/robot_perception/config/tracking.yaml` 加载：

| Parameter | Default | Validation / behavior |
| --- | --- | --- |
| `tracking.confirm_frames` | `3` | 正的 matched-observation count |
| `tracking.lost_frames` | `5` | 正的 missed synchronized cycle 数 |
| `tracking.max_match_distance` | `0.5` m | 正的 finite 3D match radius |
| `tracking.ema_alpha` | `0.4` | `(0, 1]` 内的 finite value |
| `tracking.processed_cooldown_sec` | `10.0` s | nonnegative finite ROS-time duration |

验证命令：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select robot_interfaces_perception robot_perception
colcon test-result --verbose
bash scripts/check_factory_patrol_assets.sh

ros2 launch robot_bringup factory_patrol_demo.launch.py \
  gui:=false use_rviz:=false use_nav2:=false \
  use_detector:=true geometry_input_mode:=detector

bash scripts/check_factory_patrol_detector_runtime.sh
bash scripts/check_factory_patrol_target_manager_runtime.sh
ros2 topic echo --once /perception/objects_3d
```

TargetManager runtime check 使用真实 Factory Patrol person detection 和 live RGB-D/TF graph。隔离 relay
提供 real、empty 和一个 duplicated detection cycle，在不改变 world 的情况下验证 TENTATIVE、
CONFIRMED、short dropout、LOST、same-ID reacquisition 和 duplicate suppression。执行运行有足够
sample 时会报告 measured raw/EMA XYZ standard deviation 和 manager update latency，不会制造结果。

2026-08-13 的 WSL runtime validation 使用一个真实 Factory Patrol person target，
`target_id=1` 稳定。它观察到 TENTATIVE、CONFIRMED、short dropout、LOST、same-ID reacquisition、
duplicate suppression 和 managed marker。15 组 paired stationary sample 中，raw 与 filtered
population standard deviation 都是 `x=0.000000 m`、`y=0.000000 m`、`z=0.000000 m`。因此本次
运行没有测出 EMA improvement，因为 raw projected position 没有可观察 jitter。隔离 node 记录
TargetManager update latency 为 `7.576 us`；detector inference 是独立成本，不包含在此测量中。

已知限制：association 没有 velocity model 或 appearance information，所有 class 使用一个
3D radius，frame-count lifecycle behavior 依赖 synchronized detector cycle；被删除 target
稍后重新发现时会收到新 ID。`DetectedObject3D` 按 retained target 逐条发布，不是 array。
实体机器人上的目标身份稳定性尚未验证。

## Factory Patrol Scene V2 预览

`src/robot_simulation/worlds/factory_patrol_industrial.sdf` 是独立的 industrial-layout preview
world。它保留原始 `factory_patrol.sdf` baseline，并保持相同的 `factory_patrol` world name、
robot name、topic、frame 和 mission seed semantics。V2 将视觉 Factory floor 扩展到 24 m x 16 m，
保持 collision geometry 简单，并加入分层 industrial layout：

- AMR dock / D01 charging visual area near the existing dock seed pose
- receiving and inbound buffer visuals on the left side
- back storage rack rows with simple rack collision boxes and visual loads
- right-side packing workcell with bins, tool board, safety boundaries
- a closed dock -> receiving -> storage -> packing -> dock inspection loop
- slow-zone hatching, waiting / stop markers, guardrails, bollards, and estop

V2 visual loop 尽量与现有 station seed pose 对齐。场景视觉资产与 navigation、chassis、safety
和 task logic 保持解耦，route anchor 变更需要同步审查语义配置。

显式启动 V2：

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  world_file:=$(ros2 pkg prefix robot_simulation)/share/robot_simulation/worlds/factory_patrol_industrial.sdf \
  gui:=true use_rviz:=true
```

Headless V2 smoke test：

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  world_file:=$(ros2 pkg prefix robot_simulation)/share/robot_simulation/worlds/factory_patrol_industrial.sdf \
  gui:=false use_rviz:=false
```

通过 virtual RC input 执行基础 AMR motion smoke test：

```bash
ros2 topic pub --rate 10 /virtual_rc/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 topic pub --once /virtual_rc/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

手动测试使用 `/virtual_rc/cmd_vel`。`/teleop_cmd_vel` 是 `virtual_rc_node` 进入
`cmd_vel_mux_node` 的 output；`/cmd_vel` 是 Gazebo bridge 使用的 final mux/safety output，
不应作为普通手动 input。其他 mux input 为 `/nav2_cmd_vel` 和 `/tracking_cmd_vel`。机器人
不移动时观察 `/cmd_vel_mux/active_source`、`/safety_state`、`/cmd_vel`、`/odom` 和 Gazebo
entity pose：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic echo /safety_state
ros2 topic echo /cmd_vel_mux/active_source
ros2 topic info -v /cmd_vel
gz model --model mobile_robot --pose
```

## Factory Patrol Demo Workflow

Factory Patrol 资产通过三个 workflow 提供 launch/script/config 入口和验收步骤。各 workflow
是否通过以对应运行记录为准。

### Demo 1: Multipoint Patrol

目标：

```text
start -> station_A -> station_B -> station_C -> dock
```

入口：

```bash
bash scripts/run_factory_patrol_multipoint_demo.sh
python3 scripts/print_factory_patrol_goals.py
```

配置资产：

- `src/robot_simulation/config/factory_patrol_route.yaml`
- `src/robot_simulation/config/factory_patrol_stations.yaml`
- `src/robot_simulation/config/factory_patrol_multipoint_mission.yaml`

真实运行中应观察的 topic：

- `/navigate_sequence/current_goal`
- `/navigate_sequence/current_path`
- `/mission_runner/state`
- `/cmd_vel`
- `/odom`
- `/safety/state`
- `/safety/reason`

runtime log 应验证：每个 waypoint 按顺序发布，机器人接近每个 station，mission 返回 `dock`。
当前边界：script 和 mission profile 已有，但在采集真实运行前不填写 success rate 或 travel time。

### Demo 2: Temporary Obstacle

目标：在 `station_A` 到 `station_B` 路段附近放置简单 temporary box obstacle，观察
perception/planning/safety topic。

入口：

```bash
bash scripts/run_factory_patrol_obstacle_demo.sh
```

配置/model 资产：

- `src/robot_simulation/config/factory_patrol_obstacle_demo.yaml`
- `src/robot_simulation/models/temporary_box_obstacle/model.sdf`
- `src/robot_simulation/models/temporary_box_obstacle/model.config`

建议观察的 runtime topic：

- `/scan`
- `/local_costmap/costmap`
- `/cmd_vel`
- `/safety/state`
- `/safety/reason`

runtime log 应验证：障碍出现在 scan data 和 local costmap 中，controller response 出现在
`/cmd_vel`。机器人减速、停车还是 replanning 取决于实际 Nav2 runtime state；该场景尚未
提交统一条件下的专项避障结果。

### Demo 3: Localization Lost And Recovery

目标：注入错误的 `/initialpose`，再注入恢复用 `/initialpose`，观察 localization health 与
safety-state linkage。

入口：

```bash
bash scripts/run_factory_patrol_localization_recovery_demo.sh
```

配置资产：

- `src/robot_simulation/config/factory_patrol_localization_recovery.yaml`

如果 runtime condition 触发，应观察以下 state label：

```text
LOCALIZATION_LOST -> LOCALIZATION_RECOVERING -> LOCALIZATION_RECOVERED -> LOCALIZATION_OK
```

观察 topic：

- `/localization/health`
- `/safety/state`
- `/safety/reason`
- `/amcl_pose`
- `/tf`

预期 safety linkage：`LOCALIZATION_LOST` 通过 `/safety/state` 可见，并由当前 Safety Gate
policy 强制零 command。具体 transition 需要由 ROS2/Nav2 runtime log 验证。

### Checks

静态 workflow check：

```bash
bash scripts/check_factory_patrol_demo_workflows.sh
```

Demo 和 Nav2 运行后的 runtime topic check：

```bash
bash scripts/check_factory_patrol_demo_runtime.sh
```

当前提供：

- demo workflow scripts
- multipoint mission/profile asset
- temporary obstacle config and simple SDF model
- localization recovery pose config
- runtime and static check scripts
- acceptance documentation

尚未实现或未形成统一专项证据的扩展项统一列在 [项目路线图](roadmap.md)。


## 视觉引导静态巡检

稳定的 managed target 通过 task-owned Nav2 mission 执行巡检：

```text
/perception/objects_3d CONFIRMED
  -> /perception/events INSPECTION_REQUIRED
  -> visual_inspection_task_node
  -> /navigate_sequence
  -> NavigateToPose
  -> arrival
  -> INSPECTION_COMPLETED
  -> /perception/objects_3d PROCESSED
```

`PerceptionEvent` interface 包含 `target_id`、`event_type`、class、confidence、severity 和带
timestamp 的 `target_pose`。巡检链路使用 `TARGET_CONFIRMED`、`INSPECTION_REQUIRED` 和
`INSPECTION_COMPLETED`；人员安全使用独立的 `PerceptionSafetyEvent` contract。

### Eligibility 与 duplicate policy

Detector 继续提供 `person` 和 `chair`，但默认 inspection allowlist 只有 `chair`，
`min_confidence: 0.5`。raw detector frame 不能启动 task；target 必须先通过
`TargetManager` 到达 `CONFIRMED`。Perception 为该 managed target 发布一个 actionable event，
并抑制后续 frame。task 成功后 target 标记为 `PROCESSED`；已有
`tracking.processed_cooldown_sec` 必须到期，target 再次经过 `TENTATIVE` 和 `CONFIRMED` 后，
才能发布新 event。

现有 pretrained model 不能可靠识别 code-native primitive chair。因此 runtime validation
显式加载 `tracking_visual_inspection_validation.yaml` 和 `visual_inspection_validation.yaml`，
其 allowlist 包含 `person`，并在已知 world pose `(2.80, -0.75)` 复用现有只用于视觉的静态
`person_detection_target`。这不改变默认 `chair` policy，不添加 collision geometry，
也不实现 person following；该 target 仅是此明确 validation run 的固定 inspection fixture。
Validation tracking profile 还缩小 depth ROI，并延长 LOST-target retention，使机器人转向或
静态 fixture 离开 camera view 后，原 managed ID 仍可用于 task completion。默认
retention behavior 不变。其 processed cooldown 为 120 秒，避免同一 fixture 在端到端测量
窗口内 retrigger。`--visual-inspection` helper 将 CPU inference 限制为 0.5 Hz，保证 WSL 中 Nav2
action callback 响应；detector 仍使用 live RGB image 上同一个 pretrained YOLOX backend，
默认 detector rate 不限速。Validation profile 允许一次 task-owned retry 处理到达时的瞬时
Nav2 result race；默认 mission profile 仍为 `retry_count: 0`。

### Observation Pose

规划发生在 `map` 中。对于 target position `T`、current robot position `R` 和配置的
`standoff_distance`，task 计算：

```text
d = normalize(R - T)
observation_position = T + d * standoff_distance
yaw = atan2(T.y - observation.y, T.x - observation.x)
```

默认 `standoff_distance` 为 `1.2` m。Planner 拒绝 non-finite pose、非 map frame、invalid
quaternion 和 non-positive standoff。如果 `R` 与 `T` 重合，使用机器人当前 heading 的反方向。
Mission 开始时冻结选定 pose；target update 或暂时 detector loss 不替换 Nav2 goal。该任务
面向静态 inspection target，不实现 pursuit、following 或 visual servoing。

Navigation failure 发布 `FAILED` task status，不会将 target 标记为成功 processed。默认 retry
count 为零。Nav2 rejection 或 unreachable pose 按普通 task failure 处理，不增加 custom
planner 或 costmap search。

Factory Patrol bringup 将 event consumer 延迟到已有 delayed Nav2 bringup 完成之后。
由于 `/perception/events` 是 transient-local，Nav2 activation 期间确认的 target 会在 task
启动后 replay 一次，避免把正常 lifecycle startup 当作 navigation failure。

### Launch 与 validation

准备 Detector model，构建后启动完整 chain：

```bash
bash scripts/prepare_detector_model.sh
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
bash scripts/run_factory_patrol_demo.sh --visual-inspection
```

等价的 launch command 为：

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  gui:=true use_rviz:=true use_nav2:=true use_detector:=true \
  geometry_input_mode:=detector use_visual_inspection:=true \
  perception_max_inference_rate_hz:=0.5 \
  perception_tracking_params:=$(ros2 pkg prefix --share robot_perception)/config/tracking_visual_inspection_validation.yaml \
  visual_inspection_params:=$(ros2 pkg prefix --share robot_tasks)/config/visual_inspection_validation.yaml
```

在另一个已 source 的 shell 中验证 running graph：

```bash
FACTORY_PATROL_DETECTOR_MODE=true \
FACTORY_PATROL_VISUAL_INSPECTION_MODE=true \
  bash scripts/check_factory_patrol_runtime_topics.sh

bash scripts/check_factory_patrol_visual_inspection_runtime.sh
ros2 topic echo /perception/events
ros2 topic echo /inspection/status
ros2 topic echo /inspection/observation_pose
ros2 topic echo /perception/objects_3d
```

RViz showcase 显示 managed target label/ID 和 Observation Pose；已有 current-goal 与 odometry/path
display 仍可用。Runtime measurement 来自 validation script output。

2026-08-13 在 WSL 中使用 visual-inspection profile 和 live YOLOX detector 验证了完整 headless
Factory Patrol bringup。`target_id=1`（仅 validation fixture 的 `person`）在 simulation time
`18.679 s` 产生一次 accepted visual inspection mission。测得 map position 为
`(2.842653, -0.759916)` m，规划的 Observation Pose 为 `(1.683362, -0.450007)` m；配置与
规划 standoff 都是 `1.2 m`。Nav2 接受 goal，并在 `6.489` simulated seconds 后返回 `SUCCEEDED`。
最终 robot pose 为 `(1.570, -0.334, yaw=-0.421137)`，Observation Pose error 为 `0.162199 m`，
到 target 的 measured planar distance 为 `1.342032 m`，robot displacement 为 `1.605134 m`。
运行观察到 target 1 各一次 `INSPECTION_REQUIRED` 和 `INSPECTION_COMPLETED`，随后验证进入
`PROCESSED` 且没有立即启动第二个 mission；同时验证 perception 没有 velocity topic，command
路径保持 `/nav2_cmd_vel -> cmd_vel_mux_node/Safety Gate -> /cmd_vel`。

## Showcase 证据边界

Factory Patrol asset 可支持截图、视频和报告图表。当前 `docs/showcase/` 只索引经过审阅的
证据；未来 artifact 需要记录确切 launch command、commit、map/world、parameters 和
log/rosbag path。

## 视觉人员安全与 Safety Gate 集成

人员安全 probe 使用只用于视觉的 `person_detection_target` 和 Gazebo set-pose service 做
deterministic validation，不增加 crowd 或 pedestrian framework。标准 licensed person mesh
太高，在低于 `1.5 m` STOP threshold 时无法完全位于 camera vertical field of view 内，因此
两个 Factory Patrol world 还包含 `person_safety_target`：同一 licensed mesh 的静态、
不可碰撞 `0.40` scale instance。除非 safety probe 用于 close-range STOP case，否则它隐藏
在 `z=-2`。这只改变 validation image scale；policy 仍使用 measured RGB-D XY distance 和
`person` class。

Runtime chain：

```text
managed person (`CONFIRMED` or observed `PROCESSED`)
  -> PerceptionSafetyPolicy
  -> /perception/safety_event
  -> existing cmd_vel mux / Safety Gate
  -> /cmd_vel
```

`/perception/safety_event` 类型为 `robot_interfaces_perception/msg/PerceptionSafetyEvent`。
它携带 target ID/class、map position、measured robot-relative planar distance、semantic event、
severity、source/reason 和可选 danger-zone ID，与 inspection mission event 分开。Perception NEVER
publishes `/cmd_vel` or `/nav2_cmd_vel`。

默认 STOP distance threshold 为 `1.5 m`，SPEED_LIMITED 为 `3.0 m`。精确等于 `1.5 m` 和
`3.0 m` 的边界分别按 limited 处理，而不是 stopped/clear。STOP 在超过 `1.7 m` 后清除，
SPEED_LIMITED 在超过 `3.2 m` 后清除，并要求三个有效的 less-restrictive observation。多个人员
取最严格状态。配置的 map-frame polygon `factory_person_danger_zone` 为：

```text
[(3.00, -1.20), (3.80, -1.20), (3.80, -0.30), (3.00, -0.30)]
```

Polygon boundary 也算 inside。人员在该 zone 内时，即使 robot-relative distance 大于 `1.5 m`，
也会触发 STOP。配置位于 `robot_perception/config/safety_zones.yaml`，复用已有
`robot_navigation::ZoneCatalog` map polygon convention。

Safety Gate freshness 在 ROS/simulation time 中为 `1.5 s`。stale event 只移除 perception
restriction，不覆盖 estop、localization、chassis、scan、watchdog 或 legacy `/safety_state`
condition。Invalid TF 或 malformed person data 不会创建 `CLEAR` event。

准备 Detector model 并启动人员安全 profile：

```bash
bash scripts/prepare_detector_model.sh
bash scripts/run_factory_patrol_demo.sh --perception-safety
```

在另一个已 source 的 shell 中验证 topic 和端到端 gate：

```bash
FACTORY_PATROL_DETECTOR_MODE=true \
FACTORY_PATROL_PERCEPTION_SAFETY_MODE=true \
  bash scripts/check_factory_patrol_runtime_topics.sh

bash scripts/check_factory_patrol_perception_safety_runtime.sh
```

runtime probe 让 visual fixture 经过 measured CLEAR、SPEED_LIMITED、STOP、danger-zone STOP 和
recovery case，同时保持一个已有 Nav2 goal active。它报告真实 `/nav2_cmd_vel` 与最终
`/cmd_vel` sample，以及 ROS-time STOP response latency。

2026-08-14 的 headless WSL smoke run 从 live RGB-D、YOLOX、depth projection、TargetManager、
policy 和 Safety Gate chain 得到以下 measured result：

| Case | Measured result |
| --- | --- |
| CLEAR | Person distance `3.260 m`; perception `CLEAR`; final safety state `NORMAL` |
| SPEED_LIMITED | Person distance `2.955 m`; upstream `/nav2_cmd_vel` linear `0.350 m/s`; final `/cmd_vel` linear `0.150 m/s`; final state `SPEED_LIMITED` |
| STOP | Person distance `1.314 m`; upstream linear `0.350 m/s`; final linear `0.000 m/s`; final state `STOP` |
| Danger zone | Person map position `(3.253, -0.727) m`, distance `3.222 m`; inside `factory_person_danger_zone`; final state `STOP`; final linear velocity `0.000 m/s` |
| Recovery | Three valid clear observations at `3.260 m`; final state returned to `NORMAL`; the original Nav2 goal stayed active and final `(0.350, -0.040)` linear/angular command matched its upstream intent |

STOP transition 的 qualifying condition timestamp 为 simulation time `12.079 s`，第一个要求的
final safe command timestamp 为 `12.369 s`，观察到的 response latency 为 `0.290 s`。Safety
event 在 `12.343 s` 收到。这是一次 smoke-test measurement，不属于正式 latency distribution。

运行还验证 perception 没有 `/cmd_vel` 或 `/nav2_cmd_vel` publisher。已知仿真限制是 Gazebo
`mobile_robot` world pose 几乎不变，而 bridged odometry 仍在积分运动。因此 probe 使用已有
Gazebo set-pose service 定位只用于视觉的 person fixture，并保持一个 Nav2 goal active 以提供
真实 upstream command intent；该 smoke test 没有验证 world-model 中真实移动的机器人接近人员。

### Perception Diagnostics 与恢复

Factory Patrol profile 提供可选的低频 health stream，使用以下命令启用：

```bash
bash scripts/run_factory_patrol_demo.sh --perception-diagnostics
```

标准 `diagnostic_msgs/msg/DiagnosticArray` topic 为 `/perception/diagnostics`，发布的 status name 为：

| Status | Observed condition |
| --- | --- |
| `perception/camera_rgb` | RGB receipt age and optical-frame ID |
| `perception/camera_depth` | Depth receipt age and optical-frame ID |
| `perception/camera_info` | CameraInfo receipt age, frame, and nonzero intrinsics |
| `perception/detector` | Detector model availability, exceptions, consecutive failures, and latency |
| `perception/tf` | Observation-time `map <- camera_color_optical_frame` lookup results |
| `perception/depth_quality` | Global invalid depth ratio; zero, NaN, Inf, and out-of-range values are invalid |
| `perception/pipeline` | Worst component level and component summary |

`robot_perception/config/diagnostics.yaml` 默认 startup grace 为 10 s，camera warning/error age
为 0.5/1.5 s，depth limit 为 0.2/8.0 m，invalid-depth ratio warning/error threshold 为
0.25/0.60。`config/detector.yaml` 中 Detector latency warning/error threshold 为 1200/3000 ms。
这些是仿真默认值，部署 camera 和 detector 时应重新 calibration。Diagnostics 在 WARN 时仅供
信息参考；fresh component ERROR 会由已有 `system_monitor_node` 聚合，并沿已有
`fault_supervisor_node` error path 处理。stale perception diagnostic stream 报告为 WARN，
本身不会请求 emergency stop。

Perception diagnostics profile 设置 `monitor_base_system:=false`，因为该 Gazebo profile
没有运行 hardware chassis-state publisher。参数默认值为 `true`，因此 normal hardware 和
mock bringup monitoring 不变；该 profile 仍沿同一 system-health 和 fault-supervisor
path 传播 perception ERROR。

Missing 或 invalid detector/depth/TF observation 会抑制新的 geometry 和 safety event。特别是
detector mode 中，当 frame 没有 detection 或所有 projection 都无效时，不会发出新的 CLEAR
event。已有 Safety Gate event timeout 只可移除 perception restriction；chassis、watchdog、
localization、scan 和其他 safety condition 仍然具有权威性。

在隔离的 ROS graph 中运行 deterministic fault-injection probe：

```bash
bash scripts/check_factory_patrol_perception_diagnostics_runtime.sh
```

对于 live Factory Patrol instance，当 `FACTORY_PATROL_PERCEPTION_DIAGNOSTICS_MODE=true` 时，
已有 topic checker 会包含 perception diagnostics status：

```bash
FACTORY_PATROL_DETECTOR_MODE=true \
FACTORY_PATROL_PERCEPTION_SAFETY_MODE=true \
FACTORY_PATROL_PERCEPTION_DIAGNOSTICS_MODE=true \
  bash scripts/check_factory_patrol_runtime_topics.sh
```

仿真限制仍然存在：freely settling Gazebo world pose 和 integrated odometry 不代表实体机器人
精确运动。Diagnostics 通过已有 health input 报告该情况，不改变
navigation 或 mission behavior。
