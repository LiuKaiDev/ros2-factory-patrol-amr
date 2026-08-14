# Simulation Scenarios

当前仿真基础已经能够支撑 AMR 演示和部分验收脚本，但面向“厂区 / 园区半封闭低速巡检”的专用场景仍是 planned。

## Current Simulation Basis

| Asset / module | Status | Notes |
| --- | --- | --- |
| `src/robot_simulation/worlds/indoor_room.sdf` | current | 当前 Gazebo 室内场景。 |
| `src/robot_navigation/maps/indoor_room.yaml` | current | 当前静态地图。 |
| `src/robot_simulation/config/amr_sim_stations.yaml` | current | 仿真站点配置。 |
| `src/robot_simulation/config/amr_sim_map_zones.yaml` | current | 仿真 map zones 配置。 |
| `amr_sim_visualizer_node` | current | RViz marker 可视化。 |
| `amr_sim_demo_director_node` | current | 自动演示编排器。 |
| Gazebo entity bridge | current | 将部分业务 / 安全状态映射到 Gazebo 实体。 |

## Planned Factory Patrol Assets

| Asset | Status | Purpose |
| --- | --- | --- |
| `factory_patrol.sdf` | planned | 厂区 / 园区巡检 Gazebo world。 |
| `factory_patrol.yaml` map | planned | 与巡检 world 对齐的静态地图。 |
| patrol stations | planned | 巡检点、充电点、等待点。 |
| patrol zones | planned | 限速区、禁行区、临时障碍区。 |
| scenario launch | planned | 一键运行厂区巡检 demo。 |

## Demo Design

### 1. Multi-point Patrol

Status: partial current / planned factory demo.

目标：

- 从起点出发；
- 依次到达多个巡检点；
- 每个点停留或发布到达状态；
- 完成后返回等待点或充电点。

当前可复用任务 / 站点 / mission runner 能力；最终 factory 场景、巡检点配置和报告待 Phase 5 补齐。

### 2. Temporary Obstacle Avoidance

Status: planned.

目标：

- 在巡检路径上放置临时障碍；
- 验证 local costmap 感知、清障和绕行；
- 记录是否停车、是否重新规划、到达时间和 stop_count。

不能在未运行实验前声称成功率或避障性能。

### 3. Localization Lost and Recovery

Status: current logic / planned factory demo.

目标：

- 模拟 AMCL covariance 超阈值；
- localization health 进入 `LOST`；
- 任务暂停并请求重定位；
- 设置初始位姿后进入 `RECOVERED`；
- 任务恢复。

当前代码和检查脚本已有定位健康 / 重定位入口；最终巡检场景中的可视化演示和实验报告仍待补齐。

## Phase 5A Factory Patrol Assets

Phase 5A adds a factory patrol simulation asset skeleton for a low-speed
semi-closed AMR patrol site. This phase adds files, configuration, and launch
entry points only. It does not claim a full Gazebo/Nav2 mission has been run.

Scanned current structure:

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

New Phase 5A assets:

| Asset | Path | Status |
| --- | --- | --- |
| Factory world | `src/robot_simulation/worlds/factory_patrol.sdf` | current asset |
| Factory Scene V2 world | `src/robot_simulation/worlds/factory_patrol_industrial.sdf` | optional preview asset |
| Station seed config | `src/robot_simulation/config/factory_patrol_stations.yaml` | current config asset |
| Zone seed config | `src/robot_simulation/config/factory_patrol_zones.yaml` | current config asset |
| Patrol route config | `src/robot_simulation/config/factory_patrol_route.yaml` | current config asset / planned mission input |
| Map note | `src/robot_navigation/maps/factory_patrol_map_README.md` | current documentation |
| Demo launch | `src/robot_bringup/launch/factory_patrol_demo.launch.py` | current launch skeleton |
| Demo helper | `scripts/run_factory_patrol_demo.sh` | current helper script |
| Static check | `scripts/check_factory_patrol_assets.sh` | current static check |

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
- no-go planned visual overlay

Station config:

`src/robot_simulation/config/factory_patrol_stations.yaml` defines `start`,
`station_A`, `station_B`, `station_C`, and `dock` in the `map` frame. Coordinates
are simulation seed poses aligned to the SDF layout, not measured field poses.

Zone config:

`src/robot_simulation/config/factory_patrol_zones.yaml` defines
`factory_slow_corridor`, `factory_no_go_equipment_service_area`, and
`factory_turning_caution_area`. These are Phase 5A configuration assets only and
are not claimed to be wired into Nav2 costmap filters yet.

Route config:

`src/robot_simulation/config/factory_patrol_route.yaml` defines
`factory_patrol_loop`:

```text
start -> station_A -> station_B -> station_C -> dock
```

The current mission runner is not claimed to execute this YAML directly. Later
phases can wire it into mission execution or convert it into the package's
existing mission format.

Map handling:

Phase 5A does not commit a fake occupancy map. See
`src/robot_navigation/maps/factory_patrol_map_README.md` for the planned
SLAM/map_saver flow. Until a real or reviewed placeholder map exists, the
factory demo launch keeps Nav2 disabled by default.

Demo entry:

```bash
bash scripts/run_factory_patrol_demo.sh
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true
```

To experiment with Nav2 after providing an explicit factory map:

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  use_nav2:=true nav2_map:=/absolute/path/to/factory_patrol.yaml
```

RViz views:

The Factory Patrol demo launch uses
`src/robot_simulation/rviz/factory_patrol_showcase.rviz` by default when
`use_rviz:=true`. This is the non-Nav2 showcase view with `odom` fixed frame,
RobotModel, Lidar Scan, Odometry, Odom Path, and Factory Semantics markers.
Factory Semantics is included but disabled by default for cleaner screenshots;
enable it when inspecting runtime marker state.

Use `src/robot_simulation/rviz/factory_patrol_debug.rviz` for a more verbose
factory debug view, or `src/robot_simulation/rviz/nav2_basic_debug.rviz` when
running Nav2 map/costmap debugging separately.

The Gazebo world uses lightweight procedural SDF primitives for receiving,
storage, packing, dock, safety, muted station signage, floor finish seams, scuff
marks, landmark details, a larger factory layout layer, and route markings that
keep the AMR inspection loop visually separated from racks, rails, walls, and
workcell props. These visual assets do not claim runtime mission success by
themselves.

### Factory Patrol RGB-D Camera

The primary `mobile_robot` in both `factory_patrol.sdf` and
`factory_patrol_industrial.sdf` has one simulated RGB-D camera. The same camera
mount is represented in `robot.urdf.xacro`; the Xacro properties are the
authoritative extrinsic and the static asset check verifies that both worlds
mirror it.

Camera parameters:

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

The optical frame follows the ROS camera convention: `+Z` forward, `+X` right,
and `+Y` down. RGB, depth, and CameraInfo headers use
`camera_color_optical_frame`.

ROS topics exposed by the existing `ros_gz_bridge` process:

| Topic | Type |
| --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |

The Factory Patrol showcase RViz configuration displays the RGB image by
default. Depth visualization is not enabled by default.

Static validation:

```bash
bash scripts/check_factory_patrol_assets.sh
```

Runtime validation after starting the Factory Patrol demo:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=false use_rviz:=false
```

In a second sourced shell:

```bash
bash scripts/check_factory_patrol_runtime_topics.sh
ros2 topic info /camera/color/image_raw
ros2 topic info /camera/depth/image_raw
ros2 topic echo --once /camera/color/camera_info
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

The runtime check verifies topic presence and types, nonempty RGB/depth payloads,
valid nonzero CameraInfo intrinsics, camera header frame IDs, and the camera TF.
It does not validate detection, 3D projection, visual navigation, or perception
safety behavior.

### Phase 2 RGB-D Geometry Validation

Phase 2 adds the minimal `robot_perception` package for validating depth
projection and timestamped TF without an object detector. The pipeline is:

```text
synthetic bbox + synchronized RGB/depth/CameraInfo
  -> central-ROI median depth
  -> PointStamped in camera_color_optical_frame
  -> TF2 lookup at the depth observation timestamp
  -> PointStamped in map
  -> RViz sphere marker
```

Inputs:

| Topic | Type |
| --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` (`32FC1`) |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |

Outputs:

| Topic | Type | Frame |
| --- | --- | --- |
| `/perception/geometry/camera_point` | `geometry_msgs/msg/PointStamped` | `camera_color_optical_frame` |
| `/perception/geometry/map_point` | `geometry_msgs/msg/PointStamped` | `map` |
| `/perception/markers` | `visualization_msgs/msg/Marker` | `map` |

The three camera streams use `message_filters::ApproximateTime` with a default
50 ms maximum interval. The depth message timestamp is the authoritative
observation timestamp for both PointStamped outputs and the TF lookup. The node
does not fall back to latest TF. A missing observation-time transform suppresses
the map point and marker while leaving the camera point and node alive.

For a bbox center `(u, v)`, median depth `Z`, and CameraInfo intrinsics, the
camera-frame point is:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = median valid depth
```

Depth is sampled from the central portion of the bbox. Defaults in
`src/robot_perception/config/depth.yaml` are a `0.3` ROI ratio, `0.2 m` minimum,
`8.0 m` maximum, median statistic, and at least five valid samples. Zero, NaN,
Inf, below-range, and above-range values are rejected. Invalid depth or
intrinsics produce no point.

Both Factory Patrol worlds contain a visual-only, non-colliding
`phase2_geometry_validation_target`. The target visual is centered at
`[2.76, 0.00, 0.66]`; its front face is at `x = 2.70`. The default bbox center
ray is at the settled camera/TF height `z = 0.495`, so the relevant known
surface intersection is `P_gt = [2.70, 0.00, 0.495]` in the map convention.
The synthetic bbox is centered at `(320, 240)` and can be changed through ROS
parameters without recompilation.

Gazebo's freely settling model can move slightly before wheel odometry starts
tracking motion. That passive displacement is not represented by the
differential-drive odometry frame. Record the measured `P_est` versus `P_gt`
error from the geometry node log rather than assuming exact equality; this is
a simulator/odometry-origin limitation, not a latest-TF fallback.

When Factory Patrol runs without Nav2/AMCL, the Phase 2 launch explicitly
publishes an identity `map -> odom` static transform for this simulation
validation only. With `use_nav2:=true`, that simulation transform is disabled
and AMCL remains authoritative. No transform fallback exists in the geometry
node.

Launch the integrated validation:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  gui:=false use_rviz:=false use_nav2:=false use_geometry_validation:=true
```

Or launch only the geometry component against an existing camera/TF runtime:

```bash
ros2 launch robot_perception geometry_validation.launch.py \
  publish_sim_map_tf:=true
```

Validation commands:

```bash
colcon test --packages-select robot_perception
colcon test-result --verbose
bash scripts/check_factory_patrol_assets.sh
bash scripts/check_factory_patrol_runtime_topics.sh
ros2 topic echo --once /perception/geometry/camera_point
ros2 topic echo --once /perception/geometry/map_point
ros2 topic echo --once /perception/markers
```

Known limitations: Phase 2 uses one configurable synthetic bbox and one static
validation target. It does not detect or track objects, associate targets across
frames, filter positions over time, create perception events, affect missions or
safety, or publish velocity commands. Quantitative accuracy must be reported
from an executed simulation run; target placement alone is not an accuracy
result.

### Phase 3 Real 2D Object Detection

Phase 3 adds a replaceable Python detector adapter in the existing
`robot_perception` package. The supplied backend is OpenCV-DNN YOLOX-S with
COCO classes. It consumes RGB images and publishes the standard
`vision_msgs/msg/Detection2DArray` contract:

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

The detector copies the source image header to both `Detection2DArray` and
every `Detection2D`; no inference-completion timestamp is substituted. In
`geometry_input_mode:=detector`, the geometry node synchronizes that detection
header with depth and CameraInfo. The detection/source-image timestamp remains
authoritative for both output points and the TF lookup. Multiple detections in
one frame are processed independently. Invalid depth suppresses only that 3D
result and never creates a false point.

Phase 3 topics:

| Topic | Type | Contents/frame |
| --- | --- | --- |
| `/perception/detections_2d` | `vision_msgs/msg/Detection2DArray` | class, confidence, pixel bbox; `camera_color_optical_frame` |
| `/perception/debug_image` | `sensor_msgs/msg/Image` | optional RGB boxes/labels; `camera_color_optical_frame` |
| `/perception/geometry/camera_point` | `geometry_msgs/msg/PointStamped` | valid median-depth projection; optical frame |
| `/perception/geometry/map_point` | `geometry_msgs/msg/PointStamped` | observation-time TF result; `map` |
| `/perception/markers` | `visualization_msgs/msg/Marker` | managed target ID/class/state/filtered-position markers in detector mode |
| `/perception/objects_3d` | `robot_interfaces_perception/msg/DetectedObject3D` | Phase 4 managed map-frame target; one message per retained target per update |

Detector parameters are in `src/robot_perception/config/detector.yaml`:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `backend` | `opencv_yolox` | Replaceable backend selection |
| `model_path` | empty | Empty resolves to the verified user-cache path |
| `confidence_threshold` | `0.45` | Minimum published class confidence |
| `nms_threshold` | `0.5` | Class-aware nonmaximum suppression IoU |
| `input_size` | `640` | Square YOLOX input size |
| `device` | `auto` | `auto`, `cpu`, or OpenCV-DNN `cuda` |
| `allowed_classes` | `[person]` | Published COCO class allowlist; empty allows all |
| `debug_image_enabled` | `true` | Enable annotated debug image |

The ONNX weights are not stored in Git and normal launch never downloads a
model. Fetch and SHA-256 verify the official OpenCV Zoo model explicitly:

```bash
sudo apt install ros-jazzy-cv-bridge ros-jazzy-vision-msgs \
  python3-opencv python3-numpy
bash scripts/prepare_phase3_detector_model.sh
```

The script installs `object_detection_yolox_2022nov.onnx` under
`${XDG_CACHE_HOME:-$HOME/.cache}/robot_perception/models` by default. Override
the directory with `ROBOT_PERCEPTION_MODEL_DIR`, or pass an explicit launch
path with `detector_model_path:=/absolute/model.onnx`. The download URL may be
overridden with `ROBOT_PERCEPTION_MODEL_URL`; the same fixed SHA-256 is always
required before installation. The backend uses the
system `python3-opencv` and `python3-numpy`; CUDA is used only when requested
and available through the installed OpenCV build. An absent, truncated, or
unsupported model disables inference with an error log while the ROS node
stays alive and publishes empty, correctly stamped detection arrays.

Both Factory Patrol worlds include the visual-only
`phase3_person_detection_target`, offset from the Phase 2 center-ray target so
synthetic regression remains unchanged. Its source, license, and mechanical
texture reduction are recorded beside the model in
`src/robot_simulation/models/person_standing/ATTRIBUTION.md`.

Run the real detector path:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  gui:=false use_rviz:=false use_nav2:=false \
  use_detector:=true geometry_input_mode:=detector
```

Keep the Phase 2 regression path by leaving `use_detector:=false` and
`geometry_input_mode:=synthetic` (both defaults). These switches also allow a
different node implementing the same `Detection2DArray` contract to replace
YOLOX without changing depth projection.

Validation commands:

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

Inference latency is measured around actual backend inference and logged as
current and running-average milliseconds. The runtime validator requires a
real `person` detection and the resulting camera/map points and text marker;
it does not synthesize detections. Detector failures and empty scenes yield no
new 3D observations. The detector and geometry contracts remain unchanged;
Phase 4 consumes their valid map-frame result downstream. Perception events,
mission behavior, safety integration, and velocity output remain absent.

### Phase 4 Managed Targets

Phase 4 adds a ROS-independent C++ `TargetManager` after the existing validated
map-frame projection. Detector and OpenCV types remain upstream. Its generic
input contains class name, confidence, finite map-frame XYZ, source observation
timestamp, and depth validity. Invalid depth, empty class names, out-of-range
confidence, non-finite positions, future-dated observations, and out-of-order
update cycles are rejected without creating targets.

```text
Detection2DArray + depth + CameraInfo
  -> existing DepthProjector
  -> observation-time TF to map
  -> generic 3D observations (one synchronized frame)
  -> class-compatible 3D distance association
  -> confirmation + lifecycle + EMA
  -> /perception/objects_3d + /perception/markers
```

The lifecycle implemented in Phase 4 is:

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

`confirm_frames` counts compatible matches; hits do not need to be exactly
consecutive, and a dropout shorter than `lost_frames` preserves the target and
its ID. At `lost_frames` missed synchronized detector cycles, a target becomes
LOST. It remains matchable until the missed count exceeds twice
`lost_frames`, then it is removed to bound memory. A matching LOST target is
reacquired with the same ID; a previously confirmed target returns directly to
CONFIRMED. IDs increase monotonically from 1 and are not recycled within the
process lifetime.

For each update, all same-class target/observation pairs inside the full 3D
Euclidean `max_match_distance` are sorted by distance, then target ID, then
observation order. Greedy selection enforces one target per observation and one
observation per target. Additional same-class observations within the match
radius of an accepted observation are suppressed as duplicates. This is a
small deterministic spatial association policy, not appearance tracking,
DeepSORT, ByteTrack, or ReID.

The managed position is an exponential moving average:

```text
p_filtered = ema_alpha * p_new + (1 - ema_alpha) * p_previous
```

Raw latest position is retained internally for validation; the public managed
position and markers use the filtered value. Confidence uses the latest matched
detector confidence. Markers use stable IDs derived from `target_id`, display
text such as `#12 person CONFIRMED`, and rely on `DELETEALL` plus bounded marker
lifetime for clean LOST/removed-target updates.

The minimal domain interface lives in `robot_interfaces_perception`, consistent
with the repository's split custom-interface ownership. `DetectedObject3D.msg`
contains a source-observation header, stable `target_id`, class, latest
confidence, filtered map position, depth validity, and one of `TENTATIVE`,
`CONFIRMED`, `LOST`, or `PROCESSED`. `PROCESSED` is currently reachable only
through the C++ manager API; no mission consumer or ROS behavior API is added.
During `processed_cooldown_sec`, matching observations update the retained
target but do not create a new actionable identity. After cooldown, a matching
observation re-enters TENTATIVE with the same ID.

Tracking parameters are loaded from
`src/robot_perception/config/tracking.yaml`:

| Parameter | Default | Validation / behavior |
| --- | --- | --- |
| `tracking.confirm_frames` | `3` | positive matched-observation count |
| `tracking.lost_frames` | `5` | positive missed synchronized cycles |
| `tracking.max_match_distance` | `0.5` m | positive finite 3D match radius |
| `tracking.ema_alpha` | `0.4` | finite value in `(0, 1]` |
| `tracking.processed_cooldown_sec` | `10.0` s | nonnegative finite ROS-time duration |

Validation commands:

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

The Phase 4 runtime check uses the real Factory Patrol person detection and live
RGB-D/TF graph. An isolated relay supplies real, empty, and one duplicated
detection cycle to validate TENTATIVE, CONFIRMED, short dropout, LOST,
same-ID reacquisition, and duplicate suppression without changing the world.
It reports measured raw and EMA XYZ standard deviations and manager update
latency when an executed run provides enough samples; it does not manufacture
results.

The Phase 4 WSL runtime validation on 2026-08-13 completed with one real
Factory Patrol person target and stable `target_id=1`. It observed TENTATIVE,
CONFIRMED, a short dropout, LOST, same-ID reacquisition, duplicate suppression,
and managed markers. Across 15 paired stationary samples, both raw and filtered
population standard deviations were `x=0.000000 m`, `y=0.000000 m`, and
`z=0.000000 m`. This run therefore measured no EMA improvement because the raw
projected position had no observable jitter. The isolated node logged a
TargetManager update latency of `7.576 us`; detector inference remains a
separate cost and was not included in that measurement.

Known limitations: association has no velocity model or appearance information,
all classes use one 3D radius, frame-count lifecycle behavior depends on
synchronized detector cycles, and a removed target receives a new ID if later
rediscovered. `DetectedObject3D` is published once per retained target rather
than as an array. No Phase 5 mission, Nav2 approach goal, observation pose,
perception event, Safety Gate input, Phase 6 proximity policy, or `/cmd_vel`
publisher is implemented.

Current / planned boundary:

- Current in Phase 5A: world/config assets, map-generation note, demo launch
  skeleton, run script, static check script, documentation.
- Planned after Phase 5A: dynamic obstacle demo, real/reviewed factory map,
  closed-loop multi-point mission execution, localization-lost recovery demo,
  and any experiment result report.

No Gazebo, RViz, Nav2, or real robot runtime result is claimed by this document.

## Factory Patrol Scene V2 Preview

`src/robot_simulation/worlds/factory_patrol_industrial.sdf` is an independent
industrial-layout preview world. It preserves the original `factory_patrol.sdf`
baseline and keeps the same `factory_patrol` world name, robot names, topics,
frames, and mission seed semantics. The V2 file expands the visual factory floor
to 24 m x 16 m, keeps collision geometry simple, and adds a layered industrial
layout with:

- AMR dock / D01 charging visual area near the existing dock seed pose
- receiving and inbound buffer visuals on the left side
- back storage rack rows with simple rack collision boxes and visual loads
- right-side packing workcell with bins, tool board, safety boundaries
- a closed dock -> receiving -> storage -> packing -> dock inspection loop
- slow-zone hatching, waiting / stop markers, guardrails, bollards, and estop

The V2 visual loop is aligned to the existing station seed poses where practical.
If visual polish later requires moving route anchors, update only the scene asset
and documentation first; do not change navigation, chassis, safety, or task
logic to force a screenshot.

Launch V2 explicitly:

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  world_file:=$(ros2 pkg prefix robot_simulation)/share/robot_simulation/worlds/factory_patrol_industrial.sdf \
  gui:=true use_rviz:=true
```

Headless V2 smoke test:

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  world_file:=$(ros2 pkg prefix robot_simulation)/share/robot_simulation/worlds/factory_patrol_industrial.sdf \
  gui:=false use_rviz:=false
```

Basic AMR motion smoke test through the virtual RC input:

```bash
ros2 topic pub --rate 10 /virtual_rc/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 topic pub --once /virtual_rc/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Use `/virtual_rc/cmd_vel` for manual tests. `/teleop_cmd_vel` is the
`virtual_rc_node` output into `cmd_vel_mux_node`; `/cmd_vel` is the final
mux / safety output used by the Gazebo bridge and should not be the normal
manual input. Other mux inputs are `/nav2_cmd_vel` and `/tracking_cmd_vel`.
Observe `/cmd_vel_mux/active_source`, `/safety_state`, `/cmd_vel`, `/odom`, and
the Gazebo entity pose when the robot does not move:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic echo /safety_state
ros2 topic echo /cmd_vel_mux/active_source
ros2 topic info -v /cmd_vel
gz model --model mobile_robot --pose
```

## Phase 5B Factory Patrol Demo Workflows

Phase 5B turns the Phase 5A factory patrol assets into three demo workflow
entries. These are launch/script/config entry points and acceptance steps, not
claimed runtime results.

### Demo 1: Multipoint Patrol

Goal:

```text
start -> station_A -> station_B -> station_C -> dock
```

Entry points:

```bash
bash scripts/run_factory_patrol_multipoint_demo.sh
python3 scripts/print_factory_patrol_goals.py
```

Config assets:

- `src/robot_simulation/config/factory_patrol_route.yaml`
- `src/robot_simulation/config/factory_patrol_stations.yaml`
- `src/robot_simulation/config/factory_patrol_multipoint_mission.yaml`

Observed topics during a real run:

- `/navigate_sequence/current_goal`
- `/navigate_sequence/current_path`
- `/mission_runner/state`
- `/cmd_vel`
- `/odom`
- `/safety/state`
- `/safety/reason`

Expected behavior to verify in runtime logs: each waypoint is issued in order,
the robot approaches each station, and the mission returns to `dock`. Current
boundary: scripts and mission profile are present; no success rate or travel
time is filled until a real run is captured.

### Demo 2: Temporary Obstacle

Goal: place a simple temporary box obstacle near the `station_A` to `station_B`
segment and observe perception/planning/safety topics.

Entry point:

```bash
bash scripts/run_factory_patrol_obstacle_demo.sh
```

Config/model assets:

- `src/robot_simulation/config/factory_patrol_obstacle_demo.yaml`
- `src/robot_simulation/models/temporary_box_obstacle/model.sdf`
- `src/robot_simulation/models/temporary_box_obstacle/model.config`

Suggested runtime observations:

- `/scan`
- `/local_costmap/costmap`
- `/cmd_vel`
- `/safety/state`
- `/safety/reason`

Expected behavior to verify in runtime logs: the obstacle appears in scan data
and local costmap, and the controller response is visible in `/cmd_vel`. Whether
the robot slows, stops, or replans depends on the actual Nav2 runtime state.
This document does not claim the avoidance behavior has passed.

### Demo 3: Localization Lost And Recovery

Goal: inject a bad `/initialpose`, then inject a recovery `/initialpose`, and
observe localization health and safety-state linkage.

Entry point:

```bash
bash scripts/run_factory_patrol_localization_recovery_demo.sh
```

Config asset:

- `src/robot_simulation/config/factory_patrol_localization_recovery.yaml`

Expected state labels to observe if the runtime conditions trigger them:

```text
LOCALIZATION_LOST -> LOCALIZATION_RECOVERING -> LOCALIZATION_RECOVERED -> LOCALIZATION_OK
```

Observed topics:

- `/localization/health`
- `/safety/state`
- `/safety/reason`
- `/amcl_pose`
- `/tf`

Expected safety linkage: `LOCALIZATION_LOST` should be visible through
`/safety/state` and should force zero command according to Phase 4B policy. This
must be verified from a real ROS2/Nav2 run; no result is filled here.

### Checks

Static workflow check:

```bash
bash scripts/check_factory_patrol_demo_workflows.sh
```

Runtime topic check after the demo and Nav2 are running:

```bash
bash scripts/check_factory_patrol_demo_runtime.sh
```

Current in Phase 5B:

- demo workflow scripts
- multipoint mission/profile asset
- temporary obstacle config and simple SDF model
- localization recovery pose config
- runtime and static check scripts
- acceptance documentation

Planned after Phase 5B:

- real Gazebo/RViz screenshots
- navigation success-rate statistics
- dynamic pedestrians or moving obstacles
- Nav2 keepout/speed filter integration for factory zones
- automatic mission pause/resume full closed loop

No Gazebo, RViz, Nav2, localization recovery, or obstacle-avoidance runtime
success is claimed by this phase.

## Visual Perception Phase 5: Static Inspection Approach

Phase 5 connects a stable managed target to a task-owned Nav2 mission:

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

The `PerceptionEvent` interface contains `target_id`, `event_type`, class,
confidence, severity, and a stamped `target_pose`. Phase 5 declares only
`TARGET_CONFIRMED`, `INSPECTION_REQUIRED`, and `INSPECTION_COMPLETED`; it does
not implement Phase 6 person-proximity or safety-event behavior.

### Eligibility and duplicate policy

The detector continues to expose both `person` and `chair`, but the default
inspection allowlist is only `chair` with `min_confidence: 0.5`. A raw detector
frame cannot start a task. The target must first reach `CONFIRMED` through the
Phase 4 `TargetManager`. Perception emits one actionable event for that managed
target and suppresses subsequent frames. A successful task marks the target
`PROCESSED`; the existing `tracking.processed_cooldown_sec` must expire and the
target must pass through `TENTATIVE` and `CONFIRMED` again before a newer event
can be emitted.

The existing pretrained model does not reliably classify a code-native
primitive chair. Runtime validation therefore explicitly loads
`tracking_phase5_validation.yaml` and
`visual_inspection_phase5_validation.yaml`, whose allowlists contain `person`,
and reuses the existing visual-only, static `phase3_person_detection_target` at
the known world pose `(2.80, -0.75)`. This does not change the default `chair`
policy, add collision geometry, or implement person following. The target is a
fixed inspection fixture for this explicitly configured validation run. The
validation tracking profile also narrows the depth ROI and extends LOST-target
retention so the original managed ID remains available for task completion
after the robot turns or the static fixture leaves the camera view. Default
Phase 4 retention behavior is unchanged. Its processed cooldown is 120 seconds
so the same fixture cannot retrigger during the end-to-end measurement window.
The `--phase5` helper also caps CPU inference at 0.5 Hz so Nav2 action callbacks
remain responsive in WSL. The detector is still the same pretrained YOLOX
backend on live RGB images, and the default detector rate remains uncapped.
The validation profile permits one task-owned retry for a transient Nav2 result
race at arrival; the default mission profile remains `retry_count: 0`.

### Observation pose

Planning occurs in `map`. For target position `T`, current robot position `R`,
and configured `standoff_distance`, the task computes:

```text
d = normalize(R - T)
observation_position = T + d * standoff_distance
yaw = atan2(T.y - observation.y, T.x - observation.x)
```

The default `standoff_distance` is `1.2` m. The planner rejects non-finite
poses, non-map frames, invalid quaternions, and non-positive standoff values.
If `R` and `T` are coincident, it uses the direction opposite the robot's
current heading. The selected pose is frozen when the mission starts; target
updates or temporary detector loss do not replace the Nav2 goal. Phase 5 is for
static inspection targets only and does not implement pursuit, following, or
visual servoing.

Navigation failures publish a `FAILED` task status and never mark the target
successfully processed. The default retry count is zero. Nav2 rejection or an
unreachable pose is treated as a normal task failure rather than adding a
custom planner or costmap search.

The Factory Patrol bringup delays the Phase 5 event consumer until after the
existing delayed Nav2 bringup has completed. Because `/perception/events` is
transient-local, a target confirmed during Nav2 activation is replayed once to
the task after it starts; this avoids treating normal lifecycle startup as a
navigation failure.

### Launch and validation

Prepare the existing Phase 3 model, build, then launch the complete chain:

```bash
bash scripts/prepare_phase3_detector_model.sh
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
bash scripts/run_factory_patrol_demo.sh --phase5
```

The equivalent launch command is:

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  gui:=true use_rviz:=true use_nav2:=true use_detector:=true \
  geometry_input_mode:=detector use_visual_inspection:=true \
  perception_max_inference_rate_hz:=0.5 \
  perception_tracking_params:=$(ros2 pkg prefix --share robot_perception)/config/tracking_phase5_validation.yaml \
  visual_inspection_params:=$(ros2 pkg prefix --share robot_tasks)/config/visual_inspection_phase5_validation.yaml
```

Validate the running graph from another sourced shell:

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

The RViz showcase displays managed target labels/IDs and the observation pose;
the existing current-goal and odometry/path displays remain available. Runtime
measurements come from the validation script output.

The full headless Factory Patrol bringup was validated in WSL on 2026-08-13
with the explicit Phase 5 profiles and the live YOLOX detector. Target
`target_id=1` (`person`, validation fixture only) produced one accepted visual
inspection mission at simulation time `18.679 s`. Its measured map position was
`(2.842653, -0.759916)` m and the planned observation pose was
`(1.683362, -0.450007)` m. The configured and planned target standoffs were both
`1.2 m`. Nav2 accepted the goal and returned `SUCCEEDED` after `6.489` simulated
seconds. The final robot pose was `(1.570, -0.334, yaw=-0.421137)` with a
`0.162199 m` observation-pose error and a measured `1.342032 m` planar distance
to the target. Robot displacement was `1.605134 m`. The run observed one
`INSPECTION_REQUIRED` and one `INSPECTION_COMPLETED` for target 1, then verified
that it reached `PROCESSED` without an immediate second mission. It also
verified that perception published no velocity topic and that commands remained
`/nav2_cmd_vel -> cmd_vel_mux_node/Safety Gate -> /cmd_vel`.

## Phase 6 Showcase Boundary

Factory patrol assets are ready to support screenshots, videos, and report
figures, but Phase 6 only adds the placeholder index under `docs/showcase/`.
Future artifacts should record the exact launch command, commit, map/world,
parameters, and log or rosbag path before being cited in the README or report.

## Visual Perception Phase 6: Safety Gate Integration

Phase 6 uses the existing visual-only `phase3_person_detection_target` and the
existing Gazebo set-pose service for deterministic validation. It adds no crowd
or pedestrian framework. The standard licensed person mesh is too tall to
remain fully visible in the camera vertical field of view below the `1.5 m`
STOP threshold, so both Factory Patrol worlds also contain
`phase6_person_safety_target`: a static, collision-free `0.40` scale instance
of the same licensed mesh. It stays hidden at `z=-2` unless the Phase 6 probe
uses it for the close-range STOP case. This changes only validation image
scale; the policy still uses measured RGB-D XY distance and class `person`.

The runtime chain is:

```text
managed person (`CONFIRMED` or observed `PROCESSED`)
  -> PerceptionSafetyPolicy
  -> /perception/safety_event
  -> existing cmd_vel mux / Safety Gate
  -> /cmd_vel
```

`/perception/safety_event` has type
`robot_interfaces_perception/msg/PerceptionSafetyEvent`. It carries the target
ID/class, map position, measured robot-relative planar distance, semantic event,
severity, source/reason, and optional danger-zone ID. It is distinct from the
Phase 5 mission event. Perception NEVER publishes `/cmd_vel` or
`/nav2_cmd_vel`.

Default distance thresholds are `1.5 m` for STOP and `3.0 m` for
SPEED_LIMITED. Exact `1.5 m` and `3.0 m` boundaries are limited rather than
stopped/clear. STOP clears above `1.7 m`, SPEED_LIMITED clears above `3.2 m`,
and three valid less-restrictive observations are required. Multiple persons
resolve to the most restrictive state. The configured map-frame polygon
`factory_person_danger_zone` is:

```text
[(3.00, -1.20), (3.80, -1.20), (3.80, -0.30), (3.00, -0.30)]
```

Polygon boundaries count as inside. A person inside this zone causes STOP even
when its robot-relative distance is greater than `1.5 m`. The configuration is
in `robot_perception/config/safety_zones.yaml` and reuses the existing
`robot_navigation::ZoneCatalog` map polygon convention.

Safety Gate freshness is `1.5 s` in ROS/simulation time. A stale event removes
only perception's restriction; it does not override estop, localization,
chassis, scan, watchdog, or legacy `/safety_state` conditions. Invalid TF or
malformed person data does not create a CLEAR event.

Prepare the existing detector model and launch the Phase 6 profile:

```bash
bash scripts/prepare_phase3_detector_model.sh
bash scripts/run_factory_patrol_demo.sh --phase6
```

In another sourced shell, validate topics and the end-to-end gate:

```bash
FACTORY_PATROL_DETECTOR_MODE=true \
FACTORY_PATROL_PERCEPTION_SAFETY_MODE=true \
  bash scripts/check_factory_patrol_runtime_topics.sh

bash scripts/check_factory_patrol_perception_safety_runtime.sh
```

The runtime probe moves the visual fixture through measured CLEAR,
SPEED_LIMITED, STOP, danger-zone STOP, and recovery cases while one existing
Nav2 goal stays active. It reports real `/nav2_cmd_vel` and final `/cmd_vel`
samples plus the ROS-time STOP response latency.

A headless WSL smoke run on 2026-08-14 produced these measured results from the
live RGB-D, YOLOX, depth projection, TargetManager, policy, and Safety Gate
chain:

| Case | Measured result |
| --- | --- |
| CLEAR | Person distance `3.260 m`; perception `CLEAR`; final safety state `NORMAL` |
| SPEED_LIMITED | Person distance `2.955 m`; upstream `/nav2_cmd_vel` linear `0.350 m/s`; final `/cmd_vel` linear `0.150 m/s`; final state `SPEED_LIMITED` |
| STOP | Person distance `1.314 m`; upstream linear `0.350 m/s`; final linear `0.000 m/s`; final state `STOP` |
| Danger zone | Person map position `(3.253, -0.727) m`, distance `3.222 m`; inside `factory_person_danger_zone`; final state `STOP`; final linear velocity `0.000 m/s` |
| Recovery | Three valid clear observations at `3.260 m`; final state returned to `NORMAL`; the original Nav2 goal stayed active and final `(0.350, -0.040)` linear/angular command matched its upstream intent |

For the STOP transition, the qualifying condition timestamp was `12.079 s` in
simulation time and the first required final safe command timestamp was
`12.369 s`, an observed response latency of `0.290 s`. The safety event was
received at `12.343 s`. This is one smoke-test measurement, not a Phase 8
latency distribution.

The run also verified that perception had no `/cmd_vel` or `/nav2_cmd_vel`
publisher. A known simulation limitation is that the Gazebo `mobile_robot`
world pose barely changes while the bridged odometry integrates motion. The
probe therefore uses the existing Gazebo set-pose service to position the
visual-only person fixture and keeps one Nav2 goal active to provide real
upstream command intent; it does not claim that a physically moving world-model
robot approached the person during this smoke test.
