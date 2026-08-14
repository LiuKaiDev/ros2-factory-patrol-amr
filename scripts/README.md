# Scripts

Scripts are grouped by purpose. Static checks can run without a live ROS2 graph.
Runtime checks require ROS2 topics to exist.

## Nav2

```bash
bash scripts/run_nav2_basic_demo.sh
bash scripts/check_nav2_costmap_obstacle_layer.sh
bash scripts/check_nav2_runtime_topics.sh
```

## Tracking Experiments

```bash
bash scripts/run_tracking_experiment_demo.sh
bash scripts/run_tracking_analysis_workflow.sh <tracking.csv>
python3 scripts/analyze_tracking_result.py <tracking.csv>
python3 scripts/compare_tracking_results.py --format markdown <a.csv> <b.csv>
python3 scripts/plot_tracking_result.py <tracking.csv> --output-dir src/robot_experiments/results/figures
```

## Chassis And Calibration

```bash
bash scripts/check_chassis_protocol_v2.sh
bash scripts/check_chassis_odom_calibration.sh
```

## Localization And Safety

```bash
bash scripts/check_localization_health.sh
bash scripts/check_localization_runtime_topics.sh
bash scripts/check_safety_state_machine.sh
bash scripts/check_safety_runtime_topics.sh
```

## Factory Patrol

```bash
bash scripts/run_factory_patrol_demo.sh
bash scripts/check_factory_patrol_assets.sh
bash scripts/run_factory_patrol_multipoint_demo.sh
python3 scripts/print_factory_patrol_goals.py
bash scripts/run_factory_patrol_obstacle_demo.sh
bash scripts/run_factory_patrol_localization_recovery_demo.sh
bash scripts/check_factory_patrol_demo_workflows.sh
bash scripts/check_factory_patrol_runtime_topics.sh
bash scripts/prepare_phase3_detector_model.sh
bash scripts/check_factory_patrol_detector_runtime.sh
bash scripts/check_factory_patrol_target_manager_runtime.sh
bash scripts/check_factory_patrol_visual_inspection_runtime.sh
bash scripts/check_factory_patrol_perception_safety_runtime.sh
bash scripts/check_factory_patrol_demo_runtime.sh
```

`run_factory_patrol_demo.sh` reports the default
`src/robot_simulation/rviz/factory_patrol_showcase.rviz` layout for the
non-Nav2 Gazebo/RViz showcase. `check_factory_patrol_runtime_topics.sh` requires
a running ROS2 graph and prints topic counts, `/scan` QoS hints, sampled frame
IDs, and odom TF connectivity diagnostics.

`prepare_phase3_detector_model.sh` explicitly downloads and verifies the
official OpenCV Zoo YOLOX-S model into the user cache. Normal ROS launch never
downloads weights. With the detector-mode demo running,
`check_factory_patrol_detector_runtime.sh` validates the real 2D-to-3D chain.
`check_factory_patrol_target_manager_runtime.sh` reuses that live detector,
depth, CameraInfo, and TF graph to validate stable IDs, lifecycle transitions,
duplicate suppression, markers, and raw-versus-filtered position statistics.
With the explicit Phase 5 validation profiles loaded,
`check_factory_patrol_visual_inspection_runtime.sh` validates one accepted
task-owned Nav2 approach, observation standoff and yaw, robot motion,
completion feedback, the target's `PROCESSED` state, and the unchanged
mux/Safety Gate velocity path.

`run_factory_patrol_demo.sh --phase6` starts the live detector, the managed
person safety policy, Nav2, and the existing combined mux/Safety Gate.
`check_factory_patrol_perception_safety_runtime.sh` moves the existing
visual-only person fixture through distance and map-zone cases, then compares
real `/nav2_cmd_vel` intent with final `/cmd_vel`, measures STOP response time,
and verifies recovery without sending any Twist from perception.

To preview the independent Factory Patrol Scene V2 industrial world:

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py \
  world_file:=$(ros2 pkg prefix robot_simulation)/share/robot_simulation/worlds/factory_patrol_industrial.sdf \
  gui:=true use_rviz:=true
```

Manual motion smoke tests should publish to `/virtual_rc/cmd_vel`, not directly
to `/cmd_vel`:

```bash
ros2 topic pub --rate 10 /virtual_rc/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 topic pub --once /virtual_rc/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

`/teleop_cmd_vel` is the `virtual_rc_node` output into the mux. `/cmd_vel` is
the final mux / safety output consumed by the Gazebo bridge. Useful checks are
`/cmd_vel`, `/odom`, `/safety_state`, `/cmd_vel_mux/active_source`, and
`gz model --model mobile_robot --pose`.

## Final Readiness

```bash
bash scripts/check_project_showcase_readiness.sh
```

This final readiness script checks documentation, CI, showcase placeholders, and
major validation entry points. It does not claim runtime success.
