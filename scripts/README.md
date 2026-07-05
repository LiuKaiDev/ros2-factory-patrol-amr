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
bash scripts/check_factory_patrol_demo_runtime.sh
```

## Final Readiness

```bash
bash scripts/check_project_showcase_readiness.sh
```

This final readiness script checks documentation, CI, showcase placeholders, and
major validation entry points. It does not claim runtime success.
