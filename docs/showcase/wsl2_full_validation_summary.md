# WSL2 Full Validation Summary

This document records real local ROS2 Jazzy validation performed in WSL2. It
does not claim real robot deployment, physical factory operation, or Gazebo/RViz
showcase screenshots.

## Environment

| Item | Value |
| --- | --- |
| Platform | WSL2 |
| OS | Ubuntu 24.04 LTS |
| ROS distribution | Jazzy |
| ROS package set | `ros-jazzy-desktop` |
| Dependencies | Project dependencies installed through `rosdep` |

## Build Result

The full workspace build completed locally in the WSL2 environment.

## Test Result

Final aggregated result:

```text
514 tests, 0 errors, 0 failures, 0 skipped
```

## robot_navigation Retry Note

A previous full test run had one transient `/map` message timeout in
`robot_navigation/test_nav2_mock_runtime.launch.py`. The `robot_navigation`
package was re-run independently and passed:

```text
100% tests passed, 0 tests failed out of 6
```

The final aggregated `colcon test-result` reported all tests passing. The likely
reason for the earlier timeout was WSL2/Nav2 runtime timing or DDS topic timing,
not a syntax-level build issue.

## RViz And Gazebo Runtime Validation

RViz launched successfully in the WSL2 ROS2 Jazzy environment.

Gazebo Sim launched successfully after replacing the temporary Factory Patrol
label placeholders with lightweight text mesh assets under
`src/robot_simulation/media/text_meshes/`. The `factory_patrol.sdf` world is now
visible in Gazebo with the label visuals kept in place.

The current default RViz configuration for this demo is
`src/robot_simulation/rviz/factory_patrol_showcase.rviz`. The previous
debug-oriented config remains available at
`src/robot_simulation/rviz/factory_patrol_debug.rviz`.

## Runtime Topics Observed

The Factory Patrol runtime graph exposed the expected topics:

```text
/map
/scan
/odom
/cmd_vel
/tf
/joint_states
/mission_runner/state
/safety_state
/localization/health
/global_costmap/costmap
/local_costmap/costmap
/amr_simulation/markers
/amr_simulation/demo_timeline
```

## Concise Evidence Logs

- `docs/showcase/logs/test_result_verbose_full_wsl2.txt`
- `docs/showcase/logs/robot_navigation_retry_result.txt`
- `docs/showcase/logs/factory_patrol_topic_list_wsl2.txt`
- `docs/showcase/logs/factory_patrol_node_list_wsl2.txt`

## Validation Scope

This validation covers the full local ROS2 Jazzy workspace build and test suite
in WSL2, plus RViz/Gazebo launch validation for the Factory Patrol simulation
world. It does not prove long-duration field operation, real chassis behavior,
real factory operation, or physical safety performance.
