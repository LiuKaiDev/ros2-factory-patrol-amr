# Server Docker Validation Summary

This document records low-resource server Docker validation. It is build, test,
and static-check evidence only; it does not claim real robot deployment or real
factory operation.

## Environment

| Item | Value |
| --- | --- |
| Host | Alibaba Cloud Linux 3 |
| Container image | `ros:jazzy-ros-base` |
| ROS distribution | Jazzy |
| Hardware limitation | About 1.8 GB RAM |

## Partial Build And Test Result

The server completed partial workspace validation while excluding `robot_tasks`
because the server memory limit killed the C++ compiler during that package.

```text
17 packages finished
104 tests, 0 errors, 0 failures, 0 skipped
```

## Static Checks

The following static validation scripts passed in the server Docker validation:

- `scripts/check_project_showcase_readiness.sh`
- `scripts/check_nav2_costmap_obstacle_layer.sh`
- `scripts/check_chassis_protocol_v2.sh`
- `scripts/check_chassis_odom_calibration.sh`
- `scripts/check_localization_health.sh`
- `scripts/check_safety_state_machine.sh`
- `scripts/check_factory_patrol_assets.sh`
- `scripts/check_factory_patrol_demo_workflows.sh`

## Known Limitation

`robot_tasks` failed on the low-resource server because
`mission_runner_node.cpp` triggered a `cc1plus` out-of-memory kill. Kernel logs
confirmed:

```text
Out of memory: Killed process cc1plus
```

This was an environment resource limitation on an approximately 1.8 GB RAM
server, not a syntax-level build failure.

## Validation Scope

This validation supports confidence in the main workspace subset, static checks,
and test coverage under a constrained Docker environment. It does not replace
the local WSL2 full validation or any future real robot validation.
