# Factory Patrol Map Placeholder

Phase 5A adds `src/robot_simulation/worlds/factory_patrol.sdf` and matching
station/zone/route configuration assets. It does not add a measured occupancy
map for this world.

Recommended follow-up flow:

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py use_nav2:=false
ros2 launch robot_navigation slam.launch.py use_sim_time:=true
ros2 run nav2_map_server map_saver_cli -f factory_patrol
```

After SLAM/map saving, place the generated files here as:

```text
src/robot_navigation/maps/factory_patrol.yaml
src/robot_navigation/maps/factory_patrol.pgm
```

Any generated map should be reviewed against `factory_patrol.sdf` before using it
for Nav2 acceptance. This README is not a claim that a real map has been
recorded or validated.
