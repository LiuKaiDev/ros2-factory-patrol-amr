# Factory Patrol 地图生成

仓库提供 `src/robot_simulation/worlds/factory_patrol.sdf` 及配套站点、区域和路线配置，
但当前没有提交经过审阅的 Factory Patrol occupancy map。下面的流程用于生成和审查地图。

```bash
ros2 launch robot_bringup factory_patrol_demo.launch.py use_nav2:=false
ros2 launch robot_navigation slam.launch.py use_sim_time:=true
ros2 run nav2_map_server map_saver_cli -f factory_patrol
```

完成 SLAM 和 map saving 后，将生成文件放在：

```text
src/robot_navigation/maps/factory_patrol.yaml
src/robot_navigation/maps/factory_patrol.pgm
```

生成的地图必须与 `factory_patrol.sdf` 的墙体、通道和站点位置核对后，才能用于 Nav2
验收。该文件描述的是地图生成流程，不代表实体工厂地图已经记录或验证。
