from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_localization_health = LaunchConfiguration("use_localization_health")
    use_mission_runner = LaunchConfiguration("use_mission_runner")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart_mission = LaunchConfiguration("autostart_mission")
    mission_file = LaunchConfiguration("mission_file")
    rviz_config = LaunchConfiguration("rviz_config")

    sim_launch = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "launch", "sim.launch.py"]
    )
    nav_launch = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "launch", "nav.launch.py"]
    )
    localization_health_launch = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "launch", "localization_health.launch.py"]
    )
    mission_runner_launch = PathJoinSubstitution(
        [FindPackageShare("robot_tasks"), "launch", "mission_runner.launch.py"]
    )
    world_file = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "worlds", "factory_patrol.sdf"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "rviz", "factory_patrol_debug.rviz"]
    )
    nav2_params = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "config", "nav2_basic.yaml"]
    )
    nav2_map = LaunchConfiguration("nav2_map")
    default_nav2_map = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "maps", "indoor_room.yaml"]
    )
    station_file = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "config", "factory_patrol_stations.yaml"]
    )
    zones_file = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "config", "factory_patrol_zones.yaml"]
    )
    default_mission_file = PathJoinSubstitution(
        [
            FindPackageShare("robot_simulation"),
            "config",
            "factory_patrol_multipoint_mission.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_nav2", default_value="false"),
            DeclareLaunchArgument("use_localization_health", default_value="true"),
            DeclareLaunchArgument("use_mission_runner", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart_mission", default_value="false"),
            DeclareLaunchArgument("mission_file", default_value=default_mission_file),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("nav2_map", default_value=default_nav2_map),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "gui": gui,
                    "labels_enabled": "false",
                    "world_file": world_file,
                    "world_name": "factory_patrol",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_health_launch),
                condition=IfCondition(use_localization_health),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                condition=IfCondition(use_nav2),
                launch_arguments={
                    "params_file": nav2_params,
                    "map": nav2_map,
                    "use_sim_time": use_sim_time,
                    "navigation_start_delay": "5.0",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mission_runner_launch),
                condition=IfCondition(use_mission_runner),
                launch_arguments={
                    "mission_file": mission_file,
                    "autostart": autostart_mission,
                    "station_file": station_file,
                    "preflight_zones_file": zones_file,
                    "preflight_map_name": "factory_patrol",
                    "return_to_dock_on_low_battery": "false",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
