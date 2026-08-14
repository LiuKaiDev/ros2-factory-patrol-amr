from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_localization_health = LaunchConfiguration("use_localization_health")
    use_mission_runner = LaunchConfiguration("use_mission_runner")
    use_geometry_validation = LaunchConfiguration("use_geometry_validation")
    use_detector = LaunchConfiguration("use_detector")
    use_visual_inspection = LaunchConfiguration("use_visual_inspection")
    use_perception_safety = LaunchConfiguration("use_perception_safety")
    geometry_input_mode = LaunchConfiguration("geometry_input_mode")
    detector_model_path = LaunchConfiguration("detector_model_path")
    perception_debug_image = LaunchConfiguration("perception_debug_image")
    perception_max_inference_rate_hz = LaunchConfiguration(
        "perception_max_inference_rate_hz"
    )
    perception_tracking_params = LaunchConfiguration("perception_tracking_params")
    perception_safety_params = LaunchConfiguration("perception_safety_params")
    perception_safety_zones = LaunchConfiguration("perception_safety_zones")
    visual_inspection_params = LaunchConfiguration("visual_inspection_params")
    visual_inspection_start_delay = LaunchConfiguration(
        "visual_inspection_start_delay"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart_mission = LaunchConfiguration("autostart_mission")
    mission_file = LaunchConfiguration("mission_file")
    rviz_config = LaunchConfiguration("rviz_config")
    world_file = LaunchConfiguration("world_file")

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
    perception_launch = PathJoinSubstitution(
        [FindPackageShare("robot_perception"), "launch", "perception.launch.py"]
    )
    default_tracking_params = PathJoinSubstitution(
        [FindPackageShare("robot_perception"), "config", "tracking.yaml"]
    )
    default_visual_inspection_params = PathJoinSubstitution(
        [FindPackageShare("robot_tasks"), "config", "visual_inspection.yaml"]
    )
    default_safety_params = PathJoinSubstitution(
        [FindPackageShare("robot_perception"), "config", "safety.yaml"]
    )
    default_safety_zones = PathJoinSubstitution(
        [FindPackageShare("robot_perception"), "config", "safety_zones.yaml"]
    )
    default_world_file = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "worlds", "factory_patrol.sdf"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "rviz", "factory_patrol_showcase.rviz"]
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
            DeclareLaunchArgument("use_geometry_validation", default_value="true"),
            DeclareLaunchArgument("use_detector", default_value="false"),
            DeclareLaunchArgument("use_visual_inspection", default_value="false"),
            DeclareLaunchArgument("use_perception_safety", default_value="false"),
            DeclareLaunchArgument("geometry_input_mode", default_value="synthetic"),
            DeclareLaunchArgument("detector_model_path", default_value=""),
            DeclareLaunchArgument("perception_debug_image", default_value="true"),
            DeclareLaunchArgument(
                "perception_max_inference_rate_hz", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "perception_tracking_params", default_value=default_tracking_params
            ),
            DeclareLaunchArgument(
                "perception_safety_params", default_value=default_safety_params
            ),
            DeclareLaunchArgument(
                "perception_safety_zones", default_value=default_safety_zones
            ),
            DeclareLaunchArgument(
                "visual_inspection_params", default_value=default_visual_inspection_params
            ),
            DeclareLaunchArgument("visual_inspection_start_delay", default_value="10.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart_mission", default_value="false"),
            DeclareLaunchArgument("mission_file", default_value=default_mission_file),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("nav2_map", default_value=default_nav2_map),
            DeclareLaunchArgument(
                "world_file",
                default_value=default_world_file,
                description=(
                    "Gazebo world file. Override with factory_patrol_industrial.sdf "
                    "to preview the Scene V2 industrial layout."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "gui": gui,
                    "labels_enabled": "false",
                    "world_file": world_file,
                    "world_name": "factory_patrol",
                    "cmd_vel_default_source": PythonExpression(
                        [
                            "'nav2' if ('",
                            use_visual_inspection,
                            "' == 'true' or '",
                            use_perception_safety,
                            "' == 'true') else 'teleop'",
                        ]
                    ),
                    "manual_takeover": PythonExpression(
                        [
                            "'false' if ('",
                            use_visual_inspection,
                            "' == 'true' or '",
                            use_perception_safety,
                            "' == 'true') else 'true'",
                        ]
                    ),
                    "perception_safety_enabled": use_perception_safety,
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception_launch),
                condition=IfCondition(use_geometry_validation),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "publish_sim_map_tf": PythonExpression(
                        ["'", use_nav2, "' == 'false'"]
                    ),
                    "detector_enabled": use_detector,
                    "geometry_input_mode": geometry_input_mode,
                    "model_path": detector_model_path,
                    "debug_image_enabled": perception_debug_image,
                    "max_inference_rate_hz": perception_max_inference_rate_hz,
                    "tracking_params_file": perception_tracking_params,
                    "safety_params_file": perception_safety_params,
                    "safety_zones_file": perception_safety_zones,
                }.items(),
            ),
            Node(
                package="robot_tasks",
                executable="navigate_sequence_server_node",
                condition=IfCondition(use_visual_inspection),
                parameters=[
                    {
                        "use_nav2_action": use_nav2,
                        "use_waypoint_follower": False,
                        "simulate_without_nav2": False,
                        "simulated_motion_enabled": False,
                        "nav2_server_timeout_ms": 15000,
                    }
                ],
                output="screen",
            ),
            TimerAction(
                period=visual_inspection_start_delay,
                condition=IfCondition(use_visual_inspection),
                actions=[
                    Node(
                        package="robot_tasks",
                        executable="visual_inspection_task_node",
                        parameters=[
                            visual_inspection_params,
                            {"use_sim_time": use_sim_time},
                        ],
                        output="screen",
                    )
                ],
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
