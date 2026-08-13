from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    detector_enabled = LaunchConfiguration("detector_enabled")
    geometry_input_mode = LaunchConfiguration("geometry_input_mode")
    publish_sim_map_tf = LaunchConfiguration("publish_sim_map_tf")
    model_path = LaunchConfiguration("model_path")
    debug_image_enabled = LaunchConfiguration("debug_image_enabled")
    tracking_params_file = LaunchConfiguration("tracking_params_file")
    detector_params = PathJoinSubstitution(
        [FindPackageShare("robot_perception"), "config", "detector.yaml"]
    )
    geometry_launch = PathJoinSubstitution(
        [FindPackageShare("robot_perception"), "launch", "geometry_validation.launch.py"]
    )
    default_tracking_params = PathJoinSubstitution(
        [FindPackageShare("robot_perception"), "config", "tracking.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("detector_enabled", default_value="false"),
            DeclareLaunchArgument("geometry_input_mode", default_value="synthetic"),
            DeclareLaunchArgument("publish_sim_map_tf", default_value="true"),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument("debug_image_enabled", default_value="true"),
            DeclareLaunchArgument(
                "tracking_params_file", default_value=default_tracking_params
            ),
            Node(
                package="robot_perception",
                executable="detector_node",
                condition=IfCondition(detector_enabled),
                parameters=[
                    detector_params,
                    {
                        "use_sim_time": use_sim_time,
                        "model_path": model_path,
                        "debug_image_enabled": debug_image_enabled,
                    },
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(geometry_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "publish_sim_map_tf": publish_sim_map_tf,
                    "geometry_input_mode": geometry_input_mode,
                    "tracking_params_file": tracking_params_file,
                }.items(),
            ),
        ]
    )
