from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_sim_map_tf = LaunchConfiguration("publish_sim_map_tf")
    params_file = LaunchConfiguration("params_file")
    tracking_params_file = LaunchConfiguration("tracking_params_file")
    geometry_input_mode = LaunchConfiguration("geometry_input_mode")
    default_params = PathJoinSubstitution(
        [FindPackageShare("robot_perception"), "config", "depth.yaml"]
    )
    default_tracking_params = PathJoinSubstitution(
        [FindPackageShare("robot_perception"), "config", "tracking.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "publish_sim_map_tf",
                default_value="true",
                description=(
                    "Publish identity map->odom for Factory Patrol geometry validation "
                    "when AMCL/Nav2 is not running."
                ),
            ),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument(
                "tracking_params_file", default_value=default_tracking_params
            ),
            DeclareLaunchArgument(
                "geometry_input_mode",
                default_value="synthetic",
                description="Geometry source: synthetic or detector.",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="geometry_validation_map_to_odom_tf",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(publish_sim_map_tf),
                output="screen",
            ),
            Node(
                package="robot_perception",
                executable="geometry_validation_node",
                parameters=[
                    params_file,
                    tracking_params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "geometry_input_mode": geometry_input_mode,
                    },
                ],
                output="screen",
            ),
        ]
    )
