from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_pose_topic = LaunchConfiguration("amcl_pose_topic")
    initialpose_topic = LaunchConfiguration("initialpose_topic")
    health_topic = LaunchConfiguration("health_topic")
    legacy_health_topic = LaunchConfiguration("legacy_health_topic")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    amcl_timeout_sec = LaunchConfiguration("amcl_timeout_sec")
    tf_timeout_sec = LaunchConfiguration("tf_timeout_sec")
    covariance_warn_xy = LaunchConfiguration("covariance_warn_xy")
    covariance_lost_xy = LaunchConfiguration("covariance_lost_xy")
    covariance_warn_yaw = LaunchConfiguration("covariance_warn_yaw")
    covariance_lost_yaw = LaunchConfiguration("covariance_lost_yaw")
    lost_hold_time_sec = LaunchConfiguration("lost_hold_time_sec")
    recovered_hold_time_sec = LaunchConfiguration("recovered_hold_time_sec")
    publish_period_ms = LaunchConfiguration("publish_period_ms")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("amcl_pose_topic", default_value="/amcl_pose"),
            DeclareLaunchArgument("initialpose_topic", default_value="/initialpose"),
            DeclareLaunchArgument("health_topic", default_value="/localization/health"),
            DeclareLaunchArgument("legacy_health_topic", default_value="/localization_health"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("amcl_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("tf_timeout_sec", default_value="0.5"),
            DeclareLaunchArgument("covariance_warn_xy", default_value="0.25"),
            DeclareLaunchArgument("covariance_lost_xy", default_value="0.8"),
            DeclareLaunchArgument("covariance_warn_yaw", default_value="0.25"),
            DeclareLaunchArgument("covariance_lost_yaw", default_value="0.8"),
            DeclareLaunchArgument("lost_hold_time_sec", default_value="2.0"),
            DeclareLaunchArgument("recovered_hold_time_sec", default_value="1.0"),
            DeclareLaunchArgument("publish_period_ms", default_value="200"),
            Node(
                package="robot_navigation",
                executable="localization_health_monitor_node",
                name="localization_health_monitor_node",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "amcl_pose_topic": amcl_pose_topic,
                        "initialpose_topic": initialpose_topic,
                        "health_topic": health_topic,
                        "legacy_health_topic": legacy_health_topic,
                        "map_frame": map_frame,
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,
                        "amcl_timeout_sec": amcl_timeout_sec,
                        "tf_timeout_sec": tf_timeout_sec,
                        "covariance_warn_xy": covariance_warn_xy,
                        "covariance_lost_xy": covariance_lost_xy,
                        "covariance_warn_yaw": covariance_warn_yaw,
                        "covariance_lost_yaw": covariance_lost_yaw,
                        "lost_hold_time_sec": lost_hold_time_sec,
                        "recovered_hold_time_sec": recovered_hold_time_sec,
                        "publish_period_ms": publish_period_ms,
                    }
                ],
                output="screen",
            ),
        ]
    )
