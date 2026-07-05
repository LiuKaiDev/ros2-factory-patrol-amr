from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_twist_mux = LaunchConfiguration("use_twist_mux")
    use_sim_time = LaunchConfiguration("use_sim_time")
    default_source = LaunchConfiguration("default_source")
    watchdog_timeout_ms = LaunchConfiguration("watchdog_timeout_ms")
    dynamic_angular_limit_radps = LaunchConfiguration("dynamic_angular_limit_radps")
    localization_health_topic = LaunchConfiguration("localization_health_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    chassis_state_topic = LaunchConfiguration("chassis_state_topic")
    safety_state_topic = LaunchConfiguration("safety_state_topic")
    safety_reason_topic = LaunchConfiguration("safety_reason_topic")
    scan_timeout_sec = LaunchConfiguration("scan_timeout_sec")
    odom_timeout_sec = LaunchConfiguration("odom_timeout_sec")
    localization_lost_stop = LaunchConfiguration("localization_lost_stop")
    sensor_stale_stop = LaunchConfiguration("sensor_stale_stop")
    chassis_fault_stop = LaunchConfiguration("chassis_fault_stop")
    communication_lost_stop = LaunchConfiguration("communication_lost_stop")
    emergency_stop_requires_reset = LaunchConfiguration("emergency_stop_requires_reset")
    speed_limited_max_linear_mps = LaunchConfiguration("speed_limited_max_linear_mps")
    speed_limited_max_angular_radps = LaunchConfiguration("speed_limited_max_angular_radps")
    twist_mux_config = LaunchConfiguration("twist_mux_config")
    mux_output_topic = LaunchConfiguration("mux_output_topic")

    default_twist_mux_config = PathJoinSubstitution(
        [FindPackageShare("robot_teleop"), "config", "twist_mux.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_twist_mux", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("default_source", default_value="teleop"),
            DeclareLaunchArgument("watchdog_timeout_ms", default_value="500"),
            DeclareLaunchArgument("dynamic_angular_limit_radps", default_value="0.6"),
            DeclareLaunchArgument("localization_health_topic", default_value="/localization/health"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("chassis_state_topic", default_value="/chassis/state"),
            DeclareLaunchArgument("safety_state_topic", default_value="/safety/state"),
            DeclareLaunchArgument("safety_reason_topic", default_value="/safety/reason"),
            DeclareLaunchArgument("scan_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("odom_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("localization_lost_stop", default_value="true"),
            DeclareLaunchArgument("sensor_stale_stop", default_value="true"),
            DeclareLaunchArgument("chassis_fault_stop", default_value="true"),
            DeclareLaunchArgument("communication_lost_stop", default_value="true"),
            DeclareLaunchArgument("emergency_stop_requires_reset", default_value="true"),
            DeclareLaunchArgument("speed_limited_max_linear_mps", default_value="0.15"),
            DeclareLaunchArgument("speed_limited_max_angular_radps", default_value="0.4"),
            DeclareLaunchArgument("twist_mux_config", default_value=default_twist_mux_config),
            DeclareLaunchArgument("mux_output_topic", default_value="/muxed_cmd_vel"),
            Node(
                package="twist_mux",
                executable="twist_mux",
                condition=IfCondition(use_twist_mux),
                remappings=[("/cmd_vel_out", mux_output_topic)],
                parameters=[twist_mux_config, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="robot_teleop",
                executable="cmd_vel_safety_gate_node",
                condition=IfCondition(use_twist_mux),
                parameters=[
                    {
                        "input_topic": mux_output_topic,
                        "default_source": default_source,
                        "watchdog_timeout_ms": watchdog_timeout_ms,
                        "dynamic_angular_limit_radps": dynamic_angular_limit_radps,
                        "localization_health_topic": localization_health_topic,
                        "scan_topic": scan_topic,
                        "odom_topic": odom_topic,
                        "chassis_state_topic": chassis_state_topic,
                        "safety_state_topic": safety_state_topic,
                        "safety_reason_topic": safety_reason_topic,
                        "scan_timeout_sec": scan_timeout_sec,
                        "odom_timeout_sec": odom_timeout_sec,
                        "localization_lost_stop": localization_lost_stop,
                        "sensor_stale_stop": sensor_stale_stop,
                        "chassis_fault_stop": chassis_fault_stop,
                        "communication_lost_stop": communication_lost_stop,
                        "emergency_stop_requires_reset": emergency_stop_requires_reset,
                        "speed_limited_max_linear_mps": speed_limited_max_linear_mps,
                        "speed_limited_max_angular_radps": speed_limited_max_angular_radps,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_teleop",
                executable="cmd_vel_mux_node",
                condition=UnlessCondition(use_twist_mux),
                parameters=[
                    {
                        "default_source": default_source,
                        "watchdog_timeout_ms": watchdog_timeout_ms,
                        "dynamic_angular_limit_radps": dynamic_angular_limit_radps,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
        ]
    )
