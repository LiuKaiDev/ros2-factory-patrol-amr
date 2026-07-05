from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    backend = LaunchConfiguration("backend")
    use_sim_time = LaunchConfiguration("use_sim_time")
    dev = LaunchConfiguration("dev")
    baud = LaunchConfiguration("baud")
    ip = LaunchConfiguration("ip")
    port = LaunchConfiguration("port")
    io_timeout_ms = LaunchConfiguration("io_timeout_ms")
    heartbeat_period_ms = LaunchConfiguration("heartbeat_period_ms")
    heartbeat_timeout_ms = LaunchConfiguration("heartbeat_timeout_ms")
    command_timeout_ms = LaunchConfiguration("command_timeout_ms")
    protocol = LaunchConfiguration("protocol")
    chassis_type = LaunchConfiguration("chassis_type")
    wheel_diameter_m = LaunchConfiguration("wheel_diameter_m")
    wheel_base_m = LaunchConfiguration("wheel_base_m")
    track_width_m = LaunchConfiguration("track_width_m")
    max_linear_speed_mps = LaunchConfiguration("max_linear_speed_mps")
    max_angular_speed_radps = LaunchConfiguration("max_angular_speed_radps")
    odom_pose_covariance_x = LaunchConfiguration("odom_pose_covariance_x")
    odom_pose_covariance_y = LaunchConfiguration("odom_pose_covariance_y")
    odom_pose_covariance_yaw = LaunchConfiguration("odom_pose_covariance_yaw")
    odom_twist_covariance_vx = LaunchConfiguration("odom_twist_covariance_vx")
    odom_twist_covariance_wz = LaunchConfiguration("odom_twist_covariance_wz")
    robot_description_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "urdf", "robot.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command(["xacro ", robot_description_file])
    }
    cmd_vel_stack_launch = PathJoinSubstitution(
        [FindPackageShare("robot_teleop"), "launch", "cmd_vel_stack.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="hardware"),
            DeclareLaunchArgument("backend", default_value="mock"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("dev", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("ip", default_value="192.168.1.10"),
            DeclareLaunchArgument("port", default_value="9000"),
            DeclareLaunchArgument("io_timeout_ms", default_value="500"),
            DeclareLaunchArgument("heartbeat_period_ms", default_value="500"),
            DeclareLaunchArgument("heartbeat_timeout_ms", default_value="1500"),
            DeclareLaunchArgument("command_timeout_ms", default_value="500"),
            DeclareLaunchArgument("protocol", default_value="text"),
            DeclareLaunchArgument("chassis_type", default_value="diff_drive"),
            DeclareLaunchArgument("wheel_diameter_m", default_value="0.15"),
            DeclareLaunchArgument("wheel_base_m", default_value="0.42"),
            DeclareLaunchArgument("track_width_m", default_value="0.43"),
            DeclareLaunchArgument("max_linear_speed_mps", default_value="0.6"),
            DeclareLaunchArgument("max_angular_speed_radps", default_value="1.5"),
            DeclareLaunchArgument("odom_pose_covariance_x", default_value="0.02"),
            DeclareLaunchArgument("odom_pose_covariance_y", default_value="0.02"),
            DeclareLaunchArgument("odom_pose_covariance_yaw", default_value="0.05"),
            DeclareLaunchArgument("odom_twist_covariance_vx", default_value="0.03"),
            DeclareLaunchArgument("odom_twist_covariance_wz", default_value="0.05"),
            DeclareLaunchArgument("use_twist_mux", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cmd_vel_stack_launch),
                launch_arguments={
                    "use_twist_mux": LaunchConfiguration("use_twist_mux"),
                    "use_sim_time": use_sim_time,
                    "default_source": "teleop",
                    "watchdog_timeout_ms": "500",
                }.items(),
            ),
            Node(
                package="robot_teleop",
                executable="virtual_rc_node",
                parameters=[{"manual_takeover": False}],
                output="screen",
            ),
            Node(
                package="robot_hardware",
                executable="chassis_driver_node",
                parameters=[
                    {
                        "backend": backend,
                        "mode": mode,
                        "use_sim_time": use_sim_time,
                        "serial_device": dev,
                        "serial_baud": baud,
                        "udp_host": ip,
                        "udp_port": port,
                        "io_timeout_ms": io_timeout_ms,
                        "heartbeat_period_ms": heartbeat_period_ms,
                        "heartbeat_timeout_ms": heartbeat_timeout_ms,
                        "command_timeout_ms": command_timeout_ms,
                        "protocol": protocol,
                        "chassis_type": chassis_type,
                        "wheel_diameter_m": wheel_diameter_m,
                        "wheel_base_m": wheel_base_m,
                        "track_width_m": track_width_m,
                        "max_linear_speed_mps": max_linear_speed_mps,
                        "max_angular_speed_radps": max_angular_speed_radps,
                        "odom_pose_covariance_x": odom_pose_covariance_x,
                        "odom_pose_covariance_y": odom_pose_covariance_y,
                        "odom_pose_covariance_yaw": odom_pose_covariance_yaw,
                        "odom_twist_covariance_vx": odom_twist_covariance_vx,
                        "odom_twist_covariance_wz": odom_twist_covariance_wz,
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_hardware",
                executable="odom_path_recorder_node",
                parameters=[{"input_topic": "/odom", "output_topic": "/odom/path"}],
                output="screen",
            ),
            Node(
                package="robot_sensors",
                executable="fake_scan_node",
                parameters=[{"topic": "/scan"}],
                output="screen",
            ),
            Node(
                package="robot_tasks",
                executable="navigate_sequence_server_node",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="robot_utils",
                executable="system_monitor_node",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("robot_utils"),
                            "config",
                            "diagnostic_aggregator.yaml",
                        ]
                    ),
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
            ),
            Node(
                package="robot_utils",
                executable="fault_supervisor_node",
                parameters=[
                    {"use_sim_time": use_sim_time, "auto_clear": True, "startup_grace_ms": 3000}
                ],
                output="screen",
            ),
        ]
    )
