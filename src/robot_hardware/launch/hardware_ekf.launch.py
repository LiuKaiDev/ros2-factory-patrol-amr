from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    backend = LaunchConfiguration("backend")
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
    imu_source = LaunchConfiguration("imu_source")
    ekf_params_file = LaunchConfiguration("ekf_params_file")
    ekf_output_topic = LaunchConfiguration("ekf_output_topic")
    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="mock"),
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
            DeclareLaunchArgument("imu_source", default_value="auto"),
            DeclareLaunchArgument(
                "ekf_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_hardware"), "config", "ekf.yaml"]
                ),
            ),
            DeclareLaunchArgument("ekf_output_topic", default_value="/odom"),
            Node(
                package="robot_hardware",
                executable="chassis_driver_node",
                parameters=[
                    {
                        "backend": backend,
                        "publish_tf": False,
                        "publish_odom": False,
                        "imu_source": imu_source,
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
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[ekf_params_file],
                remappings=[("odometry/filtered", ekf_output_topic)],
                output="screen",
            ),
            Node(
                package="robot_hardware",
                executable="odom_path_recorder_node",
                parameters=[{"input_topic": ekf_output_topic, "output_topic": "/odom/path"}],
                output="screen",
            ),
            Node(
                package="robot_sensors",
                executable="fake_scan_node",
                parameters=[{"topic": "/scan"}],
                output="screen",
            ),
        ]
    )
