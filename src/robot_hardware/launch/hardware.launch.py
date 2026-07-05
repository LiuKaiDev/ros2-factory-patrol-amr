from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    backend = LaunchConfiguration("backend")
    sensor_source = LaunchConfiguration("sensor_source")
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
    publish_tf = LaunchConfiguration("publish_tf")
    publish_odom = LaunchConfiguration("publish_odom")
    imu_source = LaunchConfiguration("imu_source")
    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="mock"),
            DeclareLaunchArgument("sensor_source", default_value="fake"),
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
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("publish_odom", default_value="true"),
            DeclareLaunchArgument("imu_source", default_value="auto"),
            Node(
                package="robot_hardware",
                executable="chassis_driver_node",
                parameters=[
                    {
                        "backend": backend,
                        "publish_tf": publish_tf,
                        "publish_odom": publish_odom,
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
        ]
    )
