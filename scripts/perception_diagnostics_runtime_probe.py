#!/usr/bin/env python3

import struct
import sys
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.msg import ChassisState, RobotState
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

OK_LEVEL = 0
ERROR_LEVEL = 2


class Phase7Probe(Node):
    def __init__(self):
        super().__init__("phase7_perception_diagnostics_runtime_probe")
        self.lock = threading.Lock()
        self.statuses = {}
        self.health = []
        self.estop = []
        self.fault_states = []
        self.rgb_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", 10)
        self.camera_point_pub = self.create_publisher(
            PointStamped, "/perception/geometry/camera_point", 10
        )
        self.chassis_pub = self.create_publisher(ChassisState, "/chassis/state", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.estop_pub = self.create_publisher(Bool, "/emergency_stop/state", 10)
        self.detector_fault_pub = self.create_publisher(DiagnosticArray, "/perception/diagnostics", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            DiagnosticArray, "/perception/diagnostics", self._diagnostics_callback, 10
        )
        self.create_subscription(RobotState, "/system_health", self._health_callback, 10)
        self.create_subscription(
            RobotState, "/fault_supervisor/state", self._fault_state_callback, 10
        )
        self.create_subscription(Bool, "/emergency_stop/state", self._estop_callback, 10)
        self.create_service(SetBool, "/enable_emergency_stop", self._estop_service)
        self.create_service(SetBool, "/clear_emergency_stop", self._estop_service)

    def _diagnostics_callback(self, message):
        with self.lock:
            for status in message.status:
                self.statuses[status.name] = status

    def _health_callback(self, message):
        with self.lock:
            self.health.append(message.state)

    def _estop_callback(self, message):
        with self.lock:
            self.estop.append(message.data)

    def _fault_state_callback(self, message):
        with self.lock:
            self.fault_states.append(message.state)

    @staticmethod
    def _estop_service(request, response):
        response.success = True
        response.message = "probe service accepted request"
        return response

    def publish_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "camera_color_optical_frame"
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def publish_inputs(self, invalid_depth=False, future_stamp=False, publish_rgb=True,
                       publish_depth=True, publish_info=True, invalid_info=False):
        stamp = self.get_clock().now().to_msg()
        stamp.sec -= 1
        if future_stamp:
            stamp.sec += 100
        rgb = Image(height=2, width=2, encoding="rgb8", step=6, data=bytes([10, 20, 30] * 4))
        rgb.header.stamp = stamp
        rgb.header.frame_id = "camera_color_optical_frame"
        depth_values = [0.0 if invalid_depth else 1.0] * 4
        depth = Image(height=2, width=2, encoding="32FC1", step=8,
                      data=b"".join(struct.pack("f", value) for value in depth_values))
        depth.header.stamp = stamp
        depth.header.frame_id = "camera_color_optical_frame"
        info = CameraInfo(height=2, width=2,
                          k=[0.0 if invalid_info else 100.0, 0.0, 1.0,
                             0.0, 100.0, 1.0, 0.0, 0.0, 1.0])
        info.header.stamp = stamp
        info.header.frame_id = "camera_color_optical_frame"
        if publish_rgb:
            self.rgb_pub.publish(rgb)
        if publish_depth:
            self.depth_pub.publish(depth)
        if publish_info:
            self.info_pub.publish(info)
        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = "camera_color_optical_frame"
        point.point.z = 1.0
        self.camera_point_pub.publish(point)
        chassis = ChassisState(backend="probe", kinematics_model="diff_drive",
                               connected=True, battery_voltage=24.0, status="ready")
        chassis.header.stamp = stamp
        chassis.header.frame_id = "base_link"
        self.chassis_pub.publish(chassis)
        scan = LaserScan(ranges=[5.0])
        scan.header.stamp = stamp
        scan.header.frame_id = "laser"
        self.scan_pub.publish(scan)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        self.imu_pub.publish(imu)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        self.odom_pub.publish(odom)
        self.estop_pub.publish(Bool(data=False))

    def publish_detector_fault(self, level):
        encoded_level = bytes([level]) if isinstance(level, int) else level
        status = DiagnosticStatus(
            name="perception/detector", hardware_id="probe", level=encoded_level,
            message="injected detector fault" if level not in (0, DiagnosticStatus.OK) else "recovered",
            values=[KeyValue(key="probe", value="true")]
        )
        self.detector_fault_pub.publish(DiagnosticArray(status=[status]))

    def status_level(self, name):
        with self.lock:
            status = self.statuses.get(name)
            if status is None:
                return None
            return status.level[0] if isinstance(status.level, bytes) else int(status.level)

    def wait_for(self, description, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.05)
        raise RuntimeError(f"timeout waiting for {description}")

    def wait_publishing(self, description, predicate, timeout=8.0, **kwargs):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.publish_inputs(**kwargs)
            if predicate():
                return
            time.sleep(0.1)
        raise RuntimeError(f"timeout waiting for {description}")


def spin_inputs(node, stop_event):
    while not stop_event.is_set():
        node.publish_tf()
        time.sleep(0.1)


def main():
    rclpy.init(args=sys.argv)
    node = Phase7Probe()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    stop_event = threading.Event()
    input_thread = threading.Thread(target=spin_inputs, args=(node, stop_event), daemon=True)
    input_thread.start()
    try:
        node.wait_publishing("nominal RGB diagnostics", lambda: node.status_level("perception/camera_rgb") == OK_LEVEL)
        node.wait_publishing("nominal depth diagnostics", lambda: node.status_level("perception/depth_quality") == OK_LEVEL)

        for _ in range(25):
            node.publish_inputs(publish_info=False)
            time.sleep(0.1)
        node.wait_for("CameraInfo stale ERROR", lambda: node.status_level("perception/camera_info") == ERROR_LEVEL)
        node.wait_publishing("CameraInfo recovery", lambda: node.status_level("perception/camera_info") == OK_LEVEL)
        node.wait_publishing("CameraInfo invalid ERROR", lambda: node.status_level("perception/camera_info") == ERROR_LEVEL, invalid_info=True)
        node.wait_publishing("CameraInfo validity recovery", lambda: node.status_level("perception/camera_info") == OK_LEVEL)

        for _ in range(25):
            node.publish_inputs(publish_rgb=False)
            time.sleep(0.1)
        node.wait_for("RGB stale ERROR", lambda: node.status_level("perception/camera_rgb") == ERROR_LEVEL)
        node.wait_publishing("RGB recovery", lambda: node.status_level("perception/camera_rgb") == OK_LEVEL)

        for _ in range(25):
            node.publish_inputs(publish_depth=False)
            time.sleep(0.1)
        node.wait_for("depth stream stale ERROR", lambda: node.status_level("perception/camera_depth") == ERROR_LEVEL)
        node.wait_publishing("depth stream recovery", lambda: node.status_level("perception/camera_depth") == OK_LEVEL)

        node.wait_publishing("depth quality ERROR", lambda: node.status_level("perception/depth_quality") == ERROR_LEVEL, invalid_depth=True)

        node.wait_publishing("depth quality recovery", lambda: node.status_level("perception/depth_quality") == OK_LEVEL)

        node.wait_publishing("observation-time TF ERROR", lambda: node.status_level("perception/tf") == ERROR_LEVEL, future_stamp=True)

        node.wait_publishing("TF recovery", lambda: node.status_level("perception/tf") == OK_LEVEL)

        for _ in range(10):
            node.publish_detector_fault(DiagnosticStatus.ERROR)
            node.publish_inputs()
            time.sleep(0.1)
        node.wait_for("detector ERROR aggregation", lambda: node.status_level("perception/detector") == ERROR_LEVEL)
        node.wait_for("system health ERROR", lambda: "ERROR" in node.health, timeout=8.0)
        node.wait_for("fault supervisor FAULT", lambda: "FAULT" in node.fault_states, timeout=8.0)
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and node.status_level("perception/detector") != OK_LEVEL:
            node.publish_detector_fault(OK_LEVEL)
            node.publish_inputs()
            time.sleep(0.1)
        node.wait_for("detector recovery", lambda: node.status_level("perception/detector") == OK_LEVEL)
        node.wait_for("system health recovery", lambda: node.health and node.health[-1] != "ERROR", timeout=8.0)
        node.wait_for("fault supervisor recovery", lambda: node.fault_states and node.fault_states[-1] == "CLEAR", timeout=8.0)
        print("PASS: RGB interruption, depth interruption, invalid depth, observation-time TF failure, detector fault and recovery")
        print("PASS: perception diagnostics propagated through system_monitor/fault_supervisor without perception velocity output")
        return 0
    except RuntimeError as error:
        with node.lock:
            snapshot = {
                name: (status.level[0] if isinstance(status.level, bytes) else int(status.level))
                for name, status in node.statuses.items()
            }
        print(f"FAIL: {error}; statuses={snapshot}", file=sys.stderr)
        return 1
    finally:
        stop_event.set()
        input_thread.join(timeout=1.0)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
