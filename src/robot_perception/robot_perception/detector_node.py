"""ROS adapter for replaceable 2D object detector backends."""

from pathlib import Path
import math
import os
import time

import cv2
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray

from robot_perception.detection_utils import filter_detections, to_detection_array
from robot_perception.detector_backend import OpenCvYoloXBackend


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")
        backend_name = self.declare_parameter("backend", "opencv_yolox").value
        model_path = self.declare_parameter("model_path", "").value
        confidence = float(self.declare_parameter("confidence_threshold", 0.45).value)
        input_size = int(self.declare_parameter("input_size", 640).value)
        device = self.declare_parameter("device", "auto").value
        allowed_classes = list(self.declare_parameter("allowed_classes", ["person"]).value)
        nms_threshold = float(self.declare_parameter("nms_threshold", 0.5).value)
        max_inference_rate_hz = float(
            self.declare_parameter("max_inference_rate_hz", 0.0).value
        )
        diagnostics_rate_hz = float(self.declare_parameter("diagnostics_rate_hz", 2.0).value)
        diagnostics_output_timeout_sec = float(
            self.declare_parameter("diagnostics_output_timeout_sec", 3.0).value
        )
        diagnostics_latency_warn_ms = float(
            self.declare_parameter("diagnostics_latency_warn_ms", 1200.0).value
        )
        diagnostics_latency_error_ms = float(
            self.declare_parameter("diagnostics_latency_error_ms", 3000.0).value
        )
        self._debug_enabled = bool(self.declare_parameter("debug_image_enabled", True).value)
        image_topic = self.declare_parameter("image_topic", "/camera/color/image_raw").value
        detection_topic = self.declare_parameter(
            "detection_topic", "/perception/detections_2d"
        ).value
        debug_topic = self.declare_parameter(
            "debug_image_topic", "/perception/debug_image"
        ).value

        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")
        if not math.isfinite(max_inference_rate_hz) or max_inference_rate_hz < 0.0:
            raise ValueError("max_inference_rate_hz must be finite and nonnegative")
        if not math.isfinite(diagnostics_rate_hz) or diagnostics_rate_hz <= 0.0:
            raise ValueError("diagnostics_rate_hz must be finite and positive")
        if diagnostics_output_timeout_sec <= 0.0:
            raise ValueError("diagnostics_output_timeout_sec must be positive")
        if diagnostics_latency_warn_ms <= 0.0 or diagnostics_latency_error_ms <= diagnostics_latency_warn_ms:
            raise ValueError("diagnostic latency thresholds are inconsistent")
        self._confidence = confidence
        self._input_size = input_size
        self._requested_device = device
        self._min_inference_period = (
            0.0 if max_inference_rate_hz == 0.0 else 1.0 / max_inference_rate_hz
        )
        self._last_inference_start = None
        self._last_image_receipt = None
        self._last_output_receipt = None
        self._last_latency_ms = 0.0
        self._consecutive_failures = 0
        self._model_error = ""
        self._model_path = model_path
        self._diagnostics_output_timeout_sec = diagnostics_output_timeout_sec
        self._diagnostics_latency_warn_ms = diagnostics_latency_warn_ms
        self._diagnostics_latency_error_ms = diagnostics_latency_error_ms
        self._allowed_classes = allowed_classes
        self._backend_name = backend_name
        self._bridge = CvBridge()
        self._latency_total_ms = 0.0
        self._inference_count = 0
        self._backend = None
        if not model_path:
            cache_root = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
            model_path = str(
                cache_root / "robot_perception" / "models" /
                "object_detection_yolox_2022nov.onnx"
            )
        self._model_path = model_path
        try:
            if backend_name != "opencv_yolox":
                raise ValueError(f"unsupported detector backend: {backend_name}")
            self._backend = OpenCvYoloXBackend(
                model_path, input_size, device, confidence, nms_threshold
            )
            self.get_logger().info(
                f"detector ready: backend={backend_name}, device={self._backend.device}, "
                f"model={model_path}, classes={allowed_classes}"
            )
        except Exception as error:
            self._model_error = str(error)
            self.get_logger().error(
                f"detector disabled after model initialization failure: {error}"
            )

        self._detection_pub = self.create_publisher(Detection2DArray, detection_topic, 10)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/perception/diagnostics", 10)
        self._debug_pub = self.create_publisher(Image, debug_topic, 2)
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._image_sub = self.create_subscription(
            Image, image_topic, self._on_image, image_qos
        )
        self._diagnostics_timer = self.create_timer(
            1.0 / diagnostics_rate_hz, self._publish_diagnostics
        )

    @staticmethod
    def _value(key, value):
        return KeyValue(key=key, value=str(value))

    def _publish_diagnostics(self):
        now = self.get_clock().now()
        level = DiagnosticStatus.OK
        message = "detector nominal"
        if self._backend is None:
            level = DiagnosticStatus.ERROR
            message = "model/backend unavailable"
        elif self._consecutive_failures >= 3:
            level = DiagnosticStatus.ERROR
            message = "repeated inference failures"
        elif self._last_latency_ms >= self._diagnostics_latency_error_ms:
            level = DiagnosticStatus.ERROR
            message = "inference latency high"
        elif self._last_latency_ms >= self._diagnostics_latency_warn_ms:
            level = DiagnosticStatus.WARN
            message = "inference latency elevated"
        elif (
            self._last_image_receipt is not None
            and self._last_output_receipt is not None
            and (now.nanoseconds - self._last_output_receipt.nanoseconds) / 1e9
            > self._diagnostics_output_timeout_sec
        ):
            level = DiagnosticStatus.WARN
            message = "detector output inactive"
        elif self._last_image_receipt is not None and self._last_output_receipt is None:
            level = DiagnosticStatus.WARN
            message = "detector has not produced output"
        status = DiagnosticStatus(
            name="perception/detector",
            hardware_id="factory_patrol_detector",
            level=level,
            message=message,
            values=[
                self._value("backend", self._backend_name),
                self._value(
                    "device",
                    self._backend.device if self._backend is not None else self._requested_device,
                ),
                self._value("input_size", self._input_size),
                self._value("confidence_threshold", self._confidence),
                self._value("model_path", self._model_path),
                self._value("model_error", self._model_error),
                self._value("inference_count", self._inference_count),
                self._value("consecutive_failures", self._consecutive_failures),
                self._value("last_latency_ms", f"{self._last_latency_ms:.3f}"),
                self._value("last_output_age_sec", "inf" if self._last_output_receipt is None else
                            f"{(now.nanoseconds - self._last_output_receipt.nanoseconds) / 1e9:.3f}"),
            ],
        )
        self._diagnostics_pub.publish(
            DiagnosticArray(header=Header(stamp=now.to_msg()), status=[status])
        )

    def _on_image(self, message):
        now = time.monotonic()
        self._last_image_receipt = self.get_clock().now()
        if (
            self._last_inference_start is not None
            and now - self._last_inference_start < self._min_inference_period
        ):
            return
        self._last_inference_start = now
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            self.get_logger().error(f"RGB conversion failed: {error}")
            self._consecutive_failures += 1
            return

        detections = []
        if self._backend is not None:
            start = time.perf_counter()
            try:
                raw = self._backend.infer(image)
                detections = filter_detections(
                    raw, image.shape[1], image.shape[0], self._confidence,
                    self._allowed_classes,
                )
                self._consecutive_failures = 0
            except Exception as error:
                self.get_logger().error(f"inference failed; publishing no detections: {error}")
                self._consecutive_failures += 1
            latency_ms = (time.perf_counter() - start) * 1000.0
            self._last_latency_ms = latency_ms
            self._latency_total_ms += latency_ms
            self._inference_count += 1
            if self._inference_count == 1 or self._inference_count % 30 == 0:
                average = self._latency_total_ms / self._inference_count
                self.get_logger().info(
                    f"inference latency: current={latency_ms:.1f} ms, average={average:.1f} ms, "
                    f"detections={len(detections)}"
                )

        detection_message = to_detection_array(detections, message.header)
        self._detection_pub.publish(detection_message)
        self._last_output_receipt = self.get_clock().now()
        if self._debug_enabled:
            debug = image.copy()
            for item in detections:
                x0, y0 = int(item.x), int(item.y)
                x1, y1 = int(item.x + item.width), int(item.y + item.height)
                cv2.rectangle(debug, (x0, y0), (x1, y1), (40, 220, 40), 2)
                cv2.putText(
                    debug, f"{item.class_name} {item.confidence:.2f}",
                    (x0, max(18, y0 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (40, 220, 40), 2, cv2.LINE_AA,
                )
            debug_message = self._bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_message.header = message.header
            self._debug_pub.publish(debug_message)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
