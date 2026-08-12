"""ROS adapter for replaceable 2D object detector backends."""

from pathlib import Path
import os
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
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
        self._confidence = confidence
        self._allowed_classes = allowed_classes
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
            self.get_logger().error(
                f"detector disabled after model initialization failure: {error}"
            )

        self._detection_pub = self.create_publisher(Detection2DArray, detection_topic, 10)
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

    def _on_image(self, message):
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            self.get_logger().error(f"RGB conversion failed: {error}")
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
            except Exception as error:
                self.get_logger().error(f"inference failed; publishing no detections: {error}")
            latency_ms = (time.perf_counter() - start) * 1000.0
            self._latency_total_ms += latency_ms
            self._inference_count += 1
            if self._inference_count == 1 or self._inference_count % 30 == 0:
                average = self._latency_total_ms / self._inference_count
                self.get_logger().info(
                    f"inference latency: current={latency_ms:.1f} ms, average={average:.1f} ms, "
                    f"detections={len(detections)}"
                )

        self._detection_pub.publish(to_detection_array(detections, message.header))
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
