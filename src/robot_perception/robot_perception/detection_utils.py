"""Detector-independent filtering and ROS message conversion."""

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


@dataclass(frozen=True)
class BackendDetection:
    class_id: int
    class_name: str
    confidence: float
    x: float
    y: float
    width: float
    height: float


def filter_detections(
    detections: Iterable[BackendDetection],
    image_width: int,
    image_height: int,
    confidence_threshold: float,
    allowed_classes: Sequence[str],
) -> list[BackendDetection]:
    """Validate, filter, and clamp backend boxes to the source image."""
    allowed = set(allowed_classes)
    output = []
    for item in detections:
        values = (item.confidence, item.x, item.y, item.width, item.height)
        if not all(math.isfinite(value) for value in values):
            continue
        if item.confidence < confidence_threshold:
            continue
        if allowed and item.class_name not in allowed:
            continue
        x0 = max(0.0, min(float(image_width), item.x))
        y0 = max(0.0, min(float(image_height), item.y))
        x1 = max(0.0, min(float(image_width), item.x + item.width))
        y1 = max(0.0, min(float(image_height), item.y + item.height))
        if x1 <= x0 or y1 <= y0:
            continue
        output.append(
            BackendDetection(
                class_id=item.class_id,
                class_name=item.class_name,
                confidence=item.confidence,
                x=x0,
                y=y0,
                width=x1 - x0,
                height=y1 - y0,
            )
        )
    return output


def to_detection_array(detections, header):
    """Convert normalized detections while preserving the source image header."""
    message = Detection2DArray()
    message.header = header
    for item in detections:
        detection = Detection2D()
        detection.header = header
        detection.bbox.center.position.x = item.x + item.width / 2.0
        detection.bbox.center.position.y = item.y + item.height / 2.0
        detection.bbox.size_x = item.width
        detection.bbox.size_y = item.height
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = item.class_name
        hypothesis.hypothesis.score = item.confidence
        detection.results.append(hypothesis)
        message.detections.append(detection)
    return message
