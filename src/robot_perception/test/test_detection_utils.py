from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from robot_perception.detection_utils import (
    BackendDetection,
    filter_detections,
    to_detection_array,
)


def detection(name="person", score=0.8, x=10.0, y=20.0, width=30.0, height=40.0):
    return BackendDetection(0, name, score, x, y, width, height)


def test_filters_confidence_and_allowed_class():
    result = filter_detections(
        [detection(score=0.2), detection(name="car"), detection()],
        640, 480, 0.5, ["person"],
    )
    assert result == [detection()]


def test_clamps_bbox_to_image_and_rejects_empty_box():
    result = filter_detections(
        [detection(x=-5.0, y=470.0, width=20.0, height=30.0),
         detection(x=700.0, width=10.0)],
        640, 480, 0.5, [],
    )
    assert result == [detection(x=0.0, y=470.0, width=15.0, height=10.0)]


def test_rejects_nonfinite_or_invalid_bbox():
    result = filter_detections(
        [detection(score=float("nan")), detection(width=-2.0),
         detection(x=float("inf"))],
        640, 480, 0.5, ["person"],
    )
    assert result == []


def test_converts_multiple_detections_and_preserves_header():
    header = Header(stamp=Time(sec=12, nanosec=34), frame_id="camera_color_optical_frame")
    result = to_detection_array([detection(), detection(name="car", x=50.0)], header)
    assert len(result.detections) == 2
    assert result.header == header
    assert all(item.header == header for item in result.detections)
    assert result.detections[0].results[0].hypothesis.class_id == "person"
    assert result.detections[0].results[0].hypothesis.score == 0.8
    assert result.detections[0].bbox.center.position.x == 25.0
    assert result.detections[0].bbox.size_y == 40.0
