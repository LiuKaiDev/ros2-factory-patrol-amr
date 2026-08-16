#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

safe_source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

[[ -f install/setup.bash ]] && safe_source_setup install/setup.bash
command -v ros2 >/dev/null || {
  echo "FAIL: ros2 is unavailable; source ROS 2 and the workspace first" >&2
  exit 1
}

failures=0
check_type() {
  local topic="$1" expected="$2" actual
  actual="$(timeout 5s ros2 topic type "${topic}" 2>/dev/null || true)"
  if [[ "${actual}" == "${expected}" ]]; then
    echo "PASS: ${topic} type=${actual}"
  else
    echo "FAIL: ${topic} expected ${expected}, got ${actual:-none}" >&2
    failures=$((failures + 1))
  fi
}

echo "[detector-runtime-check] Validating the detector chain..."
check_type /perception/detections_2d vision_msgs/msg/Detection2DArray
check_type /perception/debug_image sensor_msgs/msg/Image
check_type /perception/geometry/camera_point geometry_msgs/msg/PointStamped
check_type /perception/geometry/map_point geometry_msgs/msg/PointStamped
check_type /perception/markers visualization_msgs/msg/Marker

if timeout 30s python3 - <<'PY' >/tmp/factory_patrol_detector_detection.txt 2>&1
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


class DetectionProbe(Node):
    def __init__(self):
        super().__init__("factory_patrol_detector_detection_probe")
        self.debug_headers = set()
        self.detection_headers = set()
        self.camera_point_headers = set()
        self.map_point_headers = set()
        self.create_subscription(Image, "/perception/debug_image", self.on_debug, 2)
        self.create_subscription(
            Detection2DArray, "/perception/detections_2d", self.on_detection, 10
        )
        self.create_subscription(
            PointStamped, "/perception/geometry/camera_point",
            self.on_camera_point, 10,
        )
        self.create_subscription(
            PointStamped, "/perception/geometry/map_point", self.on_map_point, 10
        )

    @staticmethod
    def header_key(header):
        return (header.stamp.sec, header.stamp.nanosec)

    def on_debug(self, message):
        if (
            message.header.frame_id == "camera_color_optical_frame"
            and message.width > 0
            and message.height > 0
            and len(message.data) > 0
        ):
            self.debug_headers.add(self.header_key(message.header))
        self.maybe_finish()

    @staticmethod
    def valid_point(message, frame_id):
        values = (message.point.x, message.point.y, message.point.z)
        return message.header.frame_id == frame_id and all(
            value == value and abs(value) < 1000000.0 for value in values
        )

    def on_camera_point(self, message):
        if self.valid_point(message, "camera_color_optical_frame"):
            self.camera_point_headers.add(self.header_key(message.header))
        self.maybe_finish()

    def on_map_point(self, message):
        if self.valid_point(message, "map"):
            self.map_point_headers.add(self.header_key(message.header))
        self.maybe_finish()

    def on_detection(self, message):
        key = self.header_key(message.header)
        valid = (
            message.header.frame_id == "camera_color_optical_frame"
            and (message.header.stamp.sec > 0 or message.header.stamp.nanosec > 0)
            and len(message.detections) > 0
        )
        for detection in message.detections:
            valid = valid and (
                detection.header == message.header
                and detection.bbox.size_x > 0.0
                and detection.bbox.size_y > 0.0
                and len(detection.results) > 0
                and detection.results[0].hypothesis.class_id == "person"
                and 0.0 < detection.results[0].hypothesis.score <= 1.0
            )
        if valid:
            first = message.detections[0]
            self.detection_headers.add(key)
            self.description = (
                f"stamp={message.header.stamp.sec}.{message.header.stamp.nanosec:09d} "
                f"class={first.results[0].hypothesis.class_id} "
                f"confidence={first.results[0].hypothesis.score:.3f} "
                f"bbox=({first.bbox.center.position.x:.1f}, "
                f"{first.bbox.center.position.y:.1f}, "
                f"{first.bbox.size_x:.1f}, {first.bbox.size_y:.1f})"
            )
        self.maybe_finish()

    def maybe_finish(self):
        common = (
            self.debug_headers
            & self.detection_headers
            & self.camera_point_headers
            & self.map_point_headers
        )
        if common:
            print(
                f"{self.description} timestamp_chain=debug/detection/camera/map"
            )
            rclpy.shutdown()


rclpy.init()
probe = DetectionProbe()
rclpy.spin(probe)
PY
then
  echo "PASS: real person detection has a source-image stamp, class, score, and valid bbox"
  sed 's/^/      /' /tmp/factory_patrol_detector_detection.txt
else
  echo "FAIL: no valid person detection observed" >&2
  failures=$((failures + 1))
fi

if timeout 12s ros2 topic echo --once --timeout 10 /perception/debug_image \
    --filter "m.header.frame_id == 'camera_color_optical_frame' and m.width > 0 and m.height > 0 and len(m.data) > 0" \
    --no-arr >/dev/null 2>&1; then
  echo "PASS: detector debug image is nonempty and in the optical frame"
else
  echo "FAIL: detector debug image is missing or invalid" >&2
  failures=$((failures + 1))
fi

for specification in \
    "/perception/geometry/camera_point camera_color_optical_frame" \
    "/perception/geometry/map_point map"; do
  read -r topic frame <<<"${specification}"
  if timeout 25s ros2 topic echo --once --timeout 23 "${topic}" \
      --filter "m.header.frame_id == '${frame}' and (m.header.stamp.sec > 0 or m.header.stamp.nanosec > 0) and m.point.x == m.point.x and m.point.y == m.point.y and m.point.z == m.point.z" \
      --no-arr >/dev/null 2>&1; then
    echo "PASS: ${topic} publishes a finite stamped point in ${frame}"
  else
    echo "FAIL: ${topic} did not publish valid detector geometry" >&2
    failures=$((failures + 1))
  fi
done

if timeout 25s ros2 topic echo --once --timeout 23 /perception/markers \
    --filter "m.header.frame_id == 'map' and m.action == 0 and m.type == 9 and 'person' in m.text and 'depth=' in m.text" \
    --no-arr >/dev/null 2>&1; then
  echo "PASS: map marker exposes person confidence/depth/position text"
else
  echo "FAIL: expected detector text marker was not observed" >&2
  failures=$((failures + 1))
fi

for topic in /scan /imu/data /odom /cmd_vel /safety_state; do
  if [[ -n "$(timeout 5s ros2 topic type "${topic}" 2>/dev/null || true)" ]]; then
    echo "PASS: existing runtime topic preserved: ${topic}"
  else
    echo "FAIL: existing runtime topic missing: ${topic}" >&2
    failures=$((failures + 1))
  fi
done

if [[ "${failures}" -eq 0 ]]; then
  echo "[detector-runtime-check] PASS: detector -> 2D -> depth -> camera/map -> marker"
  exit 0
fi
echo "[detector-runtime-check] FAIL: ${failures} check(s) failed" >&2
exit 1
