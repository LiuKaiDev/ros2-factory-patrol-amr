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

for specification in \
    "/perception/detections_2d vision_msgs/msg/Detection2DArray" \
    "/camera/depth/image_raw sensor_msgs/msg/Image" \
    "/camera/color/camera_info sensor_msgs/msg/CameraInfo"; do
  read -r topic expected <<<"${specification}"
  actual="$(timeout 5s ros2 topic type "${topic}" 2>/dev/null || true)"
  if [[ "${actual}" != "${expected}" ]]; then
    echo "FAIL: ${topic} expected ${expected}, got ${actual:-none}" >&2
    exit 1
  fi
done

TEMP_DIR="$(mktemp -d /tmp/factory_patrol_phase4.XXXXXX)"
NODE_LOG="${TEMP_DIR}/target_manager_node.log"
PROBE_LOG="${TEMP_DIR}/target_manager_probe.log"
PROBE_PID=""

cleanup() {
  if [[ -n "${PROBE_PID}" ]]; then
    kill "${PROBE_PID}" 2>/dev/null || true
    wait "${PROBE_PID}" 2>/dev/null || true
  fi
  rm -r "${TEMP_DIR}"
}
trap cleanup EXIT

GEOMETRY_NODE="$(ros2 pkg prefix robot_perception)/lib/robot_perception/geometry_validation_node"
"${GEOMETRY_NODE}" --ros-args \
  -r __node:=phase4_target_manager_probe \
  -p use_sim_time:=true \
  -p geometry_input_mode:=detector \
  -p detection_topic:=/phase4_validation/detections_2d \
  -p depth_topic:=/phase4_validation/depth \
  -p camera_info_topic:=/phase4_validation/camera_info \
  -p camera_point_topic:=/phase4_validation/camera_point \
  -p map_point_topic:=/phase4_validation/raw_map_point \
  -p marker_topic:=/phase4_validation/markers \
  -p objects_3d_topic:=/phase4_validation/objects_3d \
  -p ground_truth.enabled:=false \
  -p sync_queue_size:=120 \
  -p tracking.confirm_frames:=3 \
  -p tracking.lost_frames:=5 \
  -p tracking.max_match_distance:=0.5 \
  -p tracking.ema_alpha:=0.4 \
  -p tracking.processed_cooldown_sec:=10.0 >"${NODE_LOG}" 2>&1 &
PROBE_PID=$!

for _ in $(seq 1 30); do
  if timeout 3s ros2 node list 2>/dev/null |
      grep -Fxq /phase4_target_manager_probe; then
    break
  fi
  sleep 0.25
done
if ! kill -0 "${PROBE_PID}" 2>/dev/null; then
  echo "FAIL: isolated TargetManager probe node did not remain alive" >&2
  sed 's/^/      /' "${NODE_LOG}" >&2
  exit 1
fi

if ! timeout 120s python3 - <<'PY' >"${PROBE_LOG}" 2>&1
import copy
from collections import deque
import math
import statistics

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces_perception.msg import DetectedObject3D
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection2DArray


class TargetManagerProbe(Node):
    SAMPLE_COUNT = 12

    def __init__(self):
        super().__init__("factory_patrol_phase4_runtime_probe")
        self.relay = self.create_publisher(
            Detection2DArray, "/phase4_validation/detections_2d", 10
        )
        self.depth_relay = self.create_publisher(
            Image, "/phase4_validation/depth", 10
        )
        self.info_relay = self.create_publisher(
            CameraInfo, "/phase4_validation/camera_info", 10
        )
        self.create_subscription(
            Detection2DArray, "/perception/detections_2d", self.on_source, 10
        )
        self.create_subscription(
            Image, "/camera/depth/image_raw", self.on_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.on_info,
            qos_profile_sensor_data,
        )
        self.raw_sub = self.create_subscription(
            PointStamped, "/phase4_validation/raw_map_point", self.on_raw, 20
        )
        self.target_sub = self.create_subscription(
            DetectedObject3D, "/phase4_validation/objects_3d", self.on_target, 20
        )
        self.marker_sub = self.create_subscription(
            Marker, "/phase4_validation/markers", self.on_marker, 20
        )
        self.stage = "confirm"
        self.target_id = None
        self.saw_tentative = False
        self.saw_confirmed = False
        self.short_dropout_preserved = False
        self.saw_lost = False
        self.reacquired_after_lost = False
        self.duplicate_cycle_suppressed = False
        self.marker_seen = False
        self.duplicate_timer = None
        self.processed_empty_outputs = 0
        self.raw = {}
        self.filtered = {}
        self.seen_target_ids = set()
        self.depth_buffer = deque(maxlen=180)
        self.info_buffer = deque(maxlen=180)
        self.dropout_driver = self.create_timer(0.25, self.drive_dropout)
        self.last_dropout_stamp_ns = -1

    def graph_ready(self):
        return (
            self.relay.get_subscription_count() > 0
            and self.depth_relay.get_subscription_count() > 0
            and self.info_relay.get_subscription_count() > 0
            and self.raw_sub.get_publisher_count() > 0
            and self.target_sub.get_publisher_count() > 0
            and self.marker_sub.get_publisher_count() > 0
        )

    @staticmethod
    def stamp_ns(header):
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    def on_depth(self, message):
        self.depth_buffer.append(message)

    def on_info(self, message):
        self.info_buffer.append(message)

    def relay_sensor_pair(self, header):
        if not self.depth_buffer or not self.info_buffer:
            return False
        stamp = self.stamp_ns(header)
        depth = min(
            self.depth_buffer, key=lambda message: abs(self.stamp_ns(message.header) - stamp)
        )
        info = min(
            self.info_buffer, key=lambda message: abs(self.stamp_ns(message.header) - stamp)
        )
        if (
            abs(self.stamp_ns(depth.header) - stamp) > 50_000_000
            or abs(self.stamp_ns(info.header) - stamp) > 50_000_000
        ):
            return False
        relayed_depth = copy.deepcopy(depth)
        relayed_info = copy.deepcopy(info)
        relayed_depth.header.stamp = header.stamp
        relayed_info.header.stamp = header.stamp
        self.depth_relay.publish(relayed_depth)
        self.info_relay.publish(relayed_info)
        return True

    def drive_dropout(self):
        if self.stage not in ("short_dropout", "long_dropout"):
            return
        if not self.graph_ready() or not self.depth_buffer or not self.info_buffer:
            return
        depth = self.depth_buffer[-1]
        stamp_ns = self.stamp_ns(depth.header)
        if stamp_ns <= self.last_dropout_stamp_ns:
            return
        info = min(
            self.info_buffer,
            key=lambda message: abs(self.stamp_ns(message.header) - stamp_ns),
        )
        relayed_depth = copy.deepcopy(depth)
        relayed_info = copy.deepcopy(info)
        relayed_info.header = copy.deepcopy(relayed_depth.header)
        empty = Detection2DArray()
        empty.header = copy.deepcopy(relayed_depth.header)
        self.depth_relay.publish(relayed_depth)
        self.info_relay.publish(relayed_info)
        self.relay.publish(empty)
        self.last_dropout_stamp_ns = stamp_ns

    @staticmethod
    def key(header):
        return (header.stamp.sec, header.stamp.nanosec)

    @staticmethod
    def first_person(message):
        for detection in message.detections:
            if detection.results and detection.results[0].hypothesis.class_id == "person":
                return detection
        return None

    def publish_real(self, message, duplicate=False):
        person = self.first_person(message)
        if person is None:
            return False
        relayed = copy.deepcopy(message)
        relayed.detections = [copy.deepcopy(person)]
        if duplicate:
            relayed.detections.append(copy.deepcopy(person))
        self.relay.publish(relayed)
        return True

    def paired_samples(self):
        keys = sorted(set(self.raw) & set(self.filtered))
        return [(self.raw[key], self.filtered[key]) for key in keys]

    def on_source(self, message):
        if self.stage not in (
            "confirm", "sample", "short_reacquire",
            "lost_reacquire", "duplicate",
        ):
            return
        if not self.graph_ready() or not self.relay_sensor_pair(message.header):
            return
        if self.stage in ("confirm", "sample"):
            self.publish_real(message)
        elif self.stage == "short_reacquire":
            self.publish_real(message)
        elif self.stage == "lost_reacquire":
            self.publish_real(message)
        elif self.stage == "duplicate":
            self.publish_real(message, duplicate=True)

    def on_raw(self, message):
        values = (message.point.x, message.point.y, message.point.z)
        if message.header.frame_id == "map" and all(math.isfinite(v) for v in values):
            self.raw[self.key(message.header)] = values

    def on_target(self, message):
        values = (message.position.x, message.position.y, message.position.z)
        if (
            message.header.frame_id != "map"
            or not message.depth_valid
            or not all(math.isfinite(v) for v in values)
            or message.class_name != "person"
        ):
            return
        self.seen_target_ids.add(message.target_id)
        if self.target_id is None:
            self.target_id = message.target_id
        if message.target_id != self.target_id:
            self.finish("multiple target IDs were produced for the one real person")
            return
        if message.tracking_state == DetectedObject3D.TENTATIVE:
            self.saw_tentative = True
        if message.tracking_state == DetectedObject3D.CONFIRMED:
            self.saw_confirmed = True
        if message.tracking_state in (
            DetectedObject3D.TENTATIVE, DetectedObject3D.CONFIRMED
        ):
            self.filtered[self.key(message.header)] = values

        if self.stage == "confirm" and self.saw_tentative and self.saw_confirmed:
            self.stage = "sample"
        elif self.stage == "sample" and len(self.paired_samples()) >= self.SAMPLE_COUNT:
            self.stage = "short_dropout"
        elif self.stage == "short_dropout":
            self.processed_empty_outputs += 1
            if message.tracking_state != DetectedObject3D.LOST:
                self.short_dropout_preserved = True
                self.stage = "short_reacquire"
        elif self.stage == "short_reacquire":
            if message.tracking_state == DetectedObject3D.CONFIRMED:
                self.processed_empty_outputs = 0
                self.stage = "long_dropout"
        elif self.stage == "long_dropout":
            self.processed_empty_outputs += 1
            if message.tracking_state == DetectedObject3D.LOST:
                self.saw_lost = self.processed_empty_outputs >= 5
                self.stage = "lost_reacquire"
        elif self.stage == "lost_reacquire":
            if message.tracking_state == DetectedObject3D.CONFIRMED:
                self.reacquired_after_lost = True
                self.stage = "duplicate"
        elif self.stage == "duplicate":
            if message.tracking_state == DetectedObject3D.CONFIRMED:
                self.stage = "duplicate_settle"
                self.duplicate_timer = self.create_timer(
                    1.0, self.finish_after_duplicate
                )

    def finish_after_duplicate(self):
        self.duplicate_timer.cancel()
        self.duplicate_cycle_suppressed = len(self.seen_target_ids) == 1
        self.finish()

    def on_marker(self, message):
        if (
            message.type == Marker.TEXT_VIEW_FACING
            and self.target_id is not None
            and f"#{self.target_id} person" in message.text
            and ("TENTATIVE" in message.text or "CONFIRMED" in message.text)
        ):
            self.marker_seen = True

    def finish(self, error=None):
        if error is not None:
            print(f"FAIL: {error}")
            rclpy.shutdown()
            return
        checks = {
            "initial TENTATIVE state": self.saw_tentative,
            "transition to CONFIRMED": self.saw_confirmed,
            "one stable target_id": len(self.seen_target_ids) == 1,
            "one-frame dropout preserved target": self.short_dropout_preserved,
            "lost_frames transitioned target to LOST": self.saw_lost,
            "LOST target reacquired with same ID": self.reacquired_after_lost,
            "same-cycle duplicate suppressed": self.duplicate_cycle_suppressed,
            "managed lifecycle marker observed": self.marker_seen,
        }
        for label, passed in checks.items():
            print(f"{'PASS' if passed else 'FAIL'}: {label}")
        samples = self.paired_samples()
        print(f"stable_target_id={self.target_id} paired_position_samples={len(samples)}")
        if len(samples) >= 2:
            raw = list(zip(*(sample[0] for sample in samples)))
            filtered = list(zip(*(sample[1] for sample in samples)))
            print(
                "raw_std_m=" + ",".join(
                    f"{axis}={statistics.pstdev(values):.6f}"
                    for axis, values in zip("xyz", raw)
                )
            )
            print(
                "filtered_std_m=" + ",".join(
                    f"{axis}={statistics.pstdev(values):.6f}"
                    for axis, values in zip("xyz", filtered)
                )
            )
        self.success = all(checks.values()) and len(samples) >= self.SAMPLE_COUNT
        rclpy.shutdown()


rclpy.init()
probe = TargetManagerProbe()
probe.success = False
try:
    rclpy.spin(probe)
finally:
    success = probe.success
    probe.destroy_node()
if not success:
    raise SystemExit(1)
PY
then
  echo "FAIL: Phase 4 managed-target runtime validation did not complete" >&2
  sed 's/^/      /' "${PROBE_LOG}" >&2
  sed 's/^/      node: /' "${NODE_LOG}" >&2
  exit 1
fi

echo "[target-manager-runtime-check] Real detector + RGB-D TargetManager results:"
sed 's/^/      /' "${PROBE_LOG}"
if latency_line="$(grep 'TargetManager latency:' "${NODE_LOG}" | tail -n 1)" &&
    [[ -n "${latency_line}" ]]; then
  echo "PASS: measured TargetManager update latency"
  echo "      ${latency_line}"
else
  echo "FAIL: TargetManager latency measurement was not logged" >&2
  exit 1
fi

if ! kill -0 "${PROBE_PID}" 2>/dev/null; then
  echo "FAIL: isolated TargetManager node exited during validation" >&2
  exit 1
fi

echo "[target-manager-runtime-check] PASS: stable managed target lifecycle validated"
