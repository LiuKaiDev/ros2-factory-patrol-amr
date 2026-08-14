#!/usr/bin/env python3

import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces_perception.msg import DetectedObject3D, PerceptionSafetyEvent
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String


class Phase6Probe(Node):
    def __init__(self):
        super().__init__("phase6_perception_safety_runtime_probe")
        self._lock = threading.Lock()
        self.events = []
        self.targets = []
        self.safety_states = []
        self.safety_reasons = []
        self.upstream_commands = []
        self.final_commands = []
        self.create_subscription(
            PerceptionSafetyEvent,
            "/perception/safety_event",
            self._event_callback,
            10,
        )
        self.create_subscription(
            DetectedObject3D,
            "/perception/objects_3d",
            self._target_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(String, "/safety/state", self._state_callback, 10)
        self.create_subscription(String, "/safety/reason", self._reason_callback, 10)
        self.create_subscription(Twist, "/nav2_cmd_vel", self._upstream_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self._final_callback, 10)
        self.set_pose_client = self.create_client(
            SetEntityPose, "/world/factory_patrol/set_pose"
        )
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _event_callback(self, message):
        with self._lock:
            self.events.append((self.get_clock().now().nanoseconds, message))

    def _target_callback(self, message):
        if message.class_name == "person":
            with self._lock:
                self.targets.append((self.get_clock().now().nanoseconds, message))

    def _state_callback(self, message):
        with self._lock:
            self.safety_states.append((self.get_clock().now().nanoseconds, message.data))

    def _reason_callback(self, message):
        with self._lock:
            self.safety_reasons.append((self.get_clock().now().nanoseconds, message.data))

    def _upstream_callback(self, message):
        with self._lock:
            self.upstream_commands.append(
                (self.get_clock().now().nanoseconds, message.linear.x, message.angular.z)
            )

    def _final_callback(self, message):
        with self._lock:
            self.final_commands.append(
                (self.get_clock().now().nanoseconds, message.linear.x, message.angular.z)
            )

    def wait_until(self, description, predicate, timeout_sec=30.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                result = predicate()
            if result is not None and result is not False:
                return result
            time.sleep(0.05)
        raise RuntimeError(f"timeout waiting for {description}")

    def move_fixture(self, name, x, y, z=0.0):
        if not self.set_pose_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/world/factory_patrol/set_pose is unavailable")
        request = SetEntityPose.Request()
        request.entity.name = name
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(x)
        request.pose.position.y = float(y)
        request.pose.position.z = float(z)
        request.pose.orientation.z = math.sin(math.pi / 4.0)
        request.pose.orientation.w = math.cos(math.pi / 4.0)
        future = self.set_pose_client.call_async(request)
        response = self.wait_until(
            f"Gazebo {name} pose response at ({x:.3f}, {y:.3f}, {z:.3f})",
            lambda: future.result() if future.done() else None,
            30.0,
        )
        if not response.success:
            raise RuntimeError(f"Gazebo rejected fixture pose for {name}")

    def move_standard_person(self, x, y, z=0.0):
        self.move_fixture("phase3_person_detection_target", x, y, z)
        return x, y

    def move_close_person(self, x, y, z=0.0):
        self.move_fixture("phase6_person_safety_target", x, y, z)
        return x, y

    def send_navigation_goal(self):
        if not self.nav_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("Nav2 /navigate_to_pose action is unavailable")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = 4.2
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.orientation.w = 1.0
        future = self.nav_client.send_goal_async(goal)
        goal_handle = self.wait_until(
            "Nav2 goal acceptance", lambda: future.result() if future.done() else None, 20.0
        )
        if not goal_handle.accepted:
            raise RuntimeError("Nav2 rejected the Phase 6 validation goal")
        return goal_handle

    def wait_for_event(self, start_index, predicate, description, timeout_sec=35.0):
        def find_event():
            for receipt_ns, event in self.events[start_index:]:
                if predicate(event):
                    return receipt_ns, event
            return None

        return self.wait_until(description, find_event, timeout_sec)

    def wait_for_state(self, start_index, expected, timeout_sec=10.0):
        def find_state():
            for stamp_ns, value in self.safety_states[start_index:]:
                if value == expected:
                    return stamp_ns, value
            return None

        return self.wait_until(f"safety state {expected}", find_state, timeout_sec)

    def wait_for_motion_evidence(self, after_ns, limited=False, stopped=False):
        def find_samples():
            def upstream_requests_motion(sample):
                if limited:
                    return abs(sample[1]) > 0.1501 or abs(sample[2]) > 0.4001
                return abs(sample[1]) > 0.02 or abs(sample[2]) > 0.02

            upstream = next(
                (
                    sample
                    for sample in self.upstream_commands
                    if sample[0] >= after_ns and upstream_requests_motion(sample)
                ),
                None,
            )
            if upstream is None:
                return None
            finals = [
                sample for sample in self.final_commands if sample[0] >= upstream[0]
            ]
            if stopped:
                final = next(
                    (
                        sample
                        for sample in finals
                        if abs(sample[1]) <= 1.0e-6 and abs(sample[2]) <= 1.0e-6
                    ),
                    None,
                )
            elif limited:
                final = next(
                    (
                        sample
                        for sample in finals
                        if abs(sample[1]) <= 0.1501
                        and abs(sample[2]) <= 0.4001
                        and (abs(sample[1]) > 0.01 or abs(sample[2]) > 0.01)
                    ),
                    None,
                )
            else:
                final = next(
                    (
                        sample
                        for sample in finals
                        if abs(sample[1]) > 0.02 or abs(sample[2]) > 0.02
                    ),
                    None,
                )
            if final is None:
                return None
            return upstream, final

        return self.wait_until("upstream and final velocity evidence", find_samples, 15.0)


def message_stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / 1.0e9


def main():
    rclpy.init(args=sys.argv)
    node = Phase6Probe()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    goal_handle = None
    try:
        node.wait_until(
            "perception safety publisher",
            lambda: True if node.count_publishers("/perception/safety_event") > 0 else None,
            20.0,
        )
        node.move_close_person(0.0, 0.0, -2.0)
        node.move_standard_person(2.80, -0.75)
        node.wait_until(
            "first valid perception safety event after map TF startup",
            lambda: node.events[-1] if node.events else None,
            90.0,
        )
        clear_event_index = len(node.events)
        clear_state_index = len(node.safety_states)
        far_pose = node.move_standard_person(3.20, 0.75)
        clear_receipt_ns, clear_event = node.wait_for_event(
            clear_event_index,
            lambda event: event.event_type == "CLEAR"
            and event.target_id > 0
            and event.distance_m > 3.0,
            "measured CLEAR person",
        )
        goal_handle = node.send_navigation_goal()
        node.wait_until(
            "nonzero Nav2 command",
            lambda: next(
                (
                    sample
                    for sample in node.upstream_commands
                    if sample[0] >= clear_receipt_ns
                    and (abs(sample[1]) > 0.02 or abs(sample[2]) > 0.02)
                ),
                None,
            ),
            30.0,
        )
        node.wait_for_state(clear_state_index, "NORMAL")

        zone_event_index = len(node.events)
        zone_state_index = len(node.safety_states)
        node.move_standard_person(3.10, -0.75)
        zone_receipt_ns, zone_event = node.wait_for_event(
            zone_event_index,
            lambda event: event.event_type == "PERSON_IN_DANGER_ZONE"
            and event.zone_id == "factory_person_danger_zone"
            and event.distance_m >= 1.5,
            "danger-zone-only STOP",
            60.0,
        )
        node.wait_for_state(zone_state_index, "STOP")
        _, zone_final = node.wait_for_motion_evidence(
            zone_receipt_ns, stopped=True
        )

        speed_event_index = len(node.events)
        speed_state_index = len(node.safety_states)
        speed_pose = node.move_standard_person(2.80, 0.75)
        speed_receipt_ns, speed_event = node.wait_for_event(
            speed_event_index,
            lambda event: event.event_type == "PERSON_NEAR"
            and 1.5 <= event.distance_m <= 3.0,
            "measured SPEED_LIMITED person",
        )
        node.wait_for_state(speed_state_index, "SPEED_LIMITED")
        speed_upstream, speed_final = node.wait_for_motion_evidence(
            speed_receipt_ns, limited=True
        )

        stop_event_index = len(node.events)
        stop_state_index = len(node.safety_states)
        node.move_standard_person(0.0, 0.0, -2.0)
        stop_pose = node.move_close_person(1.30, 0.0)
        stop_receipt_ns, stop_event = node.wait_for_event(
            stop_event_index,
            lambda event: event.event_type == "PERSON_TOO_CLOSE"
            and event.distance_m < 1.5,
            "measured STOP person",
            60.0,
        )
        node.wait_for_state(stop_state_index, "STOP")
        stop_upstream, stop_final = node.wait_for_motion_evidence(
            stop_receipt_ns, stopped=True
        )
        stop_condition_ns = (
            stop_event.header.stamp.sec * 1_000_000_000
            + stop_event.header.stamp.nanosec
        )
        response_latency_sec = (stop_final[0] - stop_condition_ns) / 1.0e9
        if response_latency_sec < 0.0:
            raise RuntimeError("negative safety response latency")

        recovery_event_index = len(node.events)
        recovery_state_index = len(node.safety_states)
        node.move_close_person(0.0, 0.0, -2.0)
        recovery_pose = node.move_standard_person(3.20, 0.75)
        recovery_receipt_ns, recovery_event = node.wait_for_event(
            recovery_event_index,
            lambda event: event.event_type == "CLEAR",
            "perception safety recovery",
            60.0,
        )
        node.wait_for_state(recovery_state_index, "NORMAL", 15.0)
        recovery_upstream, recovery_final = node.wait_for_motion_evidence(
            recovery_receipt_ns
        )

        perception_publishers = {
            name: node.count_publishers(name)
            for name in ("/cmd_vel", "/nav2_cmd_vel")
        }
        print("[phase6-check] PASS: deterministic perception safety runtime validation")
        print(
            f"[phase6-check] clear_fixture_pose=({far_pose[0]:.3f}, {far_pose[1]:.3f})"
        )
        print(f"[phase6-check] clear_person_distance={clear_event.distance_m:.3f} m")
        print(f"[phase6-check] clear_perception_state={clear_event.event_type}")
        print("[phase6-check] clear_final_safety_state=NORMAL")
        print(
            f"[phase6-check] speed_fixture_pose=({speed_pose[0]:.3f}, {speed_pose[1]:.3f})"
        )
        print(f"[phase6-check] speed_person_distance={speed_event.distance_m:.3f} m")
        print(
            f"[phase6-check] speed_upstream_velocity=(linear={speed_upstream[1]:.3f}, angular={speed_upstream[2]:.3f})"
        )
        print(
            f"[phase6-check] speed_final_velocity=(linear={speed_final[1]:.3f}, angular={speed_final[2]:.3f})"
        )
        print("[phase6-check] speed_effective_safety_state=SPEED_LIMITED")
        print(
            f"[phase6-check] stop_fixture_pose=({stop_pose[0]:.3f}, {stop_pose[1]:.3f}), scale=0.40"
        )
        print(f"[phase6-check] stop_person_distance={stop_event.distance_m:.3f} m")
        print(
            f"[phase6-check] stop_upstream_velocity=(linear={stop_upstream[1]:.3f}, angular={stop_upstream[2]:.3f})"
        )
        print(
            f"[phase6-check] stop_final_velocity=(linear={stop_final[1]:.3f}, angular={stop_final[2]:.3f})"
        )
        print("[phase6-check] stop_effective_safety_state=STOP")
        print(
            "[phase6-check] danger_zone=factory_person_danger_zone "
            "polygon=[(3.00,-1.20),(3.80,-1.20),(3.80,-0.30),(3.00,-0.30)]"
        )
        print(
            f"[phase6-check] danger_person_map_pose=({zone_event.target_position.x:.3f}, {zone_event.target_position.y:.3f})"
        )
        print(f"[phase6-check] danger_person_distance={zone_event.distance_m:.3f} m")
        print("[phase6-check] danger_inside_zone=true")
        print("[phase6-check] danger_effective_safety_state=STOP")
        print(
            f"[phase6-check] danger_final_velocity=(linear={zone_final[1]:.3f}, angular={zone_final[2]:.3f})"
        )
        print(
            f"[phase6-check] recovery_fixture_pose=({recovery_pose[0]:.3f}, {recovery_pose[1]:.3f})"
        )
        print("[phase6-check] recovery_clear_policy=3 valid observations")
        print(f"[phase6-check] recovery_event={recovery_event.event_type}")
        print("[phase6-check] recovery_final_safety_state=NORMAL")
        print(
            f"[phase6-check] recovery_upstream_velocity=(linear={recovery_upstream[1]:.3f}, angular={recovery_upstream[2]:.3f})"
        )
        print(
            f"[phase6-check] recovery_final_velocity=(linear={recovery_final[1]:.3f}, angular={recovery_final[2]:.3f})"
        )
        print("[phase6-check] navigation_recovery=existing Nav2 goal remained active")
        print(
            f"[phase6-check] stop_condition_timestamp={message_stamp_seconds(stop_event):.9f} sim_s"
        )
        print(f"[phase6-check] stop_event_receipt_timestamp={stop_receipt_ns / 1.0e9:.9f} sim_s")
        print(f"[phase6-check] final_safe_cmd_timestamp={stop_final[0] / 1.0e9:.9f} sim_s")
        print(f"[phase6-check] stop_response_latency={response_latency_sec:.6f} s")
        print(
            "[phase6-check] velocity_topic_publishers="
            f"cmd_vel:{perception_publishers['/cmd_vel']} "
            f"nav2_cmd_vel:{perception_publishers['/nav2_cmd_vel']}"
        )
    except Exception as error:
        print(f"[phase6-check] FAIL: {error}", file=sys.stderr)
        return_code = 1
    else:
        return_code = 0
    finally:
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
            time.sleep(0.2)
        executor.shutdown(timeout_sec=5.0)
        spin_thread.join(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
