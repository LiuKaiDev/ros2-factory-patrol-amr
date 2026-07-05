#!/usr/bin/env python3
"""Print Phase 5B factory patrol waypoint goals without sending them."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, List


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_STATIONS = REPO_ROOT / "src/robot_simulation/config/factory_patrol_stations.yaml"
DEFAULT_ROUTE = REPO_ROOT / "src/robot_simulation/config/factory_patrol_route.yaml"


def value_after_colon(line: str, key: str) -> str | None:
    prefix = f"{key}:"
    line = line.strip()
    if not line.startswith(prefix):
        return None
    return line[len(prefix) :].strip().strip("\"'")


def parse_stations(path: Path) -> Dict[str, Dict[str, str]]:
    stations: Dict[str, Dict[str, str]] = {}
    current: Dict[str, str] | None = None
    default_frame = "map"
    in_stations = False
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if value := value_after_colon(line, "frame_id"):
            if current is None:
                default_frame = value or "map"
            else:
                current["frame_id"] = value or default_frame
            continue
        if line == "stations:":
            in_stations = True
            continue
        if line == "edges:":
            if current and "id" in current:
                stations[current["id"]] = current
            current = None
            in_stations = False
            continue
        if in_stations and line.startswith("- "):
            if current and "id" in current:
                stations[current["id"]] = current
            current = {"frame_id": default_frame}
            line = line[2:].strip()
        if current is None:
            continue
        for key in ("id", "x", "y", "yaw", "type"):
            if (value := value_after_colon(line, key)) is not None:
                current[key] = value
                break
    if current and "id" in current:
        stations[current["id"]] = current
    return stations


def parse_route(path: Path) -> List[str]:
    waypoints: List[str] = []
    in_waypoints = False
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line == "waypoints:":
            in_waypoints = True
            continue
        if in_waypoints:
            if line.startswith("- "):
                waypoints.append(line[2:].strip().strip("\"'"))
            elif not line.startswith("#") and ":" in line:
                break
    return waypoints


def quaternion_z_w(yaw: float) -> tuple[float, float]:
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def print_goal_examples(stations_path: Path, route_path: Path) -> int:
    if not stations_path.exists():
        print(f"FAIL: missing stations file: {stations_path}")
        return 1
    if not route_path.exists():
        print(f"FAIL: missing route file: {route_path}")
        return 1

    stations = parse_stations(stations_path)
    waypoints = parse_route(route_path)
    if not waypoints:
        print(f"FAIL: no waypoints found in {route_path}")
        return 1

    print("Factory patrol route goals (dry-run):")
    print(f"  stations: {stations_path}")
    print(f"  route:    {route_path}")
    print("  order:    " + " -> ".join(waypoints))
    print()

    for index, station_id in enumerate(waypoints, start=1):
        station = stations.get(station_id)
        if station is None:
            print(f"FAIL: waypoint {station_id!r} is not defined in stations file")
            return 1
        frame_id = station.get("frame_id", "map")
        x = float(station.get("x", "0.0"))
        y = float(station.get("y", "0.0"))
        yaw = float(station.get("yaw", "0.0"))
        qz, qw = quaternion_z_w(yaw)
        print(f"{index}. {station_id}: frame_id={frame_id} x={x:.3f} y={y:.3f} yaw={yaw:.3f}")
        print("   Nav2 NavigateToPose example:")
        print(
            "   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
            f"'{{pose: {{header: {{frame_id: \"{frame_id}\"}}, pose: {{position: {{x: {x:.3f}, y: {y:.3f}, z: 0.0}}, "
            f"orientation: {{z: {qz:.6f}, w: {qw:.6f}}}}}}}}}'"
        )
        print()

    print("NavigateSequence topic/action note:")
    print("  The project action /navigate_sequence accepts geometry_msgs/PoseStamped[] goals.")
    print("  This script does not send goals. Use the printed poses after Nav2/demo startup.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stations", type=Path, default=DEFAULT_STATIONS)
    parser.add_argument("--route", type=Path, default=DEFAULT_ROUTE)
    parser.add_argument(
        "--send",
        action="store_true",
        help="Reserved for a future explicit sender; currently refuses to send.",
    )
    args = parser.parse_args()
    if args.send:
        print("FAIL: --send is intentionally not implemented in Phase 5B; dry-run only.")
        return 1
    return print_goal_examples(args.stations, args.route)


if __name__ == "__main__":
    raise SystemExit(main())
