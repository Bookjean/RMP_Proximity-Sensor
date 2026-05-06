#!/usr/bin/env python3

import argparse
import json
import math
import sys
import time
from typing import Dict, List

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray


SCENARIOS: Dict[str, Dict] = {
    "goal_only": {
        "goal": [0.50, -0.20, 0.58],
        "obstacles": [],
    },
    "front_obstacle": {
        "goal": [0.52, -0.18, 0.58],
        "obstacles": [
            {"center": [0.36, -0.18, 0.56], "radius": 0.12},
        ],
    },
    "offset_obstacle": {
        "goal": [0.48, -0.28, 0.55],
        "obstacles": [
            {"center": [0.34, -0.21, 0.50], "radius": 0.10},
        ],
    },
}


class RmpDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("rmp_debug")
        self.goal_pub = self.create_publisher(PoseStamped, "goal_pose", 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, "obstacles", 10)
        self.debug_sub = self.create_subscription(
            Float64MultiArray, "rmp_debug_state", self.on_debug_state, 10
        )
        self.eef_sub = self.create_subscription(
            PoseStamped, "end_effector_pose", self.on_eef_pose, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, "joint_states", self.on_joint_states, 10
        )

        self.goal = [0.50, -0.20, 0.58]
        self.obstacles: List[Dict] = []
        self.last_debug: Dict[str, float] = {}
        self.last_eef = None
        self.last_joint_speed_norm = 0.0
        self.command_timer = self.create_timer(0.05, self.publish_commands)

    def set_scenario(self, goal: List[float], obstacles: List[Dict]) -> None:
        self.goal = list(goal)
        self.obstacles = list(obstacles)

    def publish_commands(self) -> None:
        goal = PoseStamped()
        goal.header.frame_id = "base_link"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = self.goal
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

        array = MarkerArray()
        for idx, obstacle in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "debug_obstacles"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle["center"][0]
            marker.pose.position.y = obstacle["center"][1]
            marker.pose.position.z = obstacle["center"][2]
            marker.pose.orientation.w = 1.0
            radius = obstacle["radius"]
            marker.scale.x = radius * 2.0
            marker.scale.y = radius * 2.0
            marker.scale.z = radius * 2.0
            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 0.7
            array.markers.append(marker)
        self.obstacle_pub.publish(array)

    def on_debug_state(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 7:
            return
        self.last_debug = {
            "goal_distance": float(data[0]),
            "tcp_z": float(data[1]),
            "link6_z": float(data[2]),
            "external_clearance": float(data[3]),
            "body_clearance": float(data[4]),
            "joint_speed_norm": float(data[5]),
            "safety_stop": float(data[6]),
        }
        if len(data) >= 11:
            self.last_debug.update(
                {
                    "min_link_z": float(data[7]),
                    "min_joint_z": float(data[8]),
                    "min_control_point_z": float(data[9]),
                    "min_body_obstacle_z": float(data[10]),
                }
            )

    def on_eef_pose(self, msg: PoseStamped) -> None:
        self.last_eef = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    def on_joint_states(self, msg: JointState) -> None:
        self.last_joint_speed_norm = math.sqrt(sum(v * v for v in msg.velocity))

    def snapshot(self) -> Dict[str, float]:
        out = dict(self.last_debug)
        if "goal_distance" not in out and self.last_eef is not None:
            dx = self.goal[0] - self.last_eef[0]
            dy = self.goal[1] - self.last_eef[1]
            dz = self.goal[2] - self.last_eef[2]
            out["goal_distance"] = math.sqrt(dx * dx + dy * dy + dz * dz)
            out["tcp_z"] = self.last_eef[2]
        out.setdefault("joint_speed_norm", self.last_joint_speed_norm)
        return out


def run_scenario(
    node: RmpDebugNode,
    name: str,
    timeout: float,
    tolerance: float,
    settle_time: float,
) -> Dict:
    scenario = SCENARIOS[name]
    node.set_scenario(scenario["goal"], scenario["obstacles"])

    start = time.time()
    reached_since = None
    summary = {
        "scenario": name,
        "success": False,
        "min_goal_distance": float("inf"),
        "min_external_clearance": float("inf"),
        "min_body_clearance": float("inf"),
        "max_joint_speed_norm": 0.0,
        "min_tcp_z": float("inf"),
        "min_link6_z": float("inf"),
        "min_link_z": float("inf"),
        "min_joint_z": float("inf"),
        "min_control_point_z": float("inf"),
        "min_body_obstacle_z": float("inf"),
        "safety_stop_count": 0,
        "duration": 0.0,
    }

    last_print = 0.0
    while time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        snap = node.snapshot()
        if not snap:
            continue

        goal_distance = snap.get("goal_distance", float("inf"))
        tcp_z = snap.get("tcp_z", float("inf"))
        link6_z = snap.get("link6_z", float("inf"))
        ext_clearance = snap.get("external_clearance", float("inf"))
        body_clearance = snap.get("body_clearance", float("inf"))
        speed_norm = snap.get("joint_speed_norm", 0.0)
        safety_stop = snap.get("safety_stop", 0.0)
        min_link_z = snap.get("min_link_z", float("inf"))
        min_joint_z = snap.get("min_joint_z", float("inf"))
        min_control_point_z = snap.get("min_control_point_z", float("inf"))
        min_body_obstacle_z = snap.get("min_body_obstacle_z", float("inf"))

        summary["min_goal_distance"] = min(summary["min_goal_distance"], goal_distance)
        summary["min_external_clearance"] = min(summary["min_external_clearance"], ext_clearance)
        summary["min_body_clearance"] = min(summary["min_body_clearance"], body_clearance)
        summary["max_joint_speed_norm"] = max(summary["max_joint_speed_norm"], speed_norm)
        summary["min_tcp_z"] = min(summary["min_tcp_z"], tcp_z)
        summary["min_link6_z"] = min(summary["min_link6_z"], link6_z)
        summary["min_link_z"] = min(summary["min_link_z"], min_link_z)
        summary["min_joint_z"] = min(summary["min_joint_z"], min_joint_z)
        summary["min_control_point_z"] = min(summary["min_control_point_z"], min_control_point_z)
        summary["min_body_obstacle_z"] = min(summary["min_body_obstacle_z"], min_body_obstacle_z)
        if safety_stop > 0.5:
            summary["safety_stop_count"] += 1

        now = time.time() - start
        if now - last_print > 1.0:
            last_print = now
            print(
                f"[{name}] t={now:4.1f}s goal={goal_distance:0.4f} "
                f"ext={ext_clearance:0.4f} body={body_clearance:0.4f} "
                f"speed={speed_norm:0.4f} tcp_z={tcp_z:0.4f} link6_z={link6_z:0.4f} "
                f"min_link_z={min_link_z:0.4f} min_joint_z={min_joint_z:0.4f} "
                f"min_cp_z={min_control_point_z:0.4f} min_guard_z={min_body_obstacle_z:0.4f} "
                f"safety={int(safety_stop > 0.5)}"
            )

        if goal_distance <= tolerance and speed_norm <= 0.08:
            if reached_since is None:
                reached_since = time.time()
            elif time.time() - reached_since >= settle_time:
                summary["success"] = True
                break
        else:
            reached_since = None

    summary["duration"] = time.time() - start
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Headless RMP debug runner")
    parser.add_argument(
        "--scenario",
        action="append",
        choices=sorted(SCENARIOS.keys()),
        help="Scenario name to run. Defaults to all scenarios.",
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--tolerance", type=float, default=0.03)
    parser.add_argument("--settle-time", type=float, default=0.8)
    parser.add_argument("--json", action="store_true", help="Print JSON summary only")
    args = parser.parse_args()

    rclpy.init()
    node = RmpDebugNode()
    scenarios = args.scenario or list(SCENARIOS.keys())
    summaries = []
    try:
        for name in scenarios:
            print(f"=== Running scenario: {name} ===")
            summary = run_scenario(
                node,
                name=name,
                timeout=args.timeout,
                tolerance=args.tolerance,
                settle_time=args.settle_time,
            )
            summaries.append(summary)
            if not args.json:
                print(json.dumps(summary, indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if args.json:
        print(json.dumps(summaries, indent=2, sort_keys=True))

    failures = [
        s for s in summaries
        if (not s["success"]) or s["min_external_clearance"] < 0.0 or s["min_body_clearance"] < 0.0
    ]
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
