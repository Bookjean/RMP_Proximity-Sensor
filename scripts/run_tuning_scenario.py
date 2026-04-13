#!/usr/bin/env python3

import argparse
import copy
import math
import os
import sys
import time
from typing import Any, Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.node import Node


def default_scenario_path() -> str:
    try:
        share_dir = get_package_share_directory("rb10_rmpflow_rviz")
        return os.path.join(
            share_dir,
            "config",
            "tuning_scenarios",
            "target_translation_baseline.yaml",
        )
    except Exception:
        source_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        return os.path.join(
            source_root,
            "config",
            "tuning_scenarios",
            "target_translation_baseline.yaml",
        )


class TuningScenarioNode(Node):
    def __init__(
        self,
        ee_pose_topic: str,
        point_topic: str,
        pose_topic: str,
    ) -> None:
        super().__init__("run_tuning_scenario")
        self.latest_ee_pose: Optional[Pose] = None
        self.point_pub = self.create_publisher(Point, point_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.create_subscription(Pose, ee_pose_topic, self.on_ee_pose, 10)

    def on_ee_pose(self, msg: Pose) -> None:
        self.latest_ee_pose = copy.deepcopy(msg)

    def wait_for_startup_pose(self, timeout_sec: float) -> Pose:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while time.monotonic() < deadline and rclpy.ok():
            if self.latest_ee_pose is not None:
                return copy.deepcopy(self.latest_ee_pose)
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(
            f"Timed out waiting for first end-effector pose on the subscribed topic."
        )

    def wait_for_goal_subscribers(self, use_pose: bool, timeout_sec: float) -> bool:
        publisher = self.pose_pub if use_pose else self.point_pub
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while time.monotonic() < deadline and rclpy.ok():
            if publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return publisher.get_subscription_count() > 0

    def publish_goal(
        self,
        position: List[float],
        orientation: List[float],
        frame_id: str,
        use_pose: bool,
    ) -> None:
        if use_pose:
            msg = PoseStamped()
            msg.header.frame_id = frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = position[0]
            msg.pose.position.y = position[1]
            msg.pose.position.z = position[2]
            msg.pose.orientation.x = orientation[0]
            msg.pose.orientation.y = orientation[1]
            msg.pose.orientation.z = orientation[2]
            msg.pose.orientation.w = orientation[3]
            self.pose_pub.publish(msg)
            return

        msg = Point()
        msg.x = position[0]
        msg.y = position[1]
        msg.z = position[2]
        self.point_pub.publish(msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a repeatable RB10 tuning scenario by capturing the startup TCP pose "
            "and publishing a fixed sequence of relative goals."
        )
    )
    parser.add_argument(
        "--scenario",
        default=default_scenario_path(),
        help="Path to the tuning scenario YAML file.",
    )
    parser.add_argument(
        "--ee-pose-topic",
        default="/rmp_ee_pose",
        help="Controller pose topic used to capture the startup TCP pose.",
    )
    parser.add_argument(
        "--point-topic",
        default="/goal_position_cmd",
        help="Goal point command topic consumed by interactive_goal.",
    )
    parser.add_argument(
        "--pose-topic",
        default="/goal_pose_cmd",
        help="Goal pose command topic consumed by interactive_goal.",
    )
    parser.add_argument(
        "--wait-for-pose",
        type=float,
        default=5.0,
        help="How long to wait for the first startup TCP pose.",
    )
    parser.add_argument(
        "--wait-for-subscriber",
        type=float,
        default=2.0,
        help="How long to wait for the goal subscriber before publishing.",
    )
    parser.add_argument(
        "--loops",
        type=int,
        default=1,
        help="How many times to repeat the scenario sequence.",
    )
    args = parser.parse_args()
    if args.loops < 1:
        parser.error("--loops must be at least 1.")
    return args


def load_scenario(path: str) -> Dict[str, Any]:
    with open(os.path.expanduser(path), "r", encoding="utf-8") as handle:
        scenario = yaml.safe_load(handle) or {}
    if not isinstance(scenario, dict):
        raise RuntimeError(f"Scenario file must contain a YAML mapping: {path}")
    if not isinstance(scenario.get("steps"), list) or not scenario["steps"]:
        raise RuntimeError(f"Scenario file does not define any steps: {path}")
    return scenario


def pose_to_position_list(pose: Pose) -> List[float]:
    return [pose.position.x, pose.position.y, pose.position.z]


def pose_to_orientation_list(pose: Pose) -> List[float]:
    return [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]


def position_error_m(current_pose: Optional[Pose], target_position: List[float]) -> Optional[float]:
    if current_pose is None:
        return None
    dx = float(current_pose.position.x) - float(target_position[0])
    dy = float(current_pose.position.y) - float(target_position[1])
    dz = float(current_pose.position.z) - float(target_position[2])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def resolve_step(
    step: Dict[str, Any],
    startup_position: List[float],
    startup_orientation: List[float],
    default_use_pose: bool,
) -> Dict[str, Any]:
    target_mode = str(step.get("target", "startup")).strip().lower()
    position: List[float]

    if target_mode == "startup":
        position = list(startup_position)
    elif target_mode == "startup_offset":
        offset = step.get("offset_xyz", [0.0, 0.0, 0.0])
        if not isinstance(offset, list) or len(offset) != 3:
            raise RuntimeError("startup_offset steps require offset_xyz: [x, y, z].")
        position = [
            startup_position[index] + float(offset[index])
            for index in range(3)
        ]
    elif target_mode == "absolute":
        absolute = step.get("position_xyz")
        if not isinstance(absolute, list) or len(absolute) != 3:
            raise RuntimeError("absolute steps require position_xyz: [x, y, z].")
        position = [float(value) for value in absolute]
    else:
        raise RuntimeError(
            f"Unsupported step target '{target_mode}'. "
            "Use startup, startup_offset, or absolute."
        )

    orientation = list(startup_orientation)
    orientation_override = step.get("orientation_xyzw")
    if orientation_override is not None:
        if not isinstance(orientation_override, list) or len(orientation_override) != 4:
            raise RuntimeError("orientation_xyzw must contain 4 numbers.")
        orientation = [float(value) for value in orientation_override]

    completion = str(step.get("completion", "time")).strip().lower()
    if completion not in {"time", "reach"}:
        raise RuntimeError(
            f"Unsupported completion mode '{completion}'. Use time or reach."
        )

    hold_sec = float(step.get("hold_sec", 4.0))
    max_wait_sec = float(step.get("max_wait_sec", hold_sec if hold_sec > 0.0 else 20.0))
    position_tolerance_m = float(step.get("position_tolerance_m", 0.01))
    settle_sec = float(step.get("settle_sec", 0.0))

    return {
        "name": str(step.get("name", target_mode)),
        "hold_sec": hold_sec,
        "use_pose": bool(step.get("use_pose", default_use_pose)),
        "position": position,
        "orientation": orientation,
        "completion": completion,
        "max_wait_sec": max_wait_sec,
        "position_tolerance_m": position_tolerance_m,
        "settle_sec": settle_sec,
    }


def publish_hold(
    node: TuningScenarioNode,
    frame_id: str,
    use_pose: bool,
    position: List[float],
    orientation: List[float],
    hold_sec: float,
    publish_rate_hz: float,
) -> None:
    if publish_rate_hz <= 0.0:
        raise RuntimeError("publish_rate_hz must be greater than 0.")
    deadline = time.monotonic() + max(hold_sec, 0.0)
    period_sec = 1.0 / publish_rate_hz
    while time.monotonic() < deadline and rclpy.ok():
        node.publish_goal(position, orientation, frame_id, use_pose)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period_sec)


def publish_until_reached(
    node: TuningScenarioNode,
    frame_id: str,
    use_pose: bool,
    position: List[float],
    orientation: List[float],
    max_wait_sec: float,
    publish_rate_hz: float,
    position_tolerance_m: float,
    settle_sec: float,
) -> Dict[str, Optional[float]]:
    if publish_rate_hz <= 0.0:
        raise RuntimeError("publish_rate_hz must be greater than 0.")
    if position_tolerance_m <= 0.0:
        raise RuntimeError("position_tolerance_m must be greater than 0.")

    deadline = time.monotonic() + max(max_wait_sec, 0.0)
    period_sec = 1.0 / publish_rate_hz
    last_error_m: Optional[float] = None

    while time.monotonic() < deadline and rclpy.ok():
        node.publish_goal(position, orientation, frame_id, use_pose)
        rclpy.spin_once(node, timeout_sec=0.0)
        last_error_m = position_error_m(node.latest_ee_pose, position)
        if last_error_m is not None and last_error_m <= position_tolerance_m:
            if settle_sec > 0.0:
                publish_hold(
                    node=node,
                    frame_id=frame_id,
                    use_pose=use_pose,
                    position=position,
                    orientation=orientation,
                    hold_sec=settle_sec,
                    publish_rate_hz=publish_rate_hz,
                )
                last_error_m = position_error_m(node.latest_ee_pose, position)
            return {
                "reached": 1.0,
                "last_error_m": last_error_m,
            }
        time.sleep(period_sec)

    return {
        "reached": 0.0,
        "last_error_m": last_error_m,
    }


def main() -> int:
    args = parse_args()
    scenario = load_scenario(args.scenario)

    scenario_name = str(scenario.get("name", "unnamed_scenario"))
    frame_id = str(scenario.get("frame_id", "base_link"))
    publish_rate_hz = float(scenario.get("publish_rate_hz", 10.0))
    startup_settle_sec = float(scenario.get("startup_settle_sec", 2.0))
    default_use_pose = bool(scenario.get("use_pose", False))

    rclpy.init()
    node = TuningScenarioNode(
        ee_pose_topic=args.ee_pose_topic,
        point_topic=args.point_topic,
        pose_topic=args.pose_topic,
    )

    try:
        startup_pose = node.wait_for_startup_pose(args.wait_for_pose)
        startup_position = pose_to_position_list(startup_pose)
        startup_orientation = pose_to_orientation_list(startup_pose)

        resolved_steps = [
            resolve_step(step, startup_position, startup_orientation, default_use_pose)
            for step in scenario["steps"]
        ]

        any_pose_steps = any(step["use_pose"] for step in resolved_steps)
        subscriber_found = node.wait_for_goal_subscribers(
            use_pose=any_pose_steps,
            timeout_sec=args.wait_for_subscriber,
        )
        if not subscriber_found:
            node.get_logger().warning(
                "No goal subscriber detected on the command topic yet; publishing anyway."
            )

        print(f"Scenario: {scenario_name}")
        print(
            "Startup TCP pose: "
            f"pos=({startup_position[0]:.3f}, {startup_position[1]:.3f}, {startup_position[2]:.3f}) "
            f"quat=({startup_orientation[0]:.4f}, {startup_orientation[1]:.4f}, "
            f"{startup_orientation[2]:.4f}, {startup_orientation[3]:.4f})"
        )
        for index, step in enumerate(resolved_steps, start=1):
            if step["completion"] == "reach":
                print(
                    f"  Step {index}: {step['name']} -> "
                    f"({step['position'][0]:.3f}, {step['position'][1]:.3f}, {step['position'][2]:.3f}), "
                    f"until reached (tol={step['position_tolerance_m']:.4f}m, "
                    f"timeout={step['max_wait_sec']:.1f}s), "
                    f"mode={'pose' if step['use_pose'] else 'point'}"
                )
            else:
                print(
                    f"  Step {index}: {step['name']} -> "
                    f"({step['position'][0]:.3f}, {step['position'][1]:.3f}, {step['position'][2]:.3f}), "
                    f"hold={step['hold_sec']:.1f}s, mode={'pose' if step['use_pose'] else 'point'}"
                )

        if startup_settle_sec > 0.0:
            print(f"Settling at startup pose for {startup_settle_sec:.1f}s")
            publish_hold(
                node,
                frame_id=frame_id,
                use_pose=default_use_pose,
                position=startup_position,
                orientation=startup_orientation,
                hold_sec=startup_settle_sec,
                publish_rate_hz=publish_rate_hz,
            )

        for loop_index in range(args.loops):
            print(f"Running loop {loop_index + 1}/{args.loops}")
            for index, step in enumerate(resolved_steps, start=1):
                if step["completion"] == "reach":
                    print(
                        f"  [{index}/{len(resolved_steps)}] {step['name']} "
                        f"until reached (tol={step['position_tolerance_m']:.4f}m, "
                        f"timeout={step['max_wait_sec']:.1f}s)"
                    )
                    result = publish_until_reached(
                        node=node,
                        frame_id=frame_id,
                        use_pose=step["use_pose"],
                        position=step["position"],
                        orientation=step["orientation"],
                        max_wait_sec=step["max_wait_sec"],
                        publish_rate_hz=publish_rate_hz,
                        position_tolerance_m=step["position_tolerance_m"],
                        settle_sec=step["settle_sec"],
                    )
                    if result["reached"] >= 0.5:
                        last_error = result["last_error_m"]
                        print(
                            f"    reached: error={last_error:.4f}m"
                            if last_error is not None
                            else "    reached"
                        )
                    else:
                        last_error = result["last_error_m"]
                        print(
                            f"    timeout: last_error={last_error:.4f}m"
                            if last_error is not None
                            else "    timeout: no ee pose sample"
                        )
                else:
                    print(
                        f"  [{index}/{len(resolved_steps)}] {step['name']} "
                        f"for {step['hold_sec']:.1f}s"
                    )
                    publish_hold(
                        node,
                        frame_id=frame_id,
                        use_pose=step["use_pose"],
                        position=step["position"],
                        orientation=step["orientation"],
                        hold_sec=step["hold_sec"],
                        publish_rate_hz=publish_rate_hz,
                    )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
