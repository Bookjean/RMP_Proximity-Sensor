#!/usr/bin/env python3

import argparse
import math
import os
import sys
import time
from datetime import datetime
from typing import Any, Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger


def default_pose_file() -> str:
    source_path = os.path.expanduser(
        "~/ros2_ws/src/RMP_Proximity-Sensor/config/experiment_goal_poses.yaml"
    )
    if os.path.isfile(source_path):
        return source_path
    try:
        share_dir = get_package_share_directory("rb10_rmpflow_rviz")
        return os.path.join(share_dir, "config", "experiment_goal_poses.yaml")
    except Exception:
        source_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        return os.path.join(source_root, "config", "experiment_goal_poses.yaml")


class ExperimentGoalRunner(Node):
    def __init__(
        self,
        goal_topic: str,
        ee_pose_topic: str,
        rmp_flag_topic: str,
        recorder_start_service: str,
        recorder_stop_service: str,
    ) -> None:
        super().__init__("experiment_goal_runner")
        self.latest_ee_pose: Optional[Pose] = None
        self.latest_ee_pose_time: Optional[float] = None
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.rmp_flag_pub = self.create_publisher(UInt8, rmp_flag_topic, 10)
        self.recorder_start_client = self.create_client(Trigger, recorder_start_service)
        self.recorder_stop_client = self.create_client(Trigger, recorder_stop_service)
        self.create_subscription(Pose, ee_pose_topic, self.on_ee_pose, 10)

    def on_ee_pose(self, msg: Pose) -> None:
        self.latest_ee_pose = msg
        self.latest_ee_pose_time = time.monotonic()

    def wait_for_goal_subscriber(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while time.monotonic() < deadline and rclpy.ok():
            if self.goal_pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.goal_pub.get_subscription_count() > 0

    def wait_for_current_pose(self, timeout_sec: float) -> Pose:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while time.monotonic() < deadline and rclpy.ok():
            if self.latest_ee_pose is not None:
                return self.latest_ee_pose
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError("Timed out waiting for current TCP pose.")

    def call_trigger(self, client, service_name: str, timeout_sec: float) -> str:
        if not client.wait_for_service(timeout_sec=max(timeout_sec, 0.0)):
            raise RuntimeError(f"Timed out waiting for service: {service_name}")

        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                response = future.result()
                if response is None:
                    raise RuntimeError(f"Service call failed: {service_name}")
                if not response.success:
                    raise RuntimeError(f"{service_name} returned failure: {response.message}")
                return str(response.message)
        raise RuntimeError(f"Timed out waiting for service response: {service_name}")

    def publish_rmp_flag(self, active_value: int) -> None:
        msg = UInt8()
        msg.data = max(0, min(int(active_value), 255))
        self.rmp_flag_pub.publish(msg)

    def publish_goal(
        self,
        frame_id: str,
        position: List[float],
        orientation: List[float],
    ) -> None:
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
        self.goal_pub.publish(msg)

    def publish_for_duration(
        self,
        frame_id: str,
        position: List[float],
        orientation: List[float],
        duration_sec: float,
        publish_rate_hz: float,
        activate_rmp: bool,
        rmp_active_value: int,
        goal_tolerance_m: float,
        goal_settle_sec: float,
    ) -> Dict[str, Optional[float]]:
        if publish_rate_hz <= 0.0:
            raise RuntimeError("publish_rate_hz must be greater than 0.")

        period_sec = 1.0 / publish_rate_hz
        start_time = time.monotonic()
        deadline = start_time + max(duration_sec, 0.0)
        reached_since: Optional[float] = None
        last_error_m: Optional[float] = None

        while time.monotonic() < deadline and rclpy.ok():
            if activate_rmp:
                self.publish_rmp_flag(rmp_active_value)
            self.publish_goal(frame_id, position, orientation)
            rclpy.spin_once(self, timeout_sec=0.0)

            last_error_m = self.position_error_m(position)
            if goal_tolerance_m > 0.0 and last_error_m is not None:
                now_time = time.monotonic()
                if last_error_m <= goal_tolerance_m:
                    if reached_since is None:
                        reached_since = now_time
                    if now_time - reached_since >= max(goal_settle_sec, 0.0):
                        return {
                            "reached": 1.0,
                            "elapsed_sec": now_time - start_time,
                            "last_error_m": last_error_m,
                        }
                else:
                    reached_since = None
            time.sleep(period_sec)

        return {
            "reached": 0.0,
            "elapsed_sec": time.monotonic() - start_time,
            "last_error_m": last_error_m,
        }

    def publish_until_reached(
        self,
        frame_id: str,
        position: List[float],
        orientation: List[float],
        publish_rate_hz: float,
        activate_rmp: bool,
        rmp_active_value: int,
        goal_tolerance_m: float,
        goal_settle_sec: float,
    ) -> Dict[str, Optional[float]]:
        if publish_rate_hz <= 0.0:
            raise RuntimeError("publish_rate_hz must be greater than 0.")
        if goal_tolerance_m <= 0.0:
            raise RuntimeError("goal_tolerance_m must be greater than 0 because duration timeout is disabled.")

        period_sec = 1.0 / publish_rate_hz
        start_time = time.monotonic()
        reached_since: Optional[float] = None
        last_error_m: Optional[float] = None

        while rclpy.ok():
            if activate_rmp:
                self.publish_rmp_flag(rmp_active_value)
            self.publish_goal(frame_id, position, orientation)
            rclpy.spin_once(self, timeout_sec=0.0)

            last_error_m = self.position_error_m(position)
            if last_error_m is not None:
                now_time = time.monotonic()
                if last_error_m <= goal_tolerance_m:
                    if reached_since is None:
                        reached_since = now_time
                    if now_time - reached_since >= max(goal_settle_sec, 0.0):
                        return {
                            "reached": 1.0,
                            "elapsed_sec": now_time - start_time,
                            "last_error_m": last_error_m,
                        }
                else:
                    reached_since = None
            time.sleep(period_sec)

        return {
            "reached": 0.0,
            "elapsed_sec": time.monotonic() - start_time,
            "last_error_m": last_error_m,
        }

    def spin_for_duration(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(duration_sec, 0.0)
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def position_error_m(self, target_position: List[float]) -> Optional[float]:
        if self.latest_ee_pose is None:
            return None
        dx = float(self.latest_ee_pose.position.x) - float(target_position[0])
        dy = float(self.latest_ee_pose.position.y) - float(target_position[1])
        dz = float(self.latest_ee_pose.position.z) - float(target_position[2])
        return math.sqrt(dx * dx + dy * dy + dz * dz)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Interactive/menu runner for proximity-aware RMPflow pose experiments. "
            "Without --pose or --trajectory, it opens a dataset_gen_selfcol-style menu."
        )
    )
    parser.add_argument(
        "--poses-file",
        default=default_pose_file(),
        help="YAML file containing named experiment poses and trajectories.",
    )
    parser.add_argument(
        "--pose",
        help="Run one pose non-interactively and exit.",
    )
    parser.add_argument(
        "--trajectory",
        help="Run one trajectory non-interactively and exit.",
    )
    parser.add_argument(
        "--stop",
        action="store_true",
        help="Overwrite the active goal with the current TCP pose, then publish zero on /RMP_flag.",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available pose and trajectory names and exit.",
    )
    parser.add_argument(
        "--goal-tolerance-m",
        type=float,
        help="Position tolerance for goal-reached termination in meters.",
    )
    parser.add_argument(
        "--goal-settle-sec",
        type=float,
        help="How long the TCP must remain inside tolerance before a goal is marked reached.",
    )
    parser.add_argument(
        "--publish-rate",
        type=float,
        help="Override goal publish rate in Hz.",
    )
    parser.add_argument(
        "--stop-publish-sec",
        type=float,
        default=1.0,
        help="How long to publish the current TCP pose as a hold goal when stopping.",
    )
    parser.add_argument(
        "--stop-publish-rate",
        type=float,
        default=20.0,
        help="Publish rate in Hz for the stop/hold-current command.",
    )
    parser.add_argument(
        "--frame-id",
        help="Override the frame_id from the YAML file.",
    )
    parser.add_argument(
        "--goal-topic",
        default="/goal_pose",
        help="PoseStamped topic consumed directly by rmpflow_controller.",
    )
    parser.add_argument(
        "--ee-pose-topic",
        default="/rmp_ee_pose",
        help="Current end-effector pose topic used by the menu save-current-pose option.",
    )
    parser.add_argument(
        "--wait-for-subscriber",
        type=float,
        default=5.0,
        help="How long to wait for a /goal_pose subscriber.",
    )
    parser.add_argument(
        "--wait-for-current-pose",
        type=float,
        default=5.0,
        help="How long to wait for /rmp_ee_pose when saving the current pose.",
    )
    parser.add_argument(
        "--record",
        dest="record",
        action="store_true",
        help="In non-interactive mode, start/stop rmp_data_recorder.",
    )
    parser.add_argument(
        "--no-record",
        dest="record",
        action="store_false",
        help="In non-interactive mode, run without recorder services.",
    )
    parser.set_defaults(record=False)
    parser.add_argument(
        "--recorder-start-service",
        default="/rmp_data_recorder/start",
        help="Trigger service used to start recording.",
    )
    parser.add_argument(
        "--recorder-stop-service",
        default="/rmp_data_recorder/stop",
        help="Trigger service used to stop recording.",
    )
    parser.add_argument(
        "--wait-for-service",
        type=float,
        default=5.0,
        help="How long to wait for recorder services.",
    )
    parser.add_argument(
        "--pre-record-sec",
        type=float,
        default=0.5,
        help="Delay after recorder start before sending the goal.",
    )
    parser.add_argument(
        "--post-record-sec",
        type=float,
        default=0.0,
        help="Extra time to keep publishing the last goal before stopping recorder.",
    )
    parser.add_argument(
        "--rmp-flag-topic",
        default="/RMP_flag",
        help="RMP activation flag topic.",
    )
    parser.add_argument(
        "--rmp-active-value",
        type=int,
        default=1,
        help="UInt8 value that enables RMP execution.",
    )
    parser.add_argument(
        "--no-activate-rmp",
        dest="activate_rmp",
        action="store_false",
        help="Do not publish /RMP_flag while publishing goals.",
    )
    parser.set_defaults(activate_rmp=True)
    parser.add_argument(
        "--stop-rmp-on-exit",
        action="store_true",
        help="Publish zero on /RMP_flag when the runner exits.",
    )

    args = parser.parse_args()
    requested_runs = [bool(args.pose), bool(args.trajectory), bool(args.stop)]
    if sum(requested_runs) > 1:
        parser.error("Use only one of --pose, --trajectory, or --stop.")
    if args.goal_tolerance_m is not None and args.goal_tolerance_m <= 0.0:
        parser.error("--goal-tolerance-m must be greater than 0.")
    if args.goal_settle_sec is not None and args.goal_settle_sec < 0.0:
        parser.error("--goal-settle-sec must be non-negative.")
    if args.publish_rate is not None and args.publish_rate <= 0.0:
        parser.error("--publish-rate must be greater than 0.")
    if args.stop_publish_sec <= 0.0:
        parser.error("--stop-publish-sec must be greater than 0.")
    if args.stop_publish_rate <= 0.0:
        parser.error("--stop-publish-rate must be greater than 0.")
    if args.pre_record_sec < 0.0:
        parser.error("--pre-record-sec must be non-negative.")
    if args.post_record_sec < 0.0:
        parser.error("--post-record-sec must be non-negative.")
    return args


def load_pose_file(path: str) -> Dict[str, Any]:
    expanded_path = os.path.expanduser(path)
    with open(expanded_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"Pose file must contain a YAML mapping: {expanded_path}")
    poses = data.get("poses")
    if not isinstance(poses, dict) or not poses:
        raise RuntimeError(f"Pose file must define a non-empty 'poses' mapping: {expanded_path}")
    if "trajectories" in data and not isinstance(data["trajectories"], dict):
        raise RuntimeError("'trajectories' must be a YAML mapping when provided.")
    return data


def save_pose_file(path: str, data: Dict[str, Any]) -> None:
    expanded_path = os.path.expanduser(path)
    directory = os.path.dirname(expanded_path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(expanded_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, default_flow_style=False, sort_keys=False)


def list_pose_names(data: Dict[str, Any]) -> None:
    default_pose = data.get("default_pose")
    print("Poses:")
    for index, name in enumerate(data["poses"].keys(), start=1):
        suffix = " (default)" if name == default_pose else ""
        print(f"  {index}: {name}{suffix}")


def list_trajectory_names(data: Dict[str, Any]) -> None:
    trajectories = data.get("trajectories", {})
    print("Trajectories:")
    if not trajectories:
        print("  none")
        return
    for index, name in enumerate(trajectories.keys(), start=1):
        print(f"  {index}: {name}")


def list_all(data: Dict[str, Any]) -> None:
    list_pose_names(data)
    print()
    list_trajectory_names(data)


def read_vector(entry: Dict[str, Any], keys: List[str], length: int, label: str) -> List[float]:
    value = None
    for key in keys:
        if key in entry:
            value = entry[key]
            break
    if not isinstance(value, list) or len(value) != length:
        raise RuntimeError(f"{label} must be a list of {length} numbers.")
    return [float(item) for item in value]


def normalize_quaternion(quat: List[float]) -> List[float]:
    norm = math.sqrt(sum(value * value for value in quat))
    if norm <= 1e-9:
        raise RuntimeError("Orientation quaternion norm is zero.")
    return [value / norm for value in quat]


def resolve_pose(
    data: Dict[str, Any],
    pose_name: str,
    frame_id_override: Optional[str] = None,
    publish_rate_override: Optional[float] = None,
    goal_tolerance_override: Optional[float] = None,
    goal_settle_override: Optional[float] = None,
) -> Dict[str, Any]:
    if pose_name not in data["poses"]:
        available = ", ".join(str(name) for name in data["poses"].keys())
        raise RuntimeError(f"Unknown pose '{pose_name}'. Available poses: {available}")

    entry = data["poses"][pose_name]
    if not isinstance(entry, dict):
        raise RuntimeError(f"Pose '{pose_name}' must be a YAML mapping.")

    position = read_vector(entry, ["position", "position_xyz"], 3, "position")
    orientation = normalize_quaternion(
        read_vector(entry, ["orientation", "orientation_xyzw"], 4, "orientation")
    )
    frame_id = frame_id_override or entry.get("frame_id") or data.get("frame_id", "base_link")
    publish_rate_hz = (
        publish_rate_override
        if publish_rate_override is not None
        else float(entry.get("publish_rate_hz", data.get("publish_rate_hz", 20.0)))
    )
    goal_tolerance_m = (
        goal_tolerance_override
        if goal_tolerance_override is not None
        else float(entry.get("goal_tolerance_m", data.get("default_goal_tolerance_m", 0.005)))
    )
    goal_settle_sec = (
        goal_settle_override
        if goal_settle_override is not None
        else float(entry.get("goal_settle_sec", entry.get("settle_sec", data.get("default_goal_settle_sec", 0.3))))
    )

    return {
        "name": pose_name,
        "frame_id": str(frame_id),
        "position": position,
        "orientation": orientation,
        "publish_rate_hz": float(publish_rate_hz),
        "goal_tolerance_m": float(goal_tolerance_m),
        "goal_settle_sec": float(goal_settle_sec),
    }


def resolve_trajectory(
    data: Dict[str, Any],
    trajectory_name: str,
    frame_id_override: Optional[str] = None,
    publish_rate_override: Optional[float] = None,
    goal_tolerance_override: Optional[float] = None,
    goal_settle_override: Optional[float] = None,
) -> Dict[str, Any]:
    trajectories = data.get("trajectories", {})
    if trajectory_name not in trajectories:
        available = ", ".join(str(name) for name in trajectories.keys()) or "none"
        raise RuntimeError(f"Unknown trajectory '{trajectory_name}'. Available trajectories: {available}")

    entry = trajectories[trajectory_name]
    wait_sec = 0.0
    pose_names: List[str]
    if isinstance(entry, list):
        pose_names = [str(name) for name in entry]
    elif isinstance(entry, dict):
        raw_poses = entry.get("poses", entry.get("waypoints"))
        if not isinstance(raw_poses, list) or not raw_poses:
            raise RuntimeError(f"Trajectory '{trajectory_name}' requires poses: [name, ...].")
        pose_names = [str(name) for name in raw_poses]
        wait_sec = float(entry.get("wait_sec", entry.get("wait_between_sec", 0.0)))
    else:
        raise RuntimeError(f"Trajectory '{trajectory_name}' must be a list or mapping.")

    poses = [
        resolve_pose(
            data=data,
            pose_name=pose_name,
            frame_id_override=frame_id_override,
            publish_rate_override=publish_rate_override,
            goal_tolerance_override=goal_tolerance_override,
            goal_settle_override=goal_settle_override,
        )
        for pose_name in pose_names
    ]
    return {
        "name": trajectory_name,
        "poses": poses,
        "wait_sec": max(wait_sec, 0.0),
    }


def print_pose_summary(pose: Dict[str, Any], prefix: str = "Pose") -> None:
    print(
        f"{prefix}: {pose['name']} "
        f"pos=({pose['position'][0]:.3f}, {pose['position'][1]:.3f}, {pose['position'][2]:.3f}) "
        f"quat=({pose['orientation'][0]:.4f}, {pose['orientation'][1]:.4f}, "
        f"{pose['orientation'][2]:.4f}, {pose['orientation'][3]:.4f}) "
        f"tol={pose['goal_tolerance_m']:.4f}m "
        f"settle={pose['goal_settle_sec']:.2f}s"
    )


def wait_for_subscriber_once(node: ExperimentGoalRunner, args: argparse.Namespace) -> None:
    subscriber_found = node.wait_for_goal_subscriber(args.wait_for_subscriber)
    if not subscriber_found:
        node.get_logger().warning(f"No subscriber detected on {args.goal_topic}; publishing anyway.")


def start_recording(node: ExperimentGoalRunner, args: argparse.Namespace) -> bool:
    try:
        message = node.call_trigger(
            node.recorder_start_client,
            args.recorder_start_service,
            args.wait_for_service,
        )
    except RuntimeError as exc:
        if "Timed out waiting for service" in str(exc):
            raise RuntimeError(
                "Recorder service is not available. Launch with record_data:=true "
                "and preferably auto_start_recording:=false when using menu option 5."
            ) from exc
        raise
    print(message)
    if args.pre_record_sec > 0.0:
        node.spin_for_duration(args.pre_record_sec)
    return True


def stop_recording(node: ExperimentGoalRunner, args: argparse.Namespace) -> None:
    message = node.call_trigger(
        node.recorder_stop_client,
        args.recorder_stop_service,
        args.wait_for_service,
    )
    print(message)


def publish_stop_flag(node: ExperimentGoalRunner) -> None:
    for _ in range(3):
        node.publish_rmp_flag(0)
        rclpy.spin_once(node, timeout_sec=0.05)


def stop_command(
    node: ExperimentGoalRunner,
    args: argparse.Namespace,
    frame_id: str,
) -> None:
    wait_for_subscriber_once(node, args)
    print("Waiting for current TCP pose...")
    pose = node.wait_for_current_pose(args.wait_for_current_pose)
    position = [
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
    ]
    orientation = normalize_quaternion([
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    ])

    print(
        "Publishing current TCP pose as hold goal: "
        f"pos=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
    )
    node.publish_for_duration(
        frame_id=frame_id,
        position=position,
        orientation=orientation,
        duration_sec=args.stop_publish_sec,
        publish_rate_hz=args.stop_publish_rate,
        activate_rmp=args.activate_rmp,
        rmp_active_value=args.rmp_active_value,
        goal_tolerance_m=0.0,
        goal_settle_sec=0.0,
    )
    publish_stop_flag(node)
    print("Stop command complete: current TCP pose is now the held /goal_pose, then /RMP_flag = 0.")


def print_run_result(result: Dict[str, Optional[float]]) -> None:
    last_error = result.get("last_error_m")
    elapsed = result.get("elapsed_sec")
    if result.get("reached", 0.0) >= 0.5:
        print(
            f"Reached goal: elapsed={elapsed:.2f}s, error={last_error:.4f}m"
            if elapsed is not None and last_error is not None
            else "Reached goal"
        )
    else:
        print(
            f"Goal run stopped before reaching: elapsed={elapsed:.2f}s, last_error={last_error:.4f}m"
            if elapsed is not None and last_error is not None
            else "Goal run stopped before reaching"
        )


def run_pose(
    node: ExperimentGoalRunner,
    args: argparse.Namespace,
    pose: Dict[str, Any],
    record: bool,
) -> None:
    wait_for_subscriber_once(node, args)
    print_pose_summary(pose, "Pose")
    print(f"Mode: {'WITH recording' if record else 'WITHOUT recording'}")

    recording_started = False
    try:
        if record:
            recording_started = start_recording(node, args)

        result = node.publish_until_reached(
            frame_id=pose["frame_id"],
            position=pose["position"],
            orientation=pose["orientation"],
            publish_rate_hz=pose["publish_rate_hz"],
            activate_rmp=args.activate_rmp,
            rmp_active_value=args.rmp_active_value,
            goal_tolerance_m=pose["goal_tolerance_m"],
            goal_settle_sec=pose["goal_settle_sec"],
        )
        print_run_result(result)

        if record and args.post_record_sec > 0.0 and result.get("reached", 0.0) < 0.5:
            node.publish_for_duration(
                frame_id=pose["frame_id"],
                position=pose["position"],
                orientation=pose["orientation"],
                duration_sec=args.post_record_sec,
                publish_rate_hz=pose["publish_rate_hz"],
                activate_rmp=args.activate_rmp,
                rmp_active_value=args.rmp_active_value,
                goal_tolerance_m=0.0,
                goal_settle_sec=0.0,
            )
    finally:
        if record and recording_started:
            stop_recording(node, args)


def run_trajectory(
    node: ExperimentGoalRunner,
    args: argparse.Namespace,
    trajectory: Dict[str, Any],
    record: bool,
) -> None:
    wait_for_subscriber_once(node, args)
    print(f"Trajectory: {trajectory['name']} ({len(trajectory['poses'])} poses)")
    print(f"Mode: {'WITH recording' if record else 'WITHOUT recording'}")
    for index, pose in enumerate(trajectory["poses"], start=1):
        print_pose_summary(pose, f"  [{index}/{len(trajectory['poses'])}]")

    recording_started = False
    try:
        if record:
            recording_started = start_recording(node, args)

        for index, pose in enumerate(trajectory["poses"], start=1):
            print(f"Running [{index}/{len(trajectory['poses'])}] {pose['name']}")
            result = node.publish_until_reached(
                frame_id=pose["frame_id"],
                position=pose["position"],
                orientation=pose["orientation"],
                publish_rate_hz=pose["publish_rate_hz"],
                activate_rmp=args.activate_rmp,
                rmp_active_value=args.rmp_active_value,
                goal_tolerance_m=pose["goal_tolerance_m"],
                goal_settle_sec=pose["goal_settle_sec"],
            )
            print_run_result(result)
            if trajectory["wait_sec"] > 0.0 and index < len(trajectory["poses"]):
                node.spin_for_duration(trajectory["wait_sec"])

        if record and args.post_record_sec > 0.0 and trajectory["poses"]:
            last_pose = trajectory["poses"][-1]
            node.publish_for_duration(
                frame_id=last_pose["frame_id"],
                position=last_pose["position"],
                orientation=last_pose["orientation"],
                duration_sec=args.post_record_sec,
                publish_rate_hz=last_pose["publish_rate_hz"],
                activate_rmp=args.activate_rmp,
                rmp_active_value=args.rmp_active_value,
                goal_tolerance_m=0.0,
                goal_settle_sec=0.0,
            )
    finally:
        if record and recording_started:
            stop_recording(node, args)


def prompt_optional_float(prompt: str) -> Optional[float]:
    raw = input(prompt).strip()
    if not raw:
        return None
    return float(raw)


def confirm(prompt: str = "Continue? (y/n): ") -> bool:
    return input(prompt).strip().lower() == "y"


def pose_msg_to_yaml_entry(
    pose: Pose,
    frame_id: Optional[str] = None,
) -> Dict[str, Any]:
    entry: Dict[str, Any] = {
        "position": [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        ],
        "orientation": normalize_quaternion([
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ]),
    }
    if frame_id:
        entry["frame_id"] = frame_id
    return entry


def save_current_pose_menu(
    node: ExperimentGoalRunner,
    args: argparse.Namespace,
    data: Dict[str, Any],
    poses_file: str,
) -> None:
    default_name = datetime.now().strftime("pose_%Y%m%d_%H%M%S")
    expanded_path = os.path.expanduser(poses_file)
    print(f"YAML file: {expanded_path}")
    pose_name = input(f"Enter name for current pose [{default_name}]: ").strip()
    if not pose_name:
        pose_name = default_name

    if pose_name in data["poses"]:
        print(f"Pose '{pose_name}' already exists.")
        if not confirm("Overwrite? (y/n): "):
            return

    print("Waiting for current TCP pose...")
    pose = node.wait_for_current_pose(args.wait_for_current_pose)
    frame_id = args.frame_id or data.get("frame_id", "base_link")
    data["poses"][pose_name] = pose_msg_to_yaml_entry(pose, frame_id=None)
    save_pose_file(poses_file, data)

    saved_pose = resolve_pose(
        data=data,
        pose_name=pose_name,
        frame_id_override=frame_id,
    )
    print_pose_summary(saved_pose, "Saved current pose")
    print(f"File: {os.path.expanduser(poses_file)}")


def select_name(mapping: Dict[str, Any], label: str) -> Optional[str]:
    if not mapping:
        print(f"No {label} entries are loaded.")
        return None
    names = list(mapping.keys())
    print(f"Available {label}:")
    for index, name in enumerate(names, start=1):
        print(f"  {index}: {name}")
    raw = input(f"Enter {label} name or number: ").strip()
    if not raw:
        return None
    if raw.isdigit():
        index = int(raw)
        if 1 <= index <= len(names):
            return str(names[index - 1])
    if raw in mapping:
        return str(raw)
    print(f"Invalid {label}: {raw}")
    return None


def interactive_menu(
    node: ExperimentGoalRunner,
    args: argparse.Namespace,
    initial_data: Dict[str, Any],
) -> None:
    data = initial_data
    poses_file = args.poses_file

    print("=" * 80)
    print("RMPflow Proximity Experiment Runner")
    print("=" * 80)
    print(f"Loaded YAML: {os.path.expanduser(poses_file)}")

    while rclpy.ok():
        print("\n" + "-" * 80)
        print("Select option:")
        print("  1: Load goal poses YAML")
        print("  2: List poses and trajectories")
        print("  3: Save current TCP pose to YAML")
        print("  4: Run one pose WITHOUT recording")
        print("  5: Run one pose WITH recording")
        print("  6: Run trajectory WITHOUT recording")
        print("  7: Run trajectory WITH recording")
        print("  8: Stop command / hold current TCP pose")
        print("  0: EXIT")
        print("-" * 80)

        try:
            command = input("Enter option: ").strip()
        except KeyboardInterrupt:
            print("\nExiting...")
            break

        try:
            if command == "0":
                print("EXITING THE PROGRAM...")
                break

            if command == "1":
                path = input(f"Enter YAML file path [{poses_file}]: ").strip()
                if not path:
                    path = poses_file
                data = load_pose_file(path)
                poses_file = path
                print(f"Loaded {len(data['poses'])} poses from {os.path.expanduser(path)}")
                list_trajectory_names(data)

            elif command == "2":
                list_all(data)

            elif command == "3":
                save_current_pose_menu(node, args, data, poses_file)

            elif command in {"4", "5"}:
                pose_name = select_name(data["poses"], "pose")
                if pose_name is None:
                    continue
                goal_tolerance = prompt_optional_float(
                    "Goal tolerance override m [YAML/default]: "
                )
                goal_settle = prompt_optional_float("Goal settle override sec [YAML/default]: ")
                pose = resolve_pose(
                    data=data,
                    pose_name=pose_name,
                    frame_id_override=args.frame_id,
                    publish_rate_override=args.publish_rate,
                    goal_tolerance_override=(
                        goal_tolerance if goal_tolerance is not None else args.goal_tolerance_m
                    ),
                    goal_settle_override=(
                        goal_settle if goal_settle is not None else args.goal_settle_sec
                    ),
                )
                record = command == "5"
                print_pose_summary(pose, "Selected pose")
                print(f"Run {'WITH' if record else 'WITHOUT'} recording?")
                if confirm():
                    run_pose(node, args, pose, record=record)

            elif command in {"6", "7"}:
                trajectories = data.get("trajectories", {})
                trajectory_name = select_name(trajectories, "trajectory")
                if trajectory_name is None:
                    continue
                goal_tolerance = prompt_optional_float(
                    "Goal tolerance override m [YAML/default]: "
                )
                goal_settle = prompt_optional_float("Goal settle override sec [YAML/default]: ")
                trajectory = resolve_trajectory(
                    data=data,
                    trajectory_name=trajectory_name,
                    frame_id_override=args.frame_id,
                    publish_rate_override=args.publish_rate,
                    goal_tolerance_override=(
                        goal_tolerance if goal_tolerance is not None else args.goal_tolerance_m
                    ),
                    goal_settle_override=(
                        goal_settle if goal_settle is not None else args.goal_settle_sec
                    ),
                )
                record = command == "7"
                print(f"Selected trajectory: {trajectory['name']}")
                print(f"Run {'WITH' if record else 'WITHOUT'} recording?")
                if confirm():
                    run_trajectory(node, args, trajectory, record=record)

            elif command == "8":
                frame_id = args.frame_id or data.get("frame_id", "base_link")
                stop_command(node, args, str(frame_id))

            else:
                print("Invalid option")
        except Exception as exc:
            print(f"ERROR: {exc}")


def main() -> int:
    args = parse_args()
    data: Optional[Dict[str, Any]] = None
    if not args.stop:
        data = load_pose_file(args.poses_file)

    if args.list:
        if data is None:
            data = load_pose_file(args.poses_file)
        list_all(data)
        return 0

    rclpy.init()
    node = ExperimentGoalRunner(
        goal_topic=args.goal_topic,
        ee_pose_topic=args.ee_pose_topic,
        rmp_flag_topic=args.rmp_flag_topic,
        recorder_start_service=args.recorder_start_service,
        recorder_stop_service=args.recorder_stop_service,
    )

    try:
        if args.stop:
            stop_command(node, args, args.frame_id or "base_link")
        elif args.pose:
            if data is None:
                data = load_pose_file(args.poses_file)
            pose = resolve_pose(
                data=data,
                pose_name=args.pose,
                frame_id_override=args.frame_id,
                publish_rate_override=args.publish_rate,
                goal_tolerance_override=args.goal_tolerance_m,
                goal_settle_override=args.goal_settle_sec,
            )
            run_pose(node, args, pose, record=args.record)
        elif args.trajectory:
            if data is None:
                data = load_pose_file(args.poses_file)
            trajectory = resolve_trajectory(
                data=data,
                trajectory_name=args.trajectory,
                frame_id_override=args.frame_id,
                publish_rate_override=args.publish_rate,
                goal_tolerance_override=args.goal_tolerance_m,
                goal_settle_override=args.goal_settle_sec,
            )
            run_trajectory(node, args, trajectory, record=args.record)
        else:
            if data is None:
                data = load_pose_file(args.poses_file)
            interactive_menu(node, args, data)
    finally:
        if args.stop_rmp_on_exit:
            publish_stop_flag(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
