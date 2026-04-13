#!/usr/bin/env python3

import argparse
import copy
import time
from pathlib import Path
from typing import Any, Dict, List

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
import yaml


YAML_BOOLISH_STRINGS = {
    "y",
    "n",
    "yes",
    "no",
    "on",
    "off",
    "true",
    "false",
    "null",
    "~",
}


class ScenarioYamlDumper(yaml.SafeDumper):
    pass


def represent_scenario_string(dumper: yaml.SafeDumper, data: str):
    style = '"' if data.strip().lower() in YAML_BOOLISH_STRINGS else None
    return dumper.represent_scalar("tag:yaml.org,2002:str", data, style=style)


ScenarioYamlDumper.add_representer(str, represent_scenario_string)


class EePoseCaptureNode(Node):
    def __init__(self, topic: str):
        super().__init__("capture_goal_from_ee_pose")
        self.latest_pose: Pose | None = None
        self.create_subscription(Pose, topic, self.on_pose, 10)

    def on_pose(self, msg: Pose) -> None:
        self.latest_pose = copy.deepcopy(msg)

    def wait_for_pose(self, timeout_sec: float) -> Pose:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.latest_pose is not None:
                return copy.deepcopy(self.latest_pose)
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError("Timed out waiting for the first /rmp_ee_pose sample.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture the current /rmp_ee_pose and write it into an experiment "
            "scenario.yaml absolute-goal step."
        )
    )
    parser.add_argument(
        "--scenario",
        required=True,
        help="Path to the experiment scenario.yaml file.",
    )
    parser.add_argument(
        "--ee-pose-topic",
        default="/rmp_ee_pose",
        help="Pose topic used as the capture source.",
    )
    parser.add_argument(
        "--step-index",
        type=int,
        default=0,
        help="Zero-based scenario step index to overwrite.",
    )
    parser.add_argument(
        "--wait-for-pose",
        type=float,
        default=10.0,
        help="Seconds to wait for the first pose sample.",
    )
    parser.add_argument(
        "--use-pose",
        action="store_true",
        help="Store orientation_xyzw and set use_pose=true for the step.",
    )
    return parser.parse_args()


def load_yaml_mapping(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    if not isinstance(document, dict):
        raise RuntimeError(f"expected a YAML mapping at the top level: {path}")
    return document


def dump_yaml_mapping(path: Path, document: Dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        yaml.dump(
            document,
            handle,
            Dumper=ScenarioYamlDumper,
            sort_keys=False,
            allow_unicode=False,
            default_flow_style=False,
        )


def validate_steps(document: Dict[str, Any], step_index: int) -> List[Dict[str, Any]]:
    steps = document.get("steps")
    if not isinstance(steps, list) or not steps:
        raise RuntimeError("scenario.yaml must contain a non-empty steps list")
    if step_index < 0 or step_index >= len(steps):
        raise RuntimeError(f"step-index {step_index} is out of range for {len(steps)} steps")
    for step in steps:
        if not isinstance(step, dict):
            raise RuntimeError("every scenario step must be a YAML mapping")
    return steps


def main() -> int:
    args = parse_args()
    scenario_path = Path(args.scenario).expanduser().resolve()
    if not scenario_path.exists():
        raise RuntimeError(f"scenario file does not exist: {scenario_path}")

    document = load_yaml_mapping(scenario_path)
    steps = validate_steps(document, args.step_index)

    rclpy.init()
    node = EePoseCaptureNode(args.ee_pose_topic)
    try:
        pose = node.wait_for_pose(args.wait_for_pose)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    step = steps[args.step_index]
    step["target"] = "absolute"
    step["position_xyz"] = [
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
    ]

    if args.use_pose:
        step["use_pose"] = True
        step["orientation_xyzw"] = [
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ]
    else:
        step["use_pose"] = False
        step.pop("orientation_xyzw", None)

    dump_yaml_mapping(scenario_path, document)

    print(f"Updated step {args.step_index} in: {scenario_path}")
    print(
        "position_xyz: "
        f"[{pose.position.x:.6f}, {pose.position.y:.6f}, {pose.position.z:.6f}]"
    )
    if args.use_pose:
        print(
            "orientation_xyzw: "
            f"[{pose.orientation.x:.6f}, {pose.orientation.y:.6f}, "
            f"{pose.orientation.z:.6f}, {pose.orientation.w:.6f}]"
        )
    print(f"use_pose: {args.use_pose}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
