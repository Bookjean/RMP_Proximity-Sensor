#!/usr/bin/env python3

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Callable, List, Sequence, Tuple

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Move the real RB10 to the saved start joint pose stored inside an "
            "experiment params.yaml file."
        )
    )
    parser.add_argument(
        "--params",
        required=True,
        help="Path to the experiment params.yaml file.",
    )
    parser.add_argument(
        "--joint-key",
        default="default_q",
        choices=("default_q", "initial_q"),
        help="Which rmpflow_controller joint list to use as the move_j target.",
    )
    parser.add_argument("--robot-ip", default="192.168.111.50")
    parser.add_argument(
        "--simulation-mode",
        action="store_true",
        help="Use RB10 simulation mode instead of real mode.",
    )
    parser.add_argument(
        "--movej-speed",
        type=float,
        default=20.0,
        help="MoveJ speed in degrees per second.",
    )
    parser.add_argument(
        "--movej-accel",
        type=float,
        default=20.0,
        help="MoveJ acceleration in degrees per second squared.",
    )
    parser.add_argument(
        "--settle-timeout",
        type=float,
        default=15.0,
        help="How long to wait for the saved start pose to settle.",
    )
    parser.add_argument(
        "--settle-tolerance-deg",
        type=float,
        default=0.3,
        help="Maximum allowed absolute target error in degrees for both measured and reference joints.",
    )
    return parser.parse_args()


def load_target_radians(params_path: Path, joint_key: str) -> List[float]:
    with params_path.open("r", encoding="utf-8") as handle:
        doc = yaml.safe_load(handle) or {}
    try:
        values = doc["rmpflow_controller"]["ros__parameters"][joint_key]
    except KeyError as exc:
        raise RuntimeError(
            f"params file does not contain rmpflow_controller.ros__parameters.{joint_key}"
        ) from exc
    if not isinstance(values, list) or len(values) != 6:
        raise RuntimeError(f"{joint_key} must contain exactly 6 joint values")
    return [float(value) for value in values]


def radians_to_degrees(values: Sequence[float]) -> List[float]:
    return [math.degrees(value) for value in values]


def wait_for_joint_snapshot(
    timeout_sec: float,
    measured_joint_fn: Callable[[], Sequence[float]],
    reference_joint_fn: Callable[[], Sequence[float]],
) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        measured = measured_joint_fn()
        reference = reference_joint_fn()
        if (
            measured and len(measured) >= 6 and
            reference and len(reference) >= 6
        ):
            measured_deg = tuple(float(value) for value in measured[:6])
            reference_deg = tuple(float(value) for value in reference[:6])
            return measured_deg, reference_deg
        time.sleep(0.05)
    raise RuntimeError("Timed out waiting for RB10 joint state")


def format_joint_line(values_deg: Sequence[float]) -> str:
    return "[" + ", ".join(f"{value:8.3f}" for value in values_deg) + "]"


def max_abs_target_error(target_deg: Sequence[float], actual_deg: Sequence[float]) -> float:
    return max(abs(target_deg[index] - actual_deg[index]) for index in range(6))


def main() -> int:
    args = parse_args()
    params_path = Path(args.params).expanduser().resolve()
    if not params_path.exists():
        raise RuntimeError(f"params file does not exist: {params_path}")

    target_rad = load_target_radians(params_path, args.joint_key)
    target_deg = radians_to_degrees(target_rad)

    from api.cobot import (  # noqa: E402
        CobotInit,
        DisConnectToCB,
        GetCurrentMeasuredSplitedJoint,
        GetCurrentSplitedJoint,
        MoveJ,
        PG_MODE,
        SetProgramMode,
        ToCB,
    )

    mode = PG_MODE.SIMULATION if args.simulation_mode else PG_MODE.REAL
    if not ToCB(args.robot_ip):
        raise RuntimeError(f"Failed to connect to RB10 at {args.robot_ip}")
    CobotInit()
    time.sleep(0.2)
    SetProgramMode(mode)
    time.sleep(0.2)

    try:
        measured_deg, reference_deg = wait_for_joint_snapshot(
            timeout_sec=3.0,
            measured_joint_fn=GetCurrentMeasuredSplitedJoint,
            reference_joint_fn=GetCurrentSplitedJoint,
        )
        print(f"target_deg   : {format_joint_line(target_deg)}")
        print(f"measured_deg : {format_joint_line(measured_deg)}")
        print(f"reference_deg: {format_joint_line(reference_deg)}")

        MoveJ(
            target_deg[0],
            target_deg[1],
            target_deg[2],
            target_deg[3],
            target_deg[4],
            target_deg[5],
            args.movej_speed,
            args.movej_accel,
        )

        deadline = time.time() + args.settle_timeout
        while time.time() < deadline:
            measured_deg, reference_deg = wait_for_joint_snapshot(
                timeout_sec=1.0,
                measured_joint_fn=GetCurrentMeasuredSplitedJoint,
                reference_joint_fn=GetCurrentSplitedJoint,
            )
            measured_error = max_abs_target_error(target_deg, measured_deg)
            reference_error = max_abs_target_error(target_deg, reference_deg)
            print(
                "settle_deg   : "
                f"measured={measured_error:.3f}, reference={reference_error:.3f}"
            )
            if (
                measured_error <= args.settle_tolerance_deg and
                reference_error <= args.settle_tolerance_deg
            ):
                print("Reached saved start pose.")
                return 0
            time.sleep(0.2)
    finally:
        DisConnectToCB()

    raise RuntimeError("Saved start pose did not settle within the timeout")


if __name__ == "__main__":
    raise SystemExit(main())
