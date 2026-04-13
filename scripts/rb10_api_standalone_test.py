#!/usr/bin/env python3
import argparse
import os
import signal
import sys
import time


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from api.cobot import (  # noqa: E402
    CobotInit,
    CMD_TYPE,
    DisConnectToCB,
    GetCurrentMeasuredSplitedJoint,
    GetCurrentSplitedJoint,
    IsCommandSockConnect,
    IsDataSockConnect,
    MoveITPL_Clear,
    MoveJB_Clear,
    MoveJ,
    MovePB_Clear,
    MotionHalt,
    PG_MODE,
    SendCOMMAND,
    SetProgramMode,
    ToCB,
)


STOP_REQUESTED = False


def _handle_signal(signum, frame):
    del signum, frame
    global STOP_REQUESTED
    STOP_REQUESTED = True


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Standalone RB10 vendor API test tool. "
            "Use this to separate robot/API behavior from ROS/RMP behavior."
        )
    )
    parser.add_argument("--robot-ip", default="192.168.111.50")
    parser.add_argument(
        "--mode",
        choices=("status", "movej_step", "servo_hold"),
        default="status",
        help="status: print measured/reference joints only. movej_step: tiny move_j test. servo_hold: repeated move_servo_j test.",
    )
    parser.add_argument(
        "--simulation-mode",
        action="store_true",
        help="Use RB10 simulation mode instead of real mode.",
    )
    parser.add_argument(
        "--status-samples",
        type=int,
        default=20,
        help="Number of status lines to print in status mode.",
    )
    parser.add_argument(
        "--status-period",
        type=float,
        default=0.1,
        help="Status print period in seconds.",
    )
    parser.add_argument(
        "--joint-index",
        type=int,
        default=0,
        choices=range(6),
        metavar="[0-5]",
        help="Joint index for movej_step or servo_hold offset tests.",
    )
    parser.add_argument(
        "--delta-deg",
        type=float,
        default=0.3,
        help="Small joint offset in degrees for movej_step or servo_hold.",
    )
    parser.add_argument(
        "--movej-speed",
        type=float,
        default=3.0,
        help="move_j speed for movej_step.",
    )
    parser.add_argument(
        "--movej-accel",
        type=float,
        default=3.0,
        help="move_j accel for movej_step.",
    )
    parser.add_argument(
        "--servo-cycles",
        type=int,
        default=150,
        help="Number of repeated servo_j commands in servo_hold mode.",
    )
    parser.add_argument(
        "--servo-period",
        type=float,
        default=0.02,
        help="Delay between servo_j commands in seconds.",
    )
    parser.add_argument("--servo-t1", type=float, default=0.002)
    parser.add_argument("--servo-t2", type=float, default=0.1)
    parser.add_argument("--servo-gain", type=float, default=0.02)
    parser.add_argument("--servo-alpha", type=float, default=0.2)
    parser.add_argument(
        "--settle-timeout",
        type=float,
        default=10.0,
        help="How long to wait for move_j to settle.",
    )
    parser.add_argument(
        "--settle-tolerance-deg",
        type=float,
        default=0.2,
        help="Tolerance for move_j settle detection.",
    )
    parser.add_argument(
        "--return-to-start",
        action="store_true",
        help="After movej_step, command the starting pose again.",
    )
    return parser.parse_args()


def connect_robot(robot_ip, simulation_mode):
    mode = PG_MODE.SIMULATION if simulation_mode else PG_MODE.REAL
    if not ToCB(robot_ip):
        raise RuntimeError(f"Failed to connect to RB10 at {robot_ip}")
    CobotInit()
    time.sleep(0.2)
    SetProgramMode(mode)
    time.sleep(0.2)


def cleanup():
    try:
        MotionHalt()
    except Exception:
        pass
    for clear_fn in (MoveITPL_Clear, MovePB_Clear, MoveJB_Clear):
        try:
            clear_fn()
        except Exception:
            pass
    try:
        DisConnectToCB()
    except Exception:
        pass


def wait_for_joint_snapshot(timeout_sec):
    deadline = time.time() + timeout_sec
    while time.time() < deadline and not STOP_REQUESTED:
        measured = GetCurrentMeasuredSplitedJoint()
        reference = GetCurrentSplitedJoint()
        if (
            IsDataSockConnect() and
            IsCommandSockConnect() and
            measured and len(measured) >= 6 and
            reference and len(reference) >= 6 and
            not any(v is None for v in measured[:6]) and
            not any(v is None for v in reference[:6])
        ):
            return tuple(float(v) for v in measured[:6]), tuple(float(v) for v in reference[:6])
        time.sleep(0.05)
    raise RuntimeError("Timed out waiting for the initial RB10 joint state")


def print_joint_line(prefix, measured_deg, reference_deg):
    error_deg = [reference_deg[i] - measured_deg[i] for i in range(6)]
    measured_text = ", ".join(f"{value:8.3f}" for value in measured_deg)
    reference_text = ", ".join(f"{value:8.3f}" for value in reference_deg)
    error_text = ", ".join(f"{value:7.3f}" for value in error_deg)
    print(f"{prefix} measured : [{measured_text}]")
    print(f"{prefix} reference: [{reference_text}]")
    print(f"{prefix} error    : [{error_text}]")


def run_status_mode(args):
    for sample_index in range(args.status_samples):
        measured_deg, reference_deg = wait_for_joint_snapshot(timeout_sec=2.0)
        print_joint_line(f"[{sample_index:03d}]", measured_deg, reference_deg)
        if sample_index + 1 < args.status_samples:
            time.sleep(args.status_period)


def wait_until_settled(target_deg, timeout_sec, tolerance_deg):
    deadline = time.time() + timeout_sec
    while time.time() < deadline and not STOP_REQUESTED:
        measured_deg, reference_deg = wait_for_joint_snapshot(timeout_sec=1.0)
        max_measured_error = max(abs(target_deg[i] - measured_deg[i]) for i in range(6))
        max_reference_error = max(abs(target_deg[i] - reference_deg[i]) for i in range(6))
        print_joint_line("[settle]", measured_deg, reference_deg)
        print(
            f"[settle] max |target-measured|={max_measured_error:.3f} deg, "
            f"max |target-reference|={max_reference_error:.3f} deg"
        )
        if max_measured_error <= tolerance_deg and max_reference_error <= tolerance_deg:
            return True
        time.sleep(0.2)
    return False


def run_movej_step_mode(args):
    start_measured_deg, start_reference_deg = wait_for_joint_snapshot(timeout_sec=2.0)
    print_joint_line("[start]", start_measured_deg, start_reference_deg)

    target_deg = list(start_measured_deg)
    target_deg[args.joint_index] += args.delta_deg
    print(
        f"[movej] joint {args.joint_index} offset {args.delta_deg:+.3f} deg, "
        f"speed={args.movej_speed:.3f}, accel={args.movej_accel:.3f}"
    )
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

    settled = wait_until_settled(target_deg, args.settle_timeout, args.settle_tolerance_deg)
    if not settled:
        raise RuntimeError("move_j target did not settle within the timeout")

    if not args.return_to_start or STOP_REQUESTED:
        return

    print("[movej] returning to the starting pose")
    MoveJ(
        start_measured_deg[0],
        start_measured_deg[1],
        start_measured_deg[2],
        start_measured_deg[3],
        start_measured_deg[4],
        start_measured_deg[5],
        args.movej_speed,
        args.movej_accel,
    )
    settled = wait_until_settled(
        list(start_measured_deg), args.settle_timeout, args.settle_tolerance_deg
    )
    if not settled:
        raise RuntimeError("Return move_j target did not settle within the timeout")


def run_servo_hold_mode(args):
    measured_deg, reference_deg = wait_for_joint_snapshot(timeout_sec=2.0)
    print_joint_line("[start]", measured_deg, reference_deg)

    target_deg = list(measured_deg)
    target_deg[args.joint_index] += args.delta_deg
    print(
        f"[servo_hold] sending {args.servo_cycles} move_servo_j commands at {1.0 / max(args.servo_period, 1e-6):.1f} Hz "
        f"toward joint {args.joint_index} offset {args.delta_deg:+.3f} deg"
    )

    for cycle in range(args.servo_cycles):
        if STOP_REQUESTED:
            break
        command = (
            "move_servo_j(jnt[" +
            ",".join(f"{value:.3f}" for value in target_deg) +
            f"],{args.servo_t1:.6f},{args.servo_t2:.6f},{args.servo_gain:.6f},{args.servo_alpha:.6f})"
        )
        SendCOMMAND(command, CMD_TYPE.MOVE)
        if cycle % 20 == 0 or cycle + 1 == args.servo_cycles:
            measured_deg, reference_deg = wait_for_joint_snapshot(timeout_sec=1.0)
            print_joint_line(f"[servo {cycle + 1:03d}]", measured_deg, reference_deg)
        time.sleep(args.servo_period)


def main():
    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)
    args = parse_args()

    try:
        connect_robot(args.robot_ip, args.simulation_mode)
        if args.mode == "status":
            run_status_mode(args)
        elif args.mode == "movej_step":
            run_movej_step_mode(args)
        elif args.mode == "servo_hold":
            run_servo_hold_mode(args)
        else:
            raise RuntimeError(f"Unsupported mode: {args.mode}")
        return 0
    except Exception as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 1
    finally:
        cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
