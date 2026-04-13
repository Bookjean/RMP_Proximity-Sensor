#!/usr/bin/env python3

import argparse
import csv
import math
import os
from dataclasses import dataclass
from statistics import median
from typing import Dict, List, Sequence


JOINT_COUNT = 6
RAD_TO_DEG = 180.0 / math.pi


@dataclass
class JointMetrics:
    tracking_error_rms_deg: float
    command_to_reference_error_rms_deg: float
    reference_to_measured_error_rms_deg: float
    command_vel_rms_deg_s: float
    command_accel_rms_deg_s2: float
    command_flips_per_s: float
    state_vel_rms_deg_s: float
    state_accel_rms_deg_s2: float
    state_flips_per_s: float


@dataclass
class DatasetSummary:
    path: str
    rows: int
    median_dt_sec: float
    duration_sec: float
    active_duration_sec: float
    tracking_error_rms_deg_median: float
    tracking_error_rms_deg_max: float
    command_to_reference_error_rms_deg_median: float
    reference_to_measured_error_rms_deg_median: float
    command_flips_per_s_median: float
    command_flips_per_s_max: float
    state_flips_per_s_median: float
    state_flips_per_s_max: float
    command_accel_rms_deg_s2_median: float
    state_accel_rms_deg_s2_median: float
    ee_goal_error_rms_m: float
    ee_goal_final_error_m: float
    ee_goal_segment_final_error_median_m: float
    ee_goal_segment_final_error_max_m: float
    ee_goal_segment_final_errors_m: List[float]
    likely_dominant_source: str


def is_finite_number(text: str) -> bool:
    try:
        value = float(text)
    except (TypeError, ValueError):
        return False
    return math.isfinite(value)


def load_rows(path: str) -> List[Dict[str, float]]:
    raw_lines: List[str] = []
    with open(path, newline="") as handle:
        for line in handle:
            if line.startswith("#"):
                continue
            raw_lines.append(line)

    reader = csv.DictReader(raw_lines)
    rows: List[Dict[str, float]] = []
    for row in reader:
        parsed: Dict[str, float] = {}
        for key, value in row.items():
            if value is None or value == "":
                parsed[key] = float("nan")
            elif is_finite_number(value):
                parsed[key] = float(value)
            else:
                parsed[key] = float("nan")
        rows.append(parsed)
    return rows


def finite(values: Sequence[float]) -> List[float]:
    return [value for value in values if math.isfinite(value)]


def rms(values: Sequence[float]) -> float:
    clean = finite(values)
    if not clean:
        return float("nan")
    return math.sqrt(sum(value * value for value in clean) / len(clean))


def diff(values: Sequence[float], dt: float) -> List[float]:
    if dt <= 0.0 or len(values) < 2:
        return []
    out: List[float] = []
    for first, second in zip(values, values[1:]):
        if math.isfinite(first) and math.isfinite(second):
            out.append((second - first) / dt)
        else:
            out.append(float("nan"))
    return out


def sign_flips_per_second(values: Sequence[float], threshold: float, dt: float) -> float:
    if dt <= 0.0:
        return float("nan")
    previous_sign = 0
    flips = 0
    active_samples = 0
    for value in values:
        if not math.isfinite(value) or abs(value) < threshold:
            continue
        sign = 1 if value > 0.0 else -1
        active_samples += 1
        if previous_sign != 0 and sign != previous_sign:
            flips += 1
        previous_sign = sign
    duration = active_samples * dt
    if duration <= 0.0:
        return 0.0
    return flips / duration


def aggregate(values: Sequence[float]) -> Dict[str, float]:
    clean = finite(values)
    if not clean:
        return {"median": float("nan"), "max": float("nan")}
    return {"median": median(clean), "max": max(clean)}


def position_error_m(
    row: Dict[str, float],
    goal_keys: Sequence[str],
    pose_keys: Sequence[str],
) -> float:
    goal = [row.get(key, float("nan")) for key in goal_keys]
    pose = [row.get(key, float("nan")) for key in pose_keys]
    if not all(math.isfinite(value) for value in goal + pose):
        return float("nan")
    return math.sqrt(
        sum((pose[index] - goal[index]) * (pose[index] - goal[index]) for index in range(3))
    )


def goal_changed(
    previous_goal: Sequence[float],
    current_goal: Sequence[float],
    threshold_m: float = 1e-4,
) -> bool:
    if len(previous_goal) != 3 or len(current_goal) != 3:
        return True
    if not all(math.isfinite(value) for value in previous_goal + current_goal):
        return True
    return math.sqrt(
        sum(
            (current_goal[index] - previous_goal[index]) *
            (current_goal[index] - previous_goal[index])
            for index in range(3)
        )
    ) > threshold_m


def split_goal_segments(rows: Sequence[Dict[str, float]]) -> List[Sequence[Dict[str, float]]]:
    if not rows:
        return []

    segments: List[Sequence[Dict[str, float]]] = []
    start_index = 0
    previous_goal = [
        rows[0].get("goal_x", float("nan")),
        rows[0].get("goal_y", float("nan")),
        rows[0].get("goal_z", float("nan")),
    ]

    for index in range(1, len(rows)):
        current_goal = [
            rows[index].get("goal_x", float("nan")),
            rows[index].get("goal_y", float("nan")),
            rows[index].get("goal_z", float("nan")),
        ]
        if goal_changed(previous_goal, current_goal):
            segments.append(rows[start_index:index])
            start_index = index
        previous_goal = current_goal

    segments.append(rows[start_index:])
    return segments


def compute_ee_goal_metrics(rows: Sequence[Dict[str, float]], dt: float) -> Dict[str, object]:
    goal_keys = ("goal_x", "goal_y", "goal_z")
    pose_keys = ("ee_pose_x", "ee_pose_y", "ee_pose_z")
    errors = [
        position_error_m(row, goal_keys=goal_keys, pose_keys=pose_keys)
        for row in rows
    ]
    rms_error = rms(errors)

    final_error = float("nan")
    for value in reversed(errors):
        if math.isfinite(value):
            final_error = value
            break

    tail_samples = max(1, int(round(0.5 / max(dt, 1e-6))))
    segment_end_errors: List[float] = []
    for segment in split_goal_segments(rows):
        if not segment:
            continue
        tail_rows = segment[-tail_samples:]
        tail_errors = [
            position_error_m(row, goal_keys=goal_keys, pose_keys=pose_keys)
            for row in tail_rows
        ]
        clean_tail = finite(tail_errors)
        if clean_tail:
            segment_end_errors.append(median(clean_tail))
            continue

        fallback_errors = [
            position_error_m(row, goal_keys=goal_keys, pose_keys=pose_keys)
            for row in segment
        ]
        clean_fallback = finite(fallback_errors)
        if clean_fallback:
            segment_end_errors.append(clean_fallback[-1])

    aggregate_segment = aggregate(segment_end_errors)
    return {
        "rms_m": rms_error,
        "final_m": final_error,
        "segment_final_median_m": aggregate_segment["median"],
        "segment_final_max_m": aggregate_segment["max"],
        "segment_final_errors_m": segment_end_errors,
    }


def detect_active_slice(rows: Sequence[Dict[str, float]]) -> slice:
    if len(rows) < 5:
        return slice(0, len(rows))

    goal0 = [
        rows[0].get("goal_x", float("nan")),
        rows[0].get("goal_y", float("nan")),
        rows[0].get("goal_z", float("nan")),
    ]
    start = 0
    for index in range(1, len(rows)):
        goal = [
            rows[index].get("goal_x", float("nan")),
            rows[index].get("goal_y", float("nan")),
            rows[index].get("goal_z", float("nan")),
        ]
        goal_delta = math.sqrt(
            sum(
                (goal[i] - goal0[i]) * (goal[i] - goal0[i])
                for i in range(3)
                if math.isfinite(goal[i]) and math.isfinite(goal0[i])
            )
        )
        cmd_delta = 0.0
        for joint_index in range(JOINT_COUNT):
            prev_key = f"cmd_q{joint_index + 1}"
            first = rows[index - 1].get(prev_key, float("nan"))
            second = rows[index].get(prev_key, float("nan"))
            if math.isfinite(first) and math.isfinite(second):
                cmd_delta = max(cmd_delta, abs(second - first))
        if goal_delta > 0.005 or cmd_delta > 1e-4:
            start = max(0, index - 1)
            break

    return slice(start, len(rows))


def compute_joint_metrics(rows: Sequence[Dict[str, float]], dt: float) -> List[JointMetrics]:
    metrics: List[JointMetrics] = []
    for joint_index in range(JOINT_COUNT):
        q_key = f"q{joint_index + 1}"
        qd_key = f"qd{joint_index + 1}"
        cmd_key = f"cmd_q{joint_index + 1}"
        ref_key = f"ref_q{joint_index + 1}"
        meas_key = f"meas_q{joint_index + 1}"
        ref_minus_meas_key = f"ref_minus_meas_deg_q{joint_index + 1}"

        q = [row.get(q_key, float("nan")) for row in rows]
        qd = [row.get(qd_key, float("nan")) for row in rows]
        cmd = [row.get(cmd_key, float("nan")) for row in rows]
        ref_q = [row.get(ref_key, float("nan")) for row in rows]
        meas_q = [row.get(meas_key, float("nan")) for row in rows]
        ref_minus_meas_deg = [row.get(ref_minus_meas_key, float("nan")) for row in rows]

        tracking_error = [
            (cmd_value - q_value) * RAD_TO_DEG
            if math.isfinite(cmd_value) and math.isfinite(q_value)
            else float("nan")
            for cmd_value, q_value in zip(cmd, q)
        ]
        command_to_reference_error = [
            (cmd_value - ref_value) * RAD_TO_DEG
            if math.isfinite(cmd_value) and math.isfinite(ref_value)
            else float("nan")
            for cmd_value, ref_value in zip(cmd, ref_q)
        ]
        reference_to_measured_error = [
            value
            if math.isfinite(value)
            else (
                (ref_value - meas_value) * RAD_TO_DEG
                if math.isfinite(ref_value) and math.isfinite(meas_value)
                else float("nan")
            )
            for value, ref_value, meas_value in zip(ref_minus_meas_deg, ref_q, meas_q)
        ]
        cmd_vel = [value * RAD_TO_DEG for value in diff(cmd, dt)]
        cmd_accel = diff(cmd_vel, dt)
        qd_deg = [
            value * RAD_TO_DEG if math.isfinite(value) else float("nan")
            for value in qd
        ]
        qd_accel = diff(qd_deg, dt)

        metrics.append(
            JointMetrics(
                tracking_error_rms_deg=rms(tracking_error),
                command_to_reference_error_rms_deg=rms(command_to_reference_error),
                reference_to_measured_error_rms_deg=rms(reference_to_measured_error),
                command_vel_rms_deg_s=rms(cmd_vel),
                command_accel_rms_deg_s2=rms(cmd_accel),
                command_flips_per_s=sign_flips_per_second(cmd_vel, threshold=2.0, dt=dt),
                state_vel_rms_deg_s=rms(qd_deg),
                state_accel_rms_deg_s2=rms(qd_accel),
                state_flips_per_s=sign_flips_per_second(qd_deg, threshold=2.0, dt=dt),
            )
        )
    return metrics


def classify(metrics: Sequence[JointMetrics]) -> str:
    cmd_flips = aggregate([item.command_flips_per_s for item in metrics])["median"]
    state_flips = aggregate([item.state_flips_per_s for item in metrics])["median"]
    cmd_accel = aggregate([item.command_accel_rms_deg_s2 for item in metrics])["median"]
    state_accel = aggregate([item.state_accel_rms_deg_s2 for item in metrics])["median"]
    tracking_error = aggregate([item.tracking_error_rms_deg for item in metrics])["median"]
    cmd_to_ref_error = aggregate([item.command_to_reference_error_rms_deg for item in metrics])[
        "median"
    ]
    ref_to_meas_error = aggregate(
        [item.reference_to_measured_error_rms_deg for item in metrics]
    )["median"]

    if (
        math.isfinite(ref_to_meas_error)
        and ref_to_meas_error > 0.25
        and (
            not math.isfinite(cmd_to_ref_error)
            or cmd_to_ref_error < ref_to_meas_error * 0.7
        )
    ):
        return (
            "RB10 internal reference-to-measured tracking gap is larger than "
            "command-to-reference gap: downstream tracking/state-source mismatch is "
            "the most likely cause"
        )

    if math.isfinite(cmd_flips) and cmd_flips > 6.0:
        if (
            math.isfinite(state_accel)
            and math.isfinite(cmd_accel)
            and state_accel > cmd_accel * 1.5
        ):
            return (
                "upstream oscillation is visible, and state-velocity jitter is stronger "
                "than command jitter: velocity feedback/state estimation is the most "
                "likely cause"
            )
        return (
            "upstream oscillation is visible in /position_controllers/commands: "
            "RMP output aggression or feedback interaction is the most likely cause"
        )

    if (
        math.isfinite(tracking_error)
        and tracking_error > 0.8
        and math.isfinite(state_flips)
        and math.isfinite(cmd_flips)
        and state_flips > cmd_flips * 1.5
    ):
        return (
            "commands are relatively smooth but the robot state oscillates more: "
            "RB10 servo tracking/smoothing is the most likely cause"
        )

    if (
        math.isfinite(state_accel)
        and math.isfinite(cmd_accel)
        and state_accel > cmd_accel * 1.5
    ):
        return (
            "state-velocity jitter dominates command jitter: velocity feedback/state "
            "estimation is the most likely cause"
        )

    return (
        "mixed or inconclusive: both upstream command generation and downstream servo "
        "tracking may be contributing"
    )


def summarize_dataset(
    path: str,
    rows: Sequence[Dict[str, float]],
    active_rows: Sequence[Dict[str, float]],
    dt: float,
    metrics: Sequence[JointMetrics],
) -> DatasetSummary:
    duration = 0.0
    if len(rows) >= 2:
        duration = rows[-1]["timestamp_unix"] - rows[0]["timestamp_unix"]

    active_duration = 0.0
    if len(active_rows) >= 2:
        active_duration = (
            active_rows[-1]["timestamp_unix"] - active_rows[0]["timestamp_unix"]
        )

    tracking_error = aggregate([item.tracking_error_rms_deg for item in metrics])
    command_to_reference_error = aggregate(
        [item.command_to_reference_error_rms_deg for item in metrics]
    )
    reference_to_measured_error = aggregate(
        [item.reference_to_measured_error_rms_deg for item in metrics]
    )
    cmd_flips = aggregate([item.command_flips_per_s for item in metrics])
    state_flips = aggregate([item.state_flips_per_s for item in metrics])
    cmd_accel = aggregate([item.command_accel_rms_deg_s2 for item in metrics])
    state_accel = aggregate([item.state_accel_rms_deg_s2 for item in metrics])
    ee_goal = compute_ee_goal_metrics(active_rows, dt)

    return DatasetSummary(
        path=path,
        rows=len(rows),
        median_dt_sec=dt,
        duration_sec=duration,
        active_duration_sec=active_duration,
        tracking_error_rms_deg_median=tracking_error["median"],
        tracking_error_rms_deg_max=tracking_error["max"],
        command_to_reference_error_rms_deg_median=command_to_reference_error["median"],
        reference_to_measured_error_rms_deg_median=reference_to_measured_error["median"],
        command_flips_per_s_median=cmd_flips["median"],
        command_flips_per_s_max=cmd_flips["max"],
        state_flips_per_s_median=state_flips["median"],
        state_flips_per_s_max=state_flips["max"],
        command_accel_rms_deg_s2_median=cmd_accel["median"],
        state_accel_rms_deg_s2_median=state_accel["median"],
        ee_goal_error_rms_m=ee_goal["rms_m"],
        ee_goal_final_error_m=ee_goal["final_m"],
        ee_goal_segment_final_error_median_m=ee_goal["segment_final_median_m"],
        ee_goal_segment_final_error_max_m=ee_goal["segment_final_max_m"],
        ee_goal_segment_final_errors_m=list(ee_goal["segment_final_errors_m"]),
        likely_dominant_source=classify(metrics),
    )


def analyze_path(path: str) -> DatasetSummary:
    rows = load_rows(path)
    if len(rows) < 5:
        raise RuntimeError(f"not enough rows in dataset: {path}")

    times = finite([row.get("timestamp_unix", float("nan")) for row in rows])
    if len(times) < 2:
        raise RuntimeError(f"dataset does not contain valid timestamps: {path}")

    dt_values = [
        second - first
        for first, second in zip(times, times[1:])
        if second > first
    ]
    if not dt_values:
        raise RuntimeError(f"failed to compute sample dt from dataset: {path}")

    dt = median(dt_values)
    active_rows = rows[detect_active_slice(rows)]
    metrics = compute_joint_metrics(active_rows, dt)
    return summarize_dataset(path, rows, active_rows, dt, metrics)


def fmt(value: float, digits: int = 3) -> str:
    if not math.isfinite(value):
        return "nan"
    return f"{value:.{digits}f}"


def fmt_list(values: Sequence[float], digits: int = 4) -> str:
    clean = finite(values)
    if not clean:
        return "[]"
    return "[" + ", ".join(f"{value:.{digits}f}" for value in clean) + "]"


def percent_change(before: float, after: float) -> float:
    if not math.isfinite(before) or not math.isfinite(after) or abs(before) <= 1e-9:
        return float("nan")
    return (after - before) / abs(before) * 100.0


def reduction_fraction(before: float, after: float) -> float:
    if not math.isfinite(before) or not math.isfinite(after) or abs(before) <= 1e-9:
        return float("nan")
    return (before - after) / abs(before)


def print_summary(summary: DatasetSummary) -> None:
    print(f"dataset: {summary.path}")
    print(f"rows: {summary.rows}")
    print(f"median_dt_sec: {fmt(summary.median_dt_sec, 5)}")
    print(f"duration_sec: {fmt(summary.duration_sec)}")
    print(f"active_duration_sec: {fmt(summary.active_duration_sec)}")
    print(
        f"tracking_error_rms_deg_median: "
        f"{fmt(summary.tracking_error_rms_deg_median)}"
    )
    print(
        f"tracking_error_rms_deg_max: "
        f"{fmt(summary.tracking_error_rms_deg_max)}"
    )
    print(
        f"command_to_reference_error_rms_deg_median: "
        f"{fmt(summary.command_to_reference_error_rms_deg_median)}"
    )
    print(
        f"reference_to_measured_error_rms_deg_median: "
        f"{fmt(summary.reference_to_measured_error_rms_deg_median)}"
    )
    print(
        f"command_flips_per_s_median: "
        f"{fmt(summary.command_flips_per_s_median)}"
    )
    print(f"command_flips_per_s_max: {fmt(summary.command_flips_per_s_max)}")
    print(f"state_flips_per_s_median: {fmt(summary.state_flips_per_s_median)}")
    print(f"state_flips_per_s_max: {fmt(summary.state_flips_per_s_max)}")
    print(
        f"command_accel_rms_deg_s2_median: "
        f"{fmt(summary.command_accel_rms_deg_s2_median)}"
    )
    print(
        f"state_accel_rms_deg_s2_median: "
        f"{fmt(summary.state_accel_rms_deg_s2_median)}"
    )
    print(f"ee_goal_error_rms_m: {fmt(summary.ee_goal_error_rms_m, 4)}")
    print(f"ee_goal_final_error_m: {fmt(summary.ee_goal_final_error_m, 4)}")
    print(
        f"ee_goal_segment_final_error_median_m: "
        f"{fmt(summary.ee_goal_segment_final_error_median_m, 4)}"
    )
    print(
        f"ee_goal_segment_final_error_max_m: "
        f"{fmt(summary.ee_goal_segment_final_error_max_m, 4)}"
    )
    print(
        f"ee_goal_segment_final_errors_m: "
        f"{fmt_list(summary.ee_goal_segment_final_errors_m, 4)}"
    )
    print(f"likely_dominant_source: {summary.likely_dominant_source}")


def metric_line(
    label: str,
    baseline_value: float,
    variant_value: float,
    lower_is_better: bool = True,
) -> str:
    delta_pct = percent_change(baseline_value, variant_value)
    direction = "same"
    if math.isfinite(delta_pct):
        if abs(delta_pct) < 5.0:
            direction = "same"
        elif lower_is_better and delta_pct < 0.0:
            direction = "improved"
        elif lower_is_better and delta_pct > 0.0:
            direction = "worse"
        elif not lower_is_better and delta_pct > 0.0:
            direction = "improved"
        else:
            direction = "worse"
    return (
        f"{label}: {fmt(baseline_value)} -> {fmt(variant_value)} "
        f"({fmt(delta_pct)}%, {direction})"
    )


def compare_velocity_experiment(
    baseline: DatasetSummary,
    variant: DatasetSummary,
    baseline_label: str,
    variant_label: str,
) -> str:
    state_flips_reduction = reduction_fraction(
        baseline.state_flips_per_s_median, variant.state_flips_per_s_median
    )
    state_accel_reduction = reduction_fraction(
        baseline.state_accel_rms_deg_s2_median,
        variant.state_accel_rms_deg_s2_median,
    )
    cmd_flips_reduction = reduction_fraction(
        baseline.command_flips_per_s_median, variant.command_flips_per_s_median
    )
    cmd_accel_reduction = reduction_fraction(
        baseline.command_accel_rms_deg_s2_median,
        variant.command_accel_rms_deg_s2_median,
    )
    tracking_error_change = percent_change(
        baseline.tracking_error_rms_deg_median,
        variant.tracking_error_rms_deg_median,
    )
    ref_to_meas_change = percent_change(
        baseline.reference_to_measured_error_rms_deg_median,
        variant.reference_to_measured_error_rms_deg_median,
    )

    strong_reduction_count = sum(
        1
        for value in (
            state_flips_reduction,
            state_accel_reduction,
            cmd_flips_reduction,
            cmd_accel_reduction,
        )
        if math.isfinite(value) and value >= 0.25
    )
    mild_reduction_count = sum(
        1
        for value in (
            state_flips_reduction,
            state_accel_reduction,
            cmd_flips_reduction,
            cmd_accel_reduction,
        )
        if math.isfinite(value) and value >= 0.10
    )

    tracking_penalty_small = (
        not math.isfinite(tracking_error_change) or tracking_error_change <= 35.0
    )
    tracking_penalty_large = (
        math.isfinite(tracking_error_change)
        and tracking_error_change > 80.0
        and variant.tracking_error_rms_deg_median
        > baseline.tracking_error_rms_deg_median + 0.20
    )
    servo_gap_worse = (
        math.isfinite(ref_to_meas_change)
        and ref_to_meas_change > 50.0
        and variant.reference_to_measured_error_rms_deg_median
        > baseline.reference_to_measured_error_rms_deg_median + 0.15
    )

    if strong_reduction_count >= 2 and tracking_penalty_small and not servo_gap_worse:
        return (
            f"{variant_label} reduced oscillation metrics enough relative to "
            f"{baseline_label} that the velocity feedback path is very likely a "
            "dominant contributor to the shake."
        )

    if mild_reduction_count >= 2 and not tracking_penalty_large:
        return (
            f"{variant_label} improved multiple jitter metrics relative to "
            f"{baseline_label}, so the velocity feedback path is likely contributing, "
            "but it does not look like the only cause."
        )

    if tracking_penalty_large and mild_reduction_count >= 1:
        return (
            f"{variant_label} calmed some oscillation metrics but paid for it with a "
            "clear tracking penalty, so the velocity path is involved but the full "
            "problem likely also includes aggressive gains or downstream servo lag."
        )

    if strong_reduction_count == 0 and mild_reduction_count <= 1:
        return (
            f"{variant_label} did not materially improve the main oscillation metrics "
            f"relative to {baseline_label}, so the velocity feedback path does not look "
            "like the dominant cause from this pair of runs."
        )

    return (
        f"The {baseline_label} vs {variant_label} comparison is mixed. The velocity "
        "path may be contributing, but the dataset alone does not isolate it cleanly."
    )


def print_comparison(
    baseline: DatasetSummary,
    variant: DatasetSummary,
    baseline_label: str,
    variant_label: str,
) -> None:
    print(f"comparison: {baseline_label} -> {variant_label}")
    print(
        metric_line(
            "tracking_error_rms_deg_median",
            baseline.tracking_error_rms_deg_median,
            variant.tracking_error_rms_deg_median,
        )
    )
    print(
        metric_line(
            "reference_to_measured_error_rms_deg_median",
            baseline.reference_to_measured_error_rms_deg_median,
            variant.reference_to_measured_error_rms_deg_median,
        )
    )
    print(
        metric_line(
            "command_flips_per_s_median",
            baseline.command_flips_per_s_median,
            variant.command_flips_per_s_median,
        )
    )
    print(
        metric_line(
            "state_flips_per_s_median",
            baseline.state_flips_per_s_median,
            variant.state_flips_per_s_median,
        )
    )
    print(
        metric_line(
            "command_accel_rms_deg_s2_median",
            baseline.command_accel_rms_deg_s2_median,
            variant.command_accel_rms_deg_s2_median,
        )
    )
    print(
        metric_line(
            "state_accel_rms_deg_s2_median",
            baseline.state_accel_rms_deg_s2_median,
            variant.state_accel_rms_deg_s2_median,
        )
    )
    print(
        metric_line(
            "ee_goal_segment_final_error_median_m",
            baseline.ee_goal_segment_final_error_median_m,
            variant.ee_goal_segment_final_error_median_m,
        )
    )
    print(
        metric_line(
            "ee_goal_final_error_m",
            baseline.ee_goal_final_error_m,
            variant.ee_goal_final_error_m,
        )
    )
    print(
        "experiment_conclusion: "
        + compare_velocity_experiment(
            baseline,
            variant,
            baseline_label=baseline_label,
            variant_label=variant_label,
        )
    )


def latest_csv_in_directory(directory: str) -> str:
    candidates = [
        os.path.join(directory, entry)
        for entry in os.listdir(directory)
        if entry.endswith(".csv")
    ]
    if not candidates:
        raise FileNotFoundError(f"no csv files found in {directory}")
    return max(candidates, key=os.path.getmtime)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Analyze an RMP recorder CSV and estimate whether oscillation is coming "
            "from upstream RMP/feedback or downstream servo tracking. With "
            "--compare-other, also compare a second run against the primary dataset."
        )
    )
    parser.add_argument(
        "path",
        nargs="?",
        help="Recorder CSV path. If omitted, --latest-dir is used.",
    )
    parser.add_argument(
        "--latest-dir",
        help="Directory containing recorder CSV files. The newest CSV is analyzed.",
    )
    parser.add_argument(
        "--compare-other",
        help="Second recorder CSV path to compare against the primary dataset.",
    )
    parser.add_argument(
        "--baseline-label",
        default="baseline",
        help="Label for the primary dataset when using --compare-other.",
    )
    parser.add_argument(
        "--variant-label",
        default="variant",
        help="Label for the comparison dataset when using --compare-other.",
    )
    args = parser.parse_args()

    if args.path:
        path = os.path.expanduser(args.path)
    elif args.latest_dir:
        path = latest_csv_in_directory(os.path.expanduser(args.latest_dir))
    else:
        parser.error("provide either a csv path or --latest-dir")
        return 2

    primary_summary = analyze_path(path)
    print_summary(primary_summary)

    if args.compare_other:
        compare_path = os.path.expanduser(args.compare_other)
        compare_summary = analyze_path(compare_path)
        print()
        print_summary(compare_summary)
        print()
        print_comparison(
            primary_summary,
            compare_summary,
            baseline_label=args.baseline_label,
            variant_label=args.variant_label,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
