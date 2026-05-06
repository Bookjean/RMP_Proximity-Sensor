#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


DEFAULT_DATA_DIR = Path("~/ros2_ws/data/rmp_datasets").expanduser()
EE_COLUMNS = ("ee_pose_x", "ee_pose_y", "ee_pose_z")
GOAL_COLUMNS = ("goal_pose_x", "goal_pose_y", "goal_pose_z")
COLOR_CYCLE = (
    "#1f77b4",
    "#d62728",
    "#2ca02c",
    "#ff7f0e",
    "#9467bd",
    "#8c564b",
    "#17becf",
    "#7f7f7f",
)


def is_finite_number(text: object) -> bool:
    try:
        value = float(text)
    except (TypeError, ValueError):
        return False
    return math.isfinite(value)


def parse_float(row: Dict[str, str], key: str) -> float:
    value = row.get(key, "")
    if not is_finite_number(value):
        return float("nan")
    return float(value)


def latest_csv_in_directory(directory: Path) -> Path:
    if not directory.exists():
        raise FileNotFoundError(f"Dataset directory does not exist: {directory}")

    candidates = sorted(directory.glob("*.csv"), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"No CSV files found in: {directory}")
    return candidates[-1]


def resolve_input_paths(csv_paths: Sequence[str], latest_dir: str) -> List[Path]:
    if csv_paths:
        resolved_paths = []
        for csv_path in csv_paths:
            path = Path(csv_path).expanduser()
            if not path.exists():
                raise FileNotFoundError(f"CSV file not found: {path}")
            resolved_paths.append(path)
        return resolved_paths
    return [latest_csv_in_directory(Path(latest_dir).expanduser())]


def make_labels(paths: Sequence[Path], labels: Optional[Sequence[str]]) -> List[str]:
    if labels:
        return [str(label) for label in labels]

    seen: Dict[str, int] = {}
    output = []
    for path in paths:
        label = path.stem
        count = seen.get(label, 0)
        seen[label] = count + 1
        if count:
            label = f"{label}_{count + 1}"
        output.append(label)
    return output


def default_output_path(csv_paths: Sequence[Path]) -> Path:
    first_path = csv_paths[0]
    if len(csv_paths) == 1:
        return first_path.with_name(f"{first_path.stem}_ee_trajectory_3d.png")
    return first_path.with_name(f"{first_path.stem}_compare_{len(csv_paths)}_ee_trajectory_3d.png")


def append_goal_points(
    goals: Sequence[Tuple[float, float, float]],
    xs: List[float],
    ys: List[float],
    zs: List[float],
) -> None:
    for goal in goals:
        xs.append(goal[0])
        ys.append(goal[1])
        zs.append(goal[2])


def validate_labels(labels: Optional[Sequence[str]], path_count: int) -> None:
    if labels and len(labels) != path_count:
        raise ValueError(f"--labels count ({len(labels)}) must match CSV count ({path_count}).")


def print_input_paths(paths: Sequence[Path]) -> None:
    for index, path in enumerate(paths, start=1):
        print(f"Input CSV {index}: {path}")


def resolve_input_path(csv_path: Optional[str], latest_dir: str) -> Path:
    if csv_path:
        path = Path(csv_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"CSV file not found: {path}")
        return path
    return latest_csv_in_directory(Path(latest_dir).expanduser())


def read_csv_rows(path: Path) -> Tuple[List[str], List[Dict[str, str]]]:
    raw_lines: List[str] = []
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        for line in handle:
            if line.startswith("#"):
                continue
            raw_lines.append(line)

    reader = csv.DictReader(raw_lines)
    if not reader.fieldnames:
        raise ValueError(f"CSV file has no header row: {path}")

    rows = [row for row in reader]
    if not rows:
        raise ValueError(f"CSV file has no data rows: {path}")
    return list(reader.fieldnames), rows


def require_columns(fieldnames: Sequence[str], columns: Sequence[str], path: Path) -> None:
    missing = [column for column in columns if column not in fieldnames]
    if missing:
        raise ValueError(f"Missing required columns in {path}: {missing}")


def load_ee_trajectory(
    path: Path,
    stride: int,
) -> Tuple[List[float], List[float], List[float], List[float], List[Tuple[float, float, float]]]:
    fieldnames, rows = read_csv_rows(path)
    require_columns(fieldnames, EE_COLUMNS, path)

    has_time = "timestamp_unix" in fieldnames
    has_goal = all(column in fieldnames for column in GOAL_COLUMNS)
    first_time: Optional[float] = None

    times: List[float] = []
    xs: List[float] = []
    ys: List[float] = []
    zs: List[float] = []
    goals: List[Tuple[float, float, float]] = []

    stride = max(stride, 1)
    for index, row in enumerate(rows):
        if index % stride != 0:
            continue

        point = [parse_float(row, key) for key in EE_COLUMNS]
        if not all(math.isfinite(value) for value in point):
            continue

        if has_time:
            stamp = parse_float(row, "timestamp_unix")
            if not math.isfinite(stamp):
                stamp = float(index)
            if first_time is None:
                first_time = stamp
            relative_time = stamp - first_time
        else:
            relative_time = float(index)

        times.append(relative_time)
        xs.append(point[0])
        ys.append(point[1])
        zs.append(point[2])

        if has_goal:
            goal = tuple(parse_float(row, key) for key in GOAL_COLUMNS)
            if all(math.isfinite(value) for value in goal):
                if not goals or distance_3d(goal, goals[-1]) > 1e-4:
                    goals.append(goal)

    if not xs:
        raise ValueError(f"No finite EE pose samples found in: {path}")
    return times, xs, ys, zs, goals


def distance_3d(first: Sequence[float], second: Sequence[float]) -> float:
    return math.sqrt(sum((float(first[i]) - float(second[i])) ** 2 for i in range(3)))


def set_axes_equal(ax, xs: Sequence[float], ys: Sequence[float], zs: Sequence[float]) -> None:
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    z_min, z_max = min(zs), max(zs)

    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min, 1e-3)
    half = max_range * 0.5
    x_mid = (x_min + x_max) * 0.5
    y_mid = (y_min + y_max) * 0.5
    z_mid = (z_min + z_max) * 0.5

    ax.set_xlim(x_mid - half, x_mid + half)
    ax.set_ylim(y_mid - half, y_mid + half)
    ax.set_zlim(z_mid - half, z_mid + half)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Plot 3D end-effector position trajectory from an RMP dataset CSV."
    )
    parser.add_argument(
        "csv_paths",
        nargs="*",
        help=(
            "Recorder CSV path(s). If omitted, the newest CSV in "
            "~/ros2_ws/data/rmp_datasets is used."
        ),
    )
    parser.add_argument(
        "--latest-dir",
        default=str(DEFAULT_DATA_DIR),
        help="Directory used when csv_path is omitted.",
    )
    parser.add_argument(
        "--save",
        help="Output image path. Default: <csv_stem>_ee_trajectory_3d.png next to the CSV.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show an interactive matplotlib window after plotting.",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Plot every Nth sample to reduce dense plots.",
    )
    parser.add_argument(
        "--no-goal",
        action="store_true",
        help="Do not draw goal_pose_x/y/z markers even when they exist in the CSV.",
    )
    parser.add_argument(
        "--title",
        help="Optional plot title.",
    )
    parser.add_argument(
        "--labels",
        nargs="+",
        help="Optional labels for the CSV paths, in the same order.",
    )
    args = parser.parse_args()

    if args.stride <= 0:
        parser.error("--stride must be greater than 0.")

    if not args.show:
        import matplotlib

        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    csv_paths = resolve_input_paths(args.csv_paths, args.latest_dir)
    try:
        validate_labels(args.labels, len(csv_paths))
    except ValueError as exc:
        parser.error(str(exc))
    labels = make_labels(csv_paths, args.labels)

    output_path = Path(args.save).expanduser() if args.save else default_output_path(csv_paths)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    all_xs: List[float] = []
    all_ys: List[float] = []
    all_zs: List[float] = []
    single_plot_points = None
    total_samples = 0
    total_goals = 0

    for index, (csv_path, label) in enumerate(zip(csv_paths, labels)):
        times, xs, ys, zs, goals = load_ee_trajectory(csv_path, args.stride)
        color = COLOR_CYCLE[index % len(COLOR_CYCLE)]
        total_samples += len(xs)

        all_xs.extend(xs)
        all_ys.extend(ys)
        all_zs.extend(zs)

        if len(csv_paths) == 1:
            ax.plot(xs, ys, zs, color=color, linewidth=1.5, label="EE trajectory")
            single_plot_points = ax.scatter(xs, ys, zs, c=times, cmap="viridis", s=10, alpha=0.75)
            ax.scatter(xs[0], ys[0], zs[0], color="#2ca02c", s=70, marker="o", label="start")
            ax.scatter(xs[-1], ys[-1], zs[-1], color="#d62728", s=70, marker="X", label="end")
        else:
            ax.plot(xs, ys, zs, color=color, linewidth=1.5, label=label)
            ax.scatter(xs, ys, zs, color=color, s=8, alpha=0.45)
            ax.scatter(xs[0], ys[0], zs[0], color=color, s=70, marker="o", edgecolors="black")
            ax.scatter(xs[-1], ys[-1], zs[-1], color=color, s=70, marker="X", edgecolors="black")

        if goals and not args.no_goal:
            goal_x = [goal[0] for goal in goals]
            goal_y = [goal[1] for goal in goals]
            goal_z = [goal[2] for goal in goals]
            goal_label = "goal" if len(csv_paths) == 1 else f"{label} goal"
            goal_color = "black" if len(csv_paths) == 1 else color
            ax.scatter(
                goal_x,
                goal_y,
                goal_z,
                color=goal_color,
                s=60,
                marker="*",
                edgecolors="black",
                label=goal_label,
            )
            append_goal_points(goals, all_xs, all_ys, all_zs)
            total_goals += len(goals)

    set_axes_equal(ax, all_xs, all_ys, all_zs)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    if args.title:
        title = args.title
    elif len(csv_paths) == 1:
        title = f"EE trajectory: {csv_paths[0].name}"
    else:
        title = f"EE trajectory comparison ({len(csv_paths)} CSV files)"
    ax.set_title(title)
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    if single_plot_points is not None:
        fig.colorbar(single_plot_points, ax=ax, shrink=0.7, pad=0.1, label="time [s]")
    fig.tight_layout()

    fig.savefig(output_path, dpi=180)
    print_input_paths(csv_paths)
    print(f"Samples plotted: {total_samples}")
    if total_goals and not args.no_goal:
        print(f"Goal markers: {total_goals}")
    print(f"Saved plot: {output_path}")

    if args.show:
        plt.show()
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
