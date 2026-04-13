#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt

JOINT_NAMES = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]


def load_log(path: Path):
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")
    if path.stat().st_size == 0:
        raise ValueError(
            f"Log file is empty: {path}. Re-run the logger and let it record for a few seconds before plotting."
        )

    times = []
    velocities = {name: [] for name in JOINT_NAMES}

    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError(
                f"Log file has no header row: {path}. Re-run the logger and let it record for a few seconds before plotting."
            )
        expected_columns = ["stamp_sec", *[f"{name}_velocity" for name in JOINT_NAMES]]
        missing = [column for column in expected_columns if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"Missing columns in {path}: {missing}")

        first_stamp = None
        for row in reader:
            stamp = float(row["stamp_sec"])
            if first_stamp is None:
                first_stamp = stamp
            times.append(stamp - first_stamp)
            for name in JOINT_NAMES:
                velocities[name].append(float(row[f"{name}_velocity"]))

    if not times:
        raise ValueError(f"No data rows found in {path}")
    return times, velocities


def resolve_input_path(input_path: str | None, latest: bool) -> Path:
    if input_path:
        path = Path(input_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"File not found: {path}")
        return path

    log_dir = Path("~/ros2_ws/data/joint_velocity_logs").expanduser()
    if not log_dir.exists():
        raise FileNotFoundError(f"Log directory does not exist: {log_dir}")

    candidates = sorted(log_dir.glob("*.txt"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"No txt logs found in {log_dir}")

    if latest:
        return candidates[-1]

    raise ValueError("Provide a log path or use --latest")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot joint velocity txt logs as 6 subplots")
    parser.add_argument("log_path", nargs="?", help="Path to the saved joint velocity txt file")
    parser.add_argument("--latest", action="store_true", help="Plot the newest txt file in ~/ros2_ws/data/joint_velocity_logs")
    parser.add_argument("--save", help="Optional output image path (e.g. plot.png)")
    args = parser.parse_args()

    log_path = resolve_input_path(args.log_path, args.latest)
    times, velocities = load_log(log_path)

    fig, axes = plt.subplots(6, 1, figsize=(12, 12), sharex=True)
    fig.suptitle(f"Joint velocities: {log_path}")

    for axis, joint_name in zip(axes, JOINT_NAMES):
        axis.plot(times, velocities[joint_name], linewidth=1.0)
        axis.set_ylabel(joint_name)
        axis.grid(True, alpha=0.3)

    axes[-1].set_xlabel("time [s]")
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.98))

    if args.save:
        output_path = Path(args.save).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=150)
        print(f"Saved plot to: {output_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
