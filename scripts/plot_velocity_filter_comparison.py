#!/usr/bin/env python3
import argparse
import csv
import html
import math
import os
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


JOINT_COUNT = 6
JOINT_NAMES = [f"q{i}" for i in range(1, JOINT_COUNT + 1)]


def parse_float(value: str, default: float = 0.0) -> float:
    try:
        if value == "":
            return default
        parsed = float(value)
        if not math.isfinite(parsed):
            return default
        return parsed
    except (TypeError, ValueError):
        return default


def read_dataset(path: Path) -> Tuple[List[float], List[List[float]]]:
    with path.open("r", newline="") as handle:
        lines = [line for line in handle if line.strip() and not line.startswith("#")]
    if not lines:
        raise RuntimeError(f"Dataset has no CSV header/data rows: {path}")

    reader = csv.DictReader(lines)
    if not reader.fieldnames:
        raise RuntimeError(f"Dataset has no CSV header: {path}")
    missing = [name for name in JOINT_NAMES if name not in reader.fieldnames]
    if missing:
        raise RuntimeError(f"Dataset is missing joint columns {missing}: {path}")
    if "timestamp_unix" not in reader.fieldnames:
        raise RuntimeError(f"Dataset is missing timestamp_unix column: {path}")

    times: List[float] = []
    positions: List[List[float]] = []
    for row in reader:
        stamp = parse_float(row.get("timestamp_unix", ""))
        q = [parse_float(row.get(name, "")) for name in JOINT_NAMES]
        if times and stamp <= times[-1]:
            continue
        times.append(stamp)
        positions.append(q)

    if len(times) < 2:
        raise RuntimeError(f"Need at least two timestamped samples: {path}")

    t0 = times[0]
    return [stamp - t0 for stamp in times], positions


def make_control_samples(
    sample_times: Sequence[float],
    positions: Sequence[Sequence[float]],
    control_rate_hz: float,
) -> Tuple[List[float], List[List[float]]]:
    duration = sample_times[-1]
    control_dt = 1.0 / control_rate_hz
    ticks: List[float] = []
    control_positions: List[List[float]] = []
    sample_index = 0
    tick = 0.0
    while tick <= duration + 1e-9:
        while sample_index + 1 < len(sample_times) and sample_times[sample_index + 1] <= tick:
            sample_index += 1
        ticks.append(tick)
        control_positions.append(list(positions[sample_index]))
        tick += control_dt
    return ticks, control_positions


def current_controller_velocity(
    control_positions: Sequence[Sequence[float]],
    control_rate_hz: float,
    alpha: float,
) -> List[List[float]]:
    control_dt = 1.0 / control_rate_hz
    estimate = [0.0] * JOINT_COUNT
    previous_q = None
    output: List[List[float]] = []
    for q in control_positions:
        if previous_q is None:
            estimate = [0.0] * JOINT_COUNT
        else:
            raw = [(q[i] - previous_q[i]) / control_dt for i in range(JOINT_COUNT)]
            estimate = [
                alpha * raw[i] + (1.0 - alpha) * estimate[i]
                for i in range(JOINT_COUNT)
            ]
        previous_q = list(q)
        output.append(list(estimate))
    return output


def alpha_beta_velocity(
    sample_times: Sequence[float],
    positions: Sequence[Sequence[float]],
    control_ticks: Sequence[float],
    alpha: float,
    beta: float,
    reset_gap_sec: float,
) -> List[List[float]]:
    q_hat: List[float] = [0.0] * JOINT_COUNT
    v_hat: List[float] = [0.0] * JOINT_COUNT
    initialized = False
    previous_time = 0.0
    sample_index = 0
    output: List[List[float]] = []

    for tick in control_ticks:
        while sample_index < len(sample_times) and sample_times[sample_index] <= tick + 1e-12:
            q = list(positions[sample_index])
            current_time = sample_times[sample_index]
            if not initialized:
                q_hat = q
                v_hat = [0.0] * JOINT_COUNT
                initialized = True
            else:
                dt = current_time - previous_time
                if dt <= 1e-9 or dt > reset_gap_sec:
                    q_hat = q
                    v_hat = [0.0] * JOINT_COUNT
                else:
                    predicted_q = [q_hat[i] + v_hat[i] * dt for i in range(JOINT_COUNT)]
                    residual = [q[i] - predicted_q[i] for i in range(JOINT_COUNT)]
                    q_hat = [
                        predicted_q[i] + alpha * residual[i]
                        for i in range(JOINT_COUNT)
                    ]
                    v_hat = [
                        v_hat[i] + beta * residual[i] / dt
                        for i in range(JOINT_COUNT)
                    ]
            previous_time = current_time
            sample_index += 1
        output.append(list(v_hat if initialized else [0.0] * JOINT_COUNT))
    return output


def vector_norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def downsample_xy(
    xs: Sequence[float],
    ys: Sequence[float],
    max_points: int,
) -> Tuple[List[float], List[float]]:
    if len(xs) <= max_points:
        return list(xs), list(ys)
    stride = max(1, math.ceil(len(xs) / max_points))
    return list(xs[::stride]), list(ys[::stride])


def nice_number(value: float) -> str:
    if abs(value) >= 100.0:
        return f"{value:.0f}"
    if abs(value) >= 10.0:
        return f"{value:.1f}"
    if abs(value) >= 1.0:
        return f"{value:.2f}"
    return f"{value:.3f}"


def svg_chart(
    title: str,
    x_values: Sequence[float],
    series: Sequence[Tuple[str, Sequence[float], str]],
    y_label: str,
    max_points: int = 1400,
) -> str:
    width = 1120
    height = 270
    left = 72
    right = 24
    top = 34
    bottom = 42
    plot_w = width - left - right
    plot_h = height - top - bottom

    x_min = min(x_values)
    x_max = max(x_values)
    all_y = [value for _, values, _ in series for value in values]
    y_min = min(0.0, min(all_y))
    y_max = max(0.0, max(all_y))
    if y_max <= y_min:
        y_max = y_min + 1.0
    padding = 0.08 * (y_max - y_min)
    y_min -= padding
    y_max += padding

    def sx(x: float) -> float:
        if x_max <= x_min:
            return left
        return left + (x - x_min) / (x_max - x_min) * plot_w

    def sy(y: float) -> float:
        return top + plot_h - (y - y_min) / (y_max - y_min) * plot_h

    parts: List[str] = [
        f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(title)}">',
        f'<text x="{left}" y="22" class="chart-title">{html.escape(title)}</text>',
    ]

    for index in range(6):
        x = x_min + (x_max - x_min) * index / 5.0
        px = sx(x)
        parts.append(f'<line x1="{px:.2f}" y1="{top}" x2="{px:.2f}" y2="{top + plot_h}" class="grid"/>')
        parts.append(f'<text x="{px:.2f}" y="{height - 13}" text-anchor="middle" class="tick">{nice_number(x)}</text>')

    for index in range(5):
        y = y_min + (y_max - y_min) * index / 4.0
        py = sy(y)
        parts.append(f'<line x1="{left}" y1="{py:.2f}" x2="{left + plot_w}" y2="{py:.2f}" class="grid"/>')
        parts.append(f'<text x="{left - 8}" y="{py + 4:.2f}" text-anchor="end" class="tick">{nice_number(y)}</text>')

    zero_y = sy(0.0)
    parts.append(f'<line x1="{left}" y1="{zero_y:.2f}" x2="{left + plot_w}" y2="{zero_y:.2f}" class="zero"/>')
    parts.append(f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}" class="axis"/>')
    parts.append(f'<line x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" y2="{top + plot_h}" class="axis"/>')
    parts.append(f'<text x="{left + plot_w / 2:.2f}" y="{height - 2}" text-anchor="middle" class="axis-label">time [s]</text>')
    parts.append(
        f'<text x="14" y="{top + plot_h / 2:.2f}" transform="rotate(-90 14 {top + plot_h / 2:.2f})" '
        f'text-anchor="middle" class="axis-label">{html.escape(y_label)}</text>'
    )

    legend_x = left + plot_w - 300
    legend_y = top + 4
    for offset, (label, _, color) in enumerate(series):
        y = legend_y + offset * 18
        parts.append(f'<line x1="{legend_x}" y1="{y}" x2="{legend_x + 26}" y2="{y}" style="stroke:{color};stroke-width:3"/>')
        parts.append(f'<text x="{legend_x + 34}" y="{y + 4}" class="legend">{html.escape(label)}</text>')

    for label, values, color in series:
        xs, ys = downsample_xy(x_values, values, max_points)
        points = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in zip(xs, ys))
        parts.append(
            f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2" '
            f'stroke-linejoin="round" stroke-linecap="round"><title>{html.escape(label)}</title></polyline>'
        )

    parts.append("</svg>")
    return "\n".join(parts)


def build_html(
    dataset_path: Path,
    output_path: Path,
    control_rate: float,
    current_alpha: float,
    ab_alpha: float,
    ab_beta: float,
    times: Sequence[float],
    current_qd: Sequence[Sequence[float]],
    alpha_beta_qd: Sequence[Sequence[float]],
    sample_count: int,
) -> None:
    current_norm = [vector_norm(row) for row in current_qd]
    ab_norm = [vector_norm(row) for row in alpha_beta_qd]

    charts = [
        svg_chart(
            "Joint Velocity Norm",
            times,
            [
                ("current 50 Hz diff + EMA", current_norm, "#0b7285"),
                ("alpha-beta on q samples", ab_norm, "#d9480f"),
            ],
            "||qd|| [rad/s]",
        )
    ]
    colors = ["#0b7285", "#d9480f"]
    for joint in range(JOINT_COUNT):
        charts.append(
            svg_chart(
                f"Joint {joint + 1} Velocity",
                times,
                [
                    ("current", [row[joint] for row in current_qd], colors[0]),
                    ("alpha-beta", [row[joint] for row in alpha_beta_qd], colors[1]),
                ],
                "qd [rad/s]",
            )
        )

    duration = times[-1] - times[0] if times else 0.0
    body = "\n".join(charts)
    output_path.write_text(
        f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Velocity Filter Comparison</title>
  <style>
    body {{
      margin: 0;
      background: #f5f7fa;
      color: #1f2933;
      font-family: Arial, sans-serif;
    }}
    main {{
      max-width: 1180px;
      margin: 0 auto;
      padding: 24px;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 24px;
    }}
    .meta {{
      margin: 0 0 18px;
      line-height: 1.45;
      color: #52606d;
      font-size: 14px;
    }}
    .panel {{
      margin: 14px 0;
      padding: 12px 14px;
      background: white;
      border: 1px solid #d9e2ec;
      border-radius: 8px;
      overflow-x: auto;
    }}
    svg {{
      width: 100%;
      min-width: 860px;
      height: auto;
      display: block;
    }}
    .chart-title {{
      font-size: 16px;
      font-weight: 700;
      fill: #1f2933;
    }}
    .grid {{
      stroke: #d9e2ec;
      stroke-width: 1;
    }}
    .zero {{
      stroke: #9fb3c8;
      stroke-width: 1.2;
    }}
    .axis {{
      stroke: #334e68;
      stroke-width: 1.2;
    }}
    .tick, .legend, .axis-label {{
      fill: #52606d;
      font-size: 12px;
    }}
    code {{
      background: #eef2f7;
      padding: 2px 4px;
      border-radius: 4px;
    }}
  </style>
</head>
<body>
<main>
  <h1>Velocity Filter Comparison</h1>
  <p class="meta">
    Dataset: <code>{html.escape(str(dataset_path))}</code><br>
    Samples: {sample_count}, duration: {duration:.3f} s, output ticks: {len(times)} at {control_rate:.1f} Hz<br>
    Current method: <code>qd = EMA((q[k] - q[k-1]) / control_dt)</code>, alpha={current_alpha:.3f}<br>
    Alpha-beta method: update on q sample timestamps, alpha={ab_alpha:.3f}, beta={ab_beta:.3f}, output held at control ticks.
  </p>
  {''.join(f'<div class="panel">{chart}</div>' for chart in charts)}
</main>
</body>
</html>
""",
        encoding="utf-8",
    )


def latest_csv(default_dir: Path) -> Path:
    files = sorted(default_dir.glob("*.csv"), key=lambda path: path.stat().st_mtime, reverse=True)
    if not files:
        raise RuntimeError(f"No CSV files found in {default_dir}")
    return files[0]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare current 50 Hz finite-difference velocity estimation with alpha-beta filtered q velocity."
    )
    parser.add_argument(
        "csv",
        nargs="?",
        help="RMP dataset CSV. If omitted, uses latest CSV in ~/ros2_ws/data/rmp_datasets.",
    )
    parser.add_argument("--control-rate", type=float, default=50.0, help="Controller output rate in Hz.")
    parser.add_argument("--current-alpha", type=float, default=0.5, help="EMA alpha for current controller estimator.")
    parser.add_argument("--ab-alpha", type=float, default=0.35, help="Alpha-beta position residual alpha.")
    parser.add_argument("--ab-beta", type=float, default=0.015, help="Alpha-beta velocity residual beta.")
    parser.add_argument("--reset-gap-sec", type=float, default=0.1, help="Reset filter if input timestamp gap exceeds this value.")
    parser.add_argument("--output", "-o", help="Output HTML path.")
    args = parser.parse_args()

    dataset_path = Path(args.csv).expanduser() if args.csv else latest_csv(Path("~/ros2_ws/data/rmp_datasets").expanduser())
    dataset_path = dataset_path.resolve()
    sample_times, positions = read_dataset(dataset_path)
    control_ticks, control_positions = make_control_samples(sample_times, positions, args.control_rate)
    current_qd = current_controller_velocity(control_positions, args.control_rate, args.current_alpha)
    ab_qd = alpha_beta_velocity(
        sample_times,
        positions,
        control_ticks,
        args.ab_alpha,
        args.ab_beta,
        args.reset_gap_sec,
    )

    if args.output:
        output_path = Path(args.output).expanduser().resolve()
    else:
        output_path = dataset_path.with_name(f"{dataset_path.stem}_velocity_filter_comparison.html")
    build_html(
        dataset_path,
        output_path,
        args.control_rate,
        args.current_alpha,
        args.ab_alpha,
        args.ab_beta,
        control_ticks,
        current_qd,
        ab_qd,
        len(sample_times),
    )
    print(f"Saved comparison plot: {output_path}")


if __name__ == "__main__":
    main()
