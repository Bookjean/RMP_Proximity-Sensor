#!/usr/bin/env python3
import argparse
import csv
import html
import math
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


JOINT_COUNT = 6


def parse_float(value: str, default: float = 0.0) -> float:
    try:
        if value == "":
            return default
        parsed = float(value)
        return parsed if math.isfinite(parsed) else default
    except (TypeError, ValueError):
        return default


def read_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", newline="") as handle:
        lines = [line for line in handle if line.strip() and not line.startswith("#")]
    rows = list(csv.DictReader(lines))
    if not rows:
        raise RuntimeError(f"No data rows found: {path}")
    return rows


def norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def percentile(values: Sequence[float], ratio: float) -> float:
    if not values:
        return 0.0
    sorted_values = sorted(values)
    return sorted_values[int(ratio * (len(sorted_values) - 1))]


def vector_from(row: Dict[str, str], prefix: str) -> List[float]:
    return [parse_float(row.get(f"{prefix}{index}", "")) for index in range(1, JOINT_COUNT + 1)]


def dataset_series(path: Path) -> Dict[str, List[float]]:
    rows = read_rows(path)
    t_abs = [parse_float(row.get("timestamp_unix", "")) for row in rows]
    t0 = t_abs[0]
    times = [stamp - t0 for stamp in t_abs]

    qd_vectors = [vector_from(row, "qd") for row in rows]
    cmd_qd_vectors = [vector_from(row, "cmd_qd") for row in rows]
    q_vectors = [vector_from(row, "q") for row in rows]

    q_fdiff_norm = [0.0]
    for prev_q, q, prev_t, t in zip(q_vectors, q_vectors[1:], t_abs, t_abs[1:]):
      dt = t - prev_t
      if dt <= 1e-9:
          q_fdiff_norm.append(0.0)
      else:
          q_fdiff_norm.append(norm([(q[i] - prev_q[i]) / dt for i in range(JOINT_COUNT)]))

    goal_distance = []
    for row in rows:
        if row.get("goal_x", "") == "":
            goal_distance.append(0.0)
            continue
        goal = [parse_float(row.get(name, "")) for name in ("goal_x", "goal_y", "goal_z")]
        ee = [parse_float(row.get(name, "")) for name in ("ee_pose_x", "ee_pose_y", "ee_pose_z")]
        goal_distance.append(norm([goal[i] - ee[i] for i in range(3)]))

    output: Dict[str, List[float]] = {
        "time": times,
        "qd_norm": [norm(vector) for vector in qd_vectors],
        "cmd_qd_norm": [norm(vector) for vector in cmd_qd_vectors],
        "q_fdiff_norm": q_fdiff_norm,
        "ee_speed": [parse_float(row.get("ee_speed", "")) for row in rows],
        "goal_distance": goal_distance,
    }
    for joint in range(JOINT_COUNT):
        output[f"qd{joint + 1}"] = [vector[joint] for vector in qd_vectors]
        output[f"cmd_qd{joint + 1}"] = [vector[joint] for vector in cmd_qd_vectors]
    return output


def stats(values: Sequence[float]) -> Dict[str, float]:
    if not values:
        return {"mean": 0.0, "p50": 0.0, "p95": 0.0, "max": 0.0}
    return {
        "mean": sum(values) / len(values),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def downsample(xs: Sequence[float], ys: Sequence[float], max_points: int) -> Tuple[List[float], List[float]]:
    if len(xs) <= max_points:
        return list(xs), list(ys)
    stride = max(1, math.ceil(len(xs) / max_points))
    return list(xs[::stride]), list(ys[::stride])


def format_num(value: float) -> str:
    if abs(value) >= 10:
        return f"{value:.1f}"
    if abs(value) >= 1:
        return f"{value:.2f}"
    return f"{value:.3f}"


def svg_plot(
    title: str,
    y_label: str,
    datasets: Sequence[Tuple[str, Dict[str, List[float]], str]],
    key: str,
    max_points: int,
) -> str:
    width = 1120
    height = 280
    left, right, top, bottom = 70, 22, 34, 42
    plot_w = width - left - right
    plot_h = height - top - bottom

    all_x = [x for _, data, _ in datasets for x in data["time"]]
    all_y = [y for _, data, _ in datasets for y in data[key]]
    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(0.0, min(all_y)), max(0.0, max(all_y))
    if y_max <= y_min:
        y_max = y_min + 1.0
    pad = 0.08 * (y_max - y_min)
    y_min -= pad
    y_max += pad

    def sx(x: float) -> float:
        return left + (x - x_min) / (x_max - x_min) * plot_w if x_max > x_min else left

    def sy(y: float) -> float:
        return top + plot_h - (y - y_min) / (y_max - y_min) * plot_h

    parts = [
        f'<svg viewBox="0 0 {width} {height}">',
        f'<text x="{left}" y="22" class="chart-title">{html.escape(title)}</text>',
    ]
    for index in range(6):
        x = x_min + (x_max - x_min) * index / 5.0
        px = sx(x)
        parts.append(f'<line x1="{px:.2f}" y1="{top}" x2="{px:.2f}" y2="{top + plot_h}" class="grid"/>')
        parts.append(f'<text x="{px:.2f}" y="{height - 13}" text-anchor="middle" class="tick">{format_num(x)}</text>')
    for index in range(5):
        y = y_min + (y_max - y_min) * index / 4.0
        py = sy(y)
        parts.append(f'<line x1="{left}" y1="{py:.2f}" x2="{left + plot_w}" y2="{py:.2f}" class="grid"/>')
        parts.append(f'<text x="{left - 8}" y="{py + 4:.2f}" text-anchor="end" class="tick">{format_num(y)}</text>')
    zero_y = sy(0.0)
    parts.append(f'<line x1="{left}" y1="{zero_y:.2f}" x2="{left + plot_w}" y2="{zero_y:.2f}" class="zero"/>')
    parts.append(f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}" class="axis"/>')
    parts.append(f'<line x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" y2="{top + plot_h}" class="axis"/>')
    parts.append(f'<text x="{left + plot_w / 2:.2f}" y="{height - 2}" text-anchor="middle" class="axis-label">time [s]</text>')
    parts.append(
        f'<text x="14" y="{top + plot_h / 2:.2f}" transform="rotate(-90 14 {top + plot_h / 2:.2f})" '
        f'text-anchor="middle" class="axis-label">{html.escape(y_label)}</text>'
    )

    legend_x = left + plot_w - 250
    for index, (label, _, color) in enumerate(datasets):
        y = top + 6 + index * 18
        parts.append(f'<line x1="{legend_x}" y1="{y}" x2="{legend_x + 26}" y2="{y}" style="stroke:{color};stroke-width:3"/>')
        parts.append(f'<text x="{legend_x + 34}" y="{y + 4}" class="legend">{html.escape(label)}</text>')

    for label, data, color in datasets:
        xs, ys = downsample(data["time"], data[key], max_points)
        points = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in zip(xs, ys))
        parts.append(
            f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2" '
            f'stroke-linejoin="round" stroke-linecap="round"><title>{html.escape(label)}</title></polyline>'
        )
    parts.append("</svg>")
    return "\n".join(parts)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot recorded velocity columns from two RMP datasets.")
    parser.add_argument("csv", nargs="+", help="Two or more dataset CSV files.")
    parser.add_argument("--labels", nargs="+", help="Labels for each dataset.")
    parser.add_argument("-o", "--output", help="Output HTML path.")
    parser.add_argument("--max-points", type=int, default=1800)
    args = parser.parse_args()

    paths = [Path(path).expanduser().resolve() for path in args.csv]
    labels = args.labels or [path.stem for path in paths]
    if len(labels) != len(paths):
        raise RuntimeError("--labels count must match CSV count")
    colors = ["#0b7285", "#d9480f", "#5c7cfa", "#2b8a3e"]
    data = [(label, dataset_series(path), colors[index % len(colors)]) for index, (label, path) in enumerate(zip(labels, paths))]

    output = Path(args.output).expanduser().resolve() if args.output else paths[-1].with_name("velocity_dataset_comparison.html")
    rows = []
    for label, series, _ in data:
        row = [html.escape(label)]
        for key in ("qd_norm", "cmd_qd_norm", "q_fdiff_norm", "ee_speed", "goal_distance"):
            values = series[key]
            s = stats(values)
            row.append(f'{s["mean"]:.4f} / {s["p95"]:.4f} / {s["max"]:.4f}')
        rows.append("<tr>" + "".join(f"<td>{cell}</td>" for cell in row) + "</tr>")

    charts = [
        svg_plot("Recorded Solver Velocity: ||qd||", "rad/s", data, "qd_norm", args.max_points),
        svg_plot("Commanded Velocity: ||cmd_qd||", "rad/s", data, "cmd_qd_norm", args.max_points),
        svg_plot("Measured q Finite Difference: ||dq/dt||", "rad/s", data, "q_fdiff_norm", args.max_points),
        svg_plot("End-Effector Speed", "m/s", data, "ee_speed", args.max_points),
        svg_plot("Goal Distance", "m", data, "goal_distance", args.max_points),
    ]
    for joint in range(1, JOINT_COUNT + 1):
        charts.append(svg_plot(f"Recorded qd{joint}", "rad/s", data, f"qd{joint}", args.max_points))

    output.write_text(
        f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Velocity Dataset Comparison</title>
  <style>
    body {{ margin: 0; background: #f5f7fa; color: #1f2933; font-family: Arial, sans-serif; }}
    main {{ max-width: 1180px; margin: 0 auto; padding: 24px; }}
    h1 {{ margin: 0 0 12px; font-size: 24px; }}
    table {{ width: 100%; border-collapse: collapse; margin: 12px 0 18px; background: white; border: 1px solid #d9e2ec; }}
    th, td {{ padding: 8px 10px; border-bottom: 1px solid #d9e2ec; font-size: 13px; text-align: left; }}
    th {{ background: #eef2f7; }}
    .note {{ color: #52606d; font-size: 13px; line-height: 1.45; }}
    .panel {{ margin: 14px 0; padding: 12px 14px; background: white; border: 1px solid #d9e2ec; border-radius: 8px; overflow-x: auto; }}
    svg {{ width: 100%; min-width: 860px; height: auto; display: block; }}
    .chart-title {{ font-size: 16px; font-weight: 700; fill: #1f2933; }}
    .grid {{ stroke: #d9e2ec; stroke-width: 1; }}
    .zero {{ stroke: #9fb3c8; stroke-width: 1.2; }}
    .axis {{ stroke: #334e68; stroke-width: 1.2; }}
    .tick, .legend, .axis-label {{ fill: #52606d; font-size: 12px; }}
    code {{ background: #eef2f7; padding: 2px 4px; border-radius: 4px; }}
  </style>
</head>
<body>
<main>
  <h1>Velocity Dataset Comparison</h1>
  <p class="note">Stats are <code>mean / p95 / max</code>. Inputs: {'; '.join(html.escape(str(path)) for path in paths)}</p>
  <table>
    <thead><tr><th>dataset</th><th>qd norm</th><th>cmd_qd norm</th><th>q finite-diff norm</th><th>EE speed</th><th>goal distance</th></tr></thead>
    <tbody>{''.join(rows)}</tbody>
  </table>
  {''.join(f'<div class="panel">{chart}</div>' for chart in charts)}
</main>
</body>
</html>
""",
        encoding="utf-8",
    )
    print(f"Saved comparison plot: {output}")


if __name__ == "__main__":
    main()
