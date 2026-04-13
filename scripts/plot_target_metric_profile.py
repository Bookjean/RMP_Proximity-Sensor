#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def alpha_profile(distances, length_scale, min_metric_alpha):
    scaled = distances / length_scale
    return (1.0 - min_metric_alpha) * np.exp(-0.5 * scaled * scaled) + min_metric_alpha


def parallel_eigenvalue(alpha, max_metric_scalar, min_metric_scalar):
    return alpha * max_metric_scalar + (1.0 - alpha) * min_metric_scalar


def orthogonal_eigenvalue(alpha, max_metric_scalar):
    return alpha * max_metric_scalar


def main():
    parser = argparse.ArgumentParser(
        description="Plot target RMP metric profile without the proximity boost term."
    )
    parser.add_argument("--max-metric-scalar", type=float, required=True)
    parser.add_argument("--min-metric-scalar", type=float, required=True)
    parser.add_argument("--min-metric-alpha", type=float, required=True)
    parser.add_argument(
        "--length-scales",
        type=float,
        nargs="+",
        required=True,
        help="One or more target_rmp_metric_alpha_length_scale values to compare.",
    )
    parser.add_argument("--max-distance", type=float, default=1.0)
    parser.add_argument("--num-samples", type=int, default=400)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    distances = np.linspace(0.0, args.max_distance, args.num_samples)
    fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True)

    for length_scale in args.length_scales:
        alpha = alpha_profile(distances, length_scale, args.min_metric_alpha)
        lam_parallel = parallel_eigenvalue(
            alpha, args.max_metric_scalar, args.min_metric_scalar
        )
        lam_orthogonal = orthogonal_eigenvalue(alpha, args.max_metric_scalar)
        label = f"length_scale={length_scale:g}"

        axes[0].plot(distances, alpha, label=label)
        axes[1].plot(distances, lam_parallel, label=label)
        axes[2].plot(distances, lam_orthogonal, label=label)

    axes[0].set_title(
        "Target RMP Metric Profile Without Proximity Boost\n"
        f"min_metric_alpha={args.min_metric_alpha:g}, "
        f"max/min scalar={args.max_metric_scalar:g}/{args.min_metric_scalar:g}"
    )
    axes[0].set_ylabel("alpha(distance)")
    axes[1].set_ylabel("parallel eigenvalue")
    axes[2].set_ylabel("orthogonal eigenvalue")
    axes[2].set_xlabel("goal distance [m]")

    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")

    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    print(args.output)


if __name__ == "__main__":
    main()
