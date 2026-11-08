#!/usr/bin/env python3
"""Compute simple reproducible metrics and plot a recorded run CSV."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

OBSTACLES = ((1.65, 0.00, 0.24), (3.10, 0.68, 0.28), (4.40, -0.55, 0.39))
ROBOT_RADIUS = 0.18


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path)
    args = parser.parse_args()
    if not args.csv_file.is_file():
        parser.error(f"file not found: {args.csv_file}")

    with args.csv_file.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        parser.error("recording must contain at least two odometry samples")

    data = {key: np.asarray([float(row[key]) for row in rows], dtype=float)
            for key in rows[0].keys()}
    dx = np.diff(data["x"])
    dy = np.diff(data["y"])
    path_length = float(np.sum(np.hypot(dx, dy)))
    elapsed = float(data["time_s"][-1] - data["time_s"][0])
    min_clearance = float("inf")
    for ox, oy, obstacle_radius in OBSTACLES:
        clearance = np.hypot(data["x"] - ox, data["y"] - oy) - ROBOT_RADIUS - obstacle_radius
        min_clearance = min(min_clearance, float(np.min(clearance)))

    metrics = {
        "samples": int(len(rows)),
        "duration_s": elapsed,
        "path_length_m": path_length,
        "mean_abs_cmd_speed_mps": float(np.mean(np.abs(data["cmd_v"]))),
        "cmd_speed_variance": float(np.var(data["cmd_v"])),
        "mean_abs_reference_speed_mps": float(np.mean(np.abs(data["ref_v"]))),
        "mean_abs_speed_filter_delta_mps": float(np.mean(np.abs(data["cmd_v"] - data["ref_v"]))),
        "approx_min_clearance_m": min_clearance,
        "clearance_model": "2-D center distance minus fixed circular footprint radii; approximate only",
    }
    output_json = args.csv_file.with_suffix(".metrics.json")
    output_json.write_text(json.dumps(metrics, indent=2), encoding="utf-8")

    try:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.plot(data["x"], data["y"], label="robot trajectory", linewidth=2)
        for index, (ox, oy, radius) in enumerate(OBSTACLES, start=1):
            circle = plt.Circle((ox, oy), radius, alpha=0.35, label=f"obstacle {index}")
            ax.add_patch(circle)
            footprint = plt.Circle((ox, oy), radius + ROBOT_RADIUS, fill=False, linestyle="--", alpha=0.5)
            ax.add_patch(footprint)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title("Safe obstacle avoidance trajectory")
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        output_png = args.csv_file.with_suffix(".trajectory.png")
        fig.savefig(output_png, dpi=160)
        plt.close(fig)
        metrics["trajectory_plot"] = str(output_png)
        output_json.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
        print(f"Trajectory plot: {output_png}")
    except ImportError:
        print("matplotlib not installed; metrics JSON was still generated")
    print(json.dumps(metrics, indent=2))
    print(f"Metrics: {output_json}")


if __name__ == "__main__":
    main()
