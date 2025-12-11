#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt

# Automatically adjust layout when the window is resized
plt.rcParams["figure.constrained_layout.use"] = True

ROOT_DIR = Path(__file__).resolve().parent.parent
OUTPUT_DIR = ROOT_DIR / "output"


def load_tracks(filename: Path):
    data = {}
    with filename.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            tid = int(row["track_id"])
            t = float(row["time"])
            x = float(row["x"])
            y = float(row["y"])
            if tid not in data:
                data[tid] = {"t": [], "x": [], "y": []}
            data[tid]["t"].append(t)
            data[tid]["x"].append(x)
            data[tid]["y"].append(y)
    return data


def load_gt(filename: Path):
    data = {}
    with filename.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            oid = int(row["obj_id"])
            t = float(row["time"])
            x = float(row["x"])
            y = float(row["y"])
            if oid not in data:
                data[oid] = {"t": [], "x": [], "y": []}
            data[oid]["t"].append(t)
            data[oid]["x"].append(x)
            data[oid]["y"].append(y)
    return data


def main():
    gt_file = OUTPUT_DIR / "ground_truth.csv"
    tracks_file = OUTPUT_DIR / "tracks.csv"

    if not gt_file.exists() or not tracks_file.exists():
        print(f"Cannot find CSV files in {OUTPUT_DIR}.")
        print("Run the simulation first (e.g., ./run_all.sh).")
        return

    gt = load_gt(gt_file)
    tracks = load_tracks(tracks_file)

    # Create figure and axes (layout will be auto-adjusted)
    fig, ax = plt.subplots(constrained_layout=True)

    # Set window title (if supported by backend)
    try:
        fig.canvas.manager.set_window_title("Ground Truth vs Tracks (top view)")
    except Exception:
        pass

    # Ground truth trajectories (dashed)
    for oid, d in gt.items():
        ax.plot(d["x"], d["y"], linestyle="--", label=f"GT {oid}")

    # Tracked trajectories
    for tid, d in tracks.items():
        ax.plot(d["x"], d["y"], label=f"TR {tid}")

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Ground Truth vs Tracks (top view)")

    # Keep x/y scale ratio, so the trajectories are not visually distorted
    ax.grid(True)
    ax.set_aspect("equal", adjustable="datalim")

    # Place legend outside the right side of the axes
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(
            handles,
            labels,
            loc="upper left",
            bbox_to_anchor=(1.02, 1.0),
            borderaxespad=0.0,
        )

    plt.show()


if __name__ == "__main__":
    main()
