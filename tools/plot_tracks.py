#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt

def load_tracks(filename):
    data = {}
    with open(filename, newline='') as f:
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

def load_gt(filename):
    data = {}
    with open(filename, newline='') as f:
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
    gt = load_gt("ground_truth.csv")
    tracks = load_tracks("tracks.csv")

    plt.figure()
    # ground truth 궤적
    for oid, d in gt.items():
        plt.plot(d["x"], d["y"], linestyle="--", label=f"GT {oid}")

    # 추적 궤적
    for tid, d in tracks.items():
        plt.plot(d["x"], d["y"], label=f"TR {tid}")

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Ground Truth vs Tracks (top view)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()

if __name__ == "__main__":
    main()
