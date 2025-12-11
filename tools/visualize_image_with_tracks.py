#!/usr/bin/env python3
import sys
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

ROOT_DIR = Path(__file__).resolve().parent.parent
OUTPUT_DIR = ROOT_DIR / "output"


def load_last_positions(filename: Path, id_field: str):
    """Load positions (x, y) at the last timestamp from a CSV file."""
    positions = {}
    last_time = None
    with filename.open(newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row["time"])
            if last_time is None or t > last_time:
                last_time = t
                positions.clear()
            if t == last_time:
                obj_id = int(row[id_field])
                x = float(row["x"])
                y = float(row["y"])
                positions[obj_id] = (x, y)
    return last_time, positions


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 tools/visualize_image_with_tracks.py <image_path>")
        sys.exit(1)

    img_path = Path(sys.argv[1])
    if not img_path.is_absolute():
        img_path = (ROOT_DIR / img_path).resolve()

    if not img_path.exists():
        print(f"Image not found: {img_path}")
        sys.exit(1)

    gt_file = OUTPUT_DIR / "ground_truth.csv"
    tracks_file = OUTPUT_DIR / "tracks.csv"

    if not gt_file.exists() or not tracks_file.exists():
        print(f"Missing CSV files in {OUTPUT_DIR}. Run the simulation first (e.g., ./run_all.sh).")
        sys.exit(1)

    gt_time, gt_pos = load_last_positions(gt_file, "obj_id")
    tr_time, tr_pos = load_last_positions(tracks_file, "track_id")

    if not gt_pos and not tr_pos:
        print("No positions found in CSVs.")
        sys.exit(1)

    # Collect all coordinates for scaling
    xs = [p[0] for p in gt_pos.values()] + [p[0] for p in tr_pos.values()]
    ys = [p[1] for p in gt_pos.values()] + [p[1] for p in tr_pos.values()]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    img = mpimg.imread(img_path)
    h, w = img.shape[0], img.shape[1]

    def world_to_img(x, y):
        # Normalize world coordinates to image coordinates
        if max_x == min_x:
            ix = 0.5 * w
        else:
            ix = (x - min_x) / (max_x - min_x) * w

        if max_y == min_y:
            iy = 0.5 * h
        else:
            # Flip y-axis so that increasing y goes "up" on the image
            iy = h - (y - min_y) / (max_y - min_y) * h

        return ix, iy

    fig, ax = plt.subplots()
    ax.imshow(img)

    # Plot ground truth positions (last frame)
    for idx, (oid, (x, y)) in enumerate(gt_pos.items()):
        ix, iy = world_to_img(x, y)
        label = f"GT {oid}" if idx == 0 else None
        ax.scatter(ix, iy, marker="o", s=50, label=label)

    # Plot track positions (last frame)
    for idx, (tid, (x, y)) in enumerate(tr_pos.items()):
        ix, iy = world_to_img(x, y)
        label = f"TR {tid}" if idx == 0 else None
        ax.scatter(ix, iy, marker="x", s=60, label=label)

    ax.set_axis_off()
    t_str = tr_time if tr_time is not None else gt_time
    if t_str is None:
        title = f"Tracks over {img_path.name}"
    else:
        title = f"Tracks at t={t_str:.2f}s over {img_path.name}"
    ax.set_title(title)

    if gt_pos or tr_pos:
        ax.legend()

    OUTPUT_DIR.mkdir(exist_ok=True)
    out_img_path = OUTPUT_DIR / "visualization.png"
    fig.savefig(out_img_path, bbox_inches="tight", dpi=150)
    print(f"Saved visualization to {out_img_path}")
    plt.show()


if __name__ == "__main__":
    main()
