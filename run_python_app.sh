#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$ROOT_DIR"
python3 tools/plot_tracks.py
python3 tools/visualize_image_with_tracks.py data/road.png