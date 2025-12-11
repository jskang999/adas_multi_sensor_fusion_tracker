#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[clean] Removing build/ and output/ directories..."
rm -rf "$ROOT_DIR/build" "$ROOT_DIR/output"

echo "[clean] Removing Python __pycache__ directories..."
find "$ROOT_DIR" -name "__pycache__" -type d -exec rm -rf {} +

echo "[clean] Done."
