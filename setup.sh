#!/usr/bin/env bash
set -e

echo "[setup] Installing system dependencies (if using apt)..."

if command -v apt-get >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y build-essential cmake libeigen3-dev python3 python3-pip
else
  echo "apt-get not found. Please install build tools, CMake, Eigen3, Python3, and pip manually."
fi

echo "[setup] Installing Python dependencies..."
pip3 install -r requirements.txt

echo "[setup] Done."
