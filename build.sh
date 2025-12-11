#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

mkdir -p "$ROOT_DIR/build"
mkdir -p "$ROOT_DIR/output"

cd "$ROOT_DIR/build"
cmake .. -DBUILD_EXAMPLES=ON
make -j

./run_simulation 5 300 "$ROOT_DIR/output"