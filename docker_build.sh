#!/usr/bin/env bash
set -euo pipefail

# CD to the script's directory to ensure correct volume mounts
cd "$(dirname "$0")"

IMAGE_NAME=cuda-build

docker image inspect "$IMAGE_NAME" > /dev/null 2>&1 || docker build -t "$IMAGE_NAME" .

docker run --rm --memory=16g -v "$(pwd)":/workspace/apriltags_cuda -w /workspace/apriltags_cuda "$IMAGE_NAME" \
  bash -c "cmake -S . -B build && cmake --build build --target testpic -- -j28"
