#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME=cuda-build

docker image inspect "$IMAGE_NAME" > /dev/null 2>&1 || docker build -t "$IMAGE_NAME" .

docker run --rm -v "$(pwd)":/workspace/apriltags_cuda -w /workspace/apriltags_cuda "$IMAGE_NAME" \
  bash -c "cmake -S . -B build && cmake --build build --target testpic"
