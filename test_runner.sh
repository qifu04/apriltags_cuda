#!/usr/bin/env bash
set -euo pipefail

# CD to the script's directory to ensure correct paths
cd "$(dirname "$0")"

IMAGE_NAME=cuda-build

# Check if the image exists. If not, instruct the user to run the build script.
docker image inspect "$IMAGE_NAME" > /dev/null 2>&1 || { echo "Docker image '$IMAGE_NAME' not found. Please run docker_build.sh first."; exit 1; }

# Run the testpic executable with GPU support.
# We mount the parent directory (/root) to /workspace to access both the executable and the test image.
# The working directory is set to /workspace/apriltags_cuda.
docker run --rm --gpus all --memory=16g -v "$(pwd)/..":/workspace -w /workspace/apriltags_cuda "$IMAGE_NAME" \
  ./build/testpic /workspace/project/testfiles/Pic.bmp
