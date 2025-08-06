#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
# Allow specifying parallelism, defaulting to the number of processors.
NPROC=${1:-$(nproc)}
cmake -S . -B build >/dev/null
cmake --build build --target testpic -- -j"${NPROC}" >/dev/null
