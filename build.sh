#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
cmake -S . -B build >/dev/null
cmake --build build --target testpic -- -j$(nproc) >/dev/null
