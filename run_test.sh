#!/usr/bin/env bash

set -euo pipefail

# Usage:
#   scripts/build_and_test.sh [Debug|Release] [--ctest-args "..."]
#
# Examples:
#   scripts/build_and_test.sh
#   scripts/build_and_test.sh Release
#   scripts/build_and_test.sh Debug --ctest-args "-R dsp -j4 --output-on-failure"

BUILD_TYPE="${1:-Debug}"
shift || true

CTEST_ARGS=""
if [[ "${1:-}" == "--ctest-args" ]]; then
  shift
  CTEST_ARGS="${1:-}"
  shift || true
fi

BUILD_DIR="build/${BUILD_TYPE,,}"   # e.g., build-debug or build-release

echo "==> Configuring (${BUILD_TYPE}) with tests enabled..."
cmake -B "${BUILD_DIR}" -S . -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" -DBUILD_TESTS=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "==> Building..."
cmake --build "${BUILD_DIR}" --config "${BUILD_TYPE}" -- -j$(nproc)

echo "==> Running tests..."
./"${BUILD_DIR}/test_dsp"

echo "==> Done."
