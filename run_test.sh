#!/usr/bin/env bash

if [[ -t 1 ]]; then
  C_RESET='\e[0m'
  C_BOLD='\e[1m'
  C_DIM='\e[2m'
  C_UNDER='\e[4m'

  C_RED='\e[31m'
  C_GRN='\e[32m'
  C_YEL='\e[33m'
  C_BLU='\e[34m'
  C_MAG='\e[35m'
  C_CYN='\e[36m'
  C_WHT='\e[37m'
else
  C_RESET= C_BOLD= C_DIM= C_UNDER= C_RED= C_GRN= C_YEL= C_BLU= C_MAG= C_CYN= C_WHT=
fi
log_info()    { printf "${C_BLU}%s${C_RESET}\n" "$*"; }
log_ok()      { printf "${C_GRN}%s${C_RESET}\n" "$*"; }
log_warn()    { printf "${C_YEL}%s${C_RESET}\n" "$*"; }
log_error()   { printf "${C_RED}%s${C_RESET}\n" "$*"; }


usage() {
BASENAME="run_test.sh"

echo "UwU"

}


REQUIRED_UTILS=(
    "realpath"
    "ctest"
)

for util in "${REQUIRED_UTILS[@]}"; do
    if ! command -v "$util" >/dev/null 2>&1; then
        log_error "Required utility is not found: '$util'"
        exit 1
    fi
done

BUILD_TYPE="Debug"
RUN_BENCH=false
VERBOSITY="output-on-failure"
while [[ $# -gt 0 ]]; do
  case "$1" in
  -t)
    shift
    BUILD_TYPE="$1"
    shift

    if [[ "$BUILD_TYPE" != "Debug" && "$BUILD_TYPE" != "Release" ]]; then
      log_error "Invalid build type: $BUILD_TYPE. Allowed values are 'Debug' or 'Release'."
      exit 1
    fi
    ;;
  -v|--verbose)
    VERBOSITY="verbose"
    shift
    ;;
  --bench)
    shift
    RUN_BENCH=true;
    BUILD_TYPE="Release"
    ;;
  --*)
    log_error "Error: Unknown flag: $1"
    usage
    exit 1
    ;;

  -*)
    log_error "Unknown option: $1"
    usage
    exit 1
    ;;
  *)
    ;;
  esac
done

echo "Preparing CMake configuration."
mkdir -p "./build/${BUILD_TYPE}"
cmake --log-level=DEBUG \
        -S "." -B "./build/${BUILD_TYPE}" -G Ninja \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DCMAKE_VERBOSE_MAKEFILE=ON \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DBUILD_TESTS=ON

echo "Building project (${BUILD_TYPE})."
ninja -C "./build/${BUILD_TYPE}"

if [[ $? -ne 0 ]]; then
  log_error "Build failed"
  exit 1
fi

if [[ "$RUN_BENCH" == true ]]; then
  log_info "Running benchmarks."
  BUILD_TYPE="Release"
fi

if [[ "$RUN_BENCH" == true ]]; then
  log_info "Running benchmarks."

  "./build/${BUILD_TYPE}/dsp-bench"

  exit "$?"
fi

ctest --test-dir "./build/${BUILD_TYPE}" "--$VERBOSITY"