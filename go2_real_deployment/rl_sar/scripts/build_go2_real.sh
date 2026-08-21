#!/usr/bin/env bash

# Configure, compile, and locally install only the Unitree Go2 hardware target.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "${SCRIPT_DIR}")"
source "${SCRIPT_DIR}/common.sh"

BUILD_DIR="${RL_SAR_BUILD_DIR:-${PROJECT_ROOT}/cmake_build/go2_real}"
INSTALL_PREFIX="${RL_SAR_INSTALL_PREFIX:-${PROJECT_ROOT}/install/go2_real}"
INFERENCE_RUNTIME_DIR="${RL_SAR_INFERENCE_RUNTIME_DIR:-${PROJECT_ROOT}/library/inference_runtime}"
POLICY_DIR="${RL_SAR_POLICY_DIR:-${PROJECT_ROOT}/policy}"
PYTHON_EXECUTABLE="${RL_SAR_PYTHON_EXECUTABLE:-/usr/bin/python3}"
INFERENCE_BACKEND="${RL_SAR_INFERENCE_BACKEND:-TORCH}"
INFERENCE_BACKEND="${INFERENCE_BACKEND^^}"
BUILD_TYPE="Release"
JOBS="$(nproc 2>/dev/null || echo 1)"
CLEAN=false

usage() {
    cat <<'EOF'
Usage: ./scripts/build_go2_real.sh [options]

Options:
  --clean         Remove this script's local Go2 build/install outputs first
  --debug         Build with CMAKE_BUILD_TYPE=Debug
  --release       Build with CMAKE_BUILD_TYPE=Release (default)
  --jobs <count>  Parallel compile jobs (default: number of host CPUs)
  -h, --help      Show this help

Optional environment overrides:
  RL_SAR_BUILD_DIR
  RL_SAR_INSTALL_PREFIX
  RL_SAR_INFERENCE_RUNTIME_DIR
  RL_SAR_INFERENCE_BACKEND (TORCH or ONNX; default: TORCH)
  RL_SAR_POLICY_DIR
  RL_SAR_PYTHON_EXECUTABLE
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --clean)
            CLEAN=true
            shift
            ;;
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --release)
            BUILD_TYPE="Release"
            shift
            ;;
        --jobs)
            if [[ $# -lt 2 || ! "$2" =~ ^[1-9][0-9]*$ ]]; then
                print_error "--jobs requires a positive integer."
                exit 2
            fi
            JOBS="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            usage
            exit 2
            ;;
    esac
done

case "${INFERENCE_BACKEND}" in
    TORCH|ONNX) ;;
    *)
        print_error "RL_SAR_INFERENCE_BACKEND must be TORCH or ONNX for a reproducible deployment build."
        exit 2
        ;;
esac

# Resolve lexical '..' components and existing symlinks before any cleanup.
# This keeps environment overrides useful without allowing --clean to escape
# the two generated-output roots owned by this script.
if ! command -v realpath >/dev/null 2>&1; then
    print_error "realpath is required for safe build-directory handling."
    exit 1
fi
BUILD_DIR="$(realpath -m -- "${BUILD_DIR}")"
INSTALL_PREFIX="$(realpath -m -- "${INSTALL_PREFIX}")"
ALLOWED_BUILD_ROOT="$(realpath -m -- "${PROJECT_ROOT}/cmake_build")"
ALLOWED_INSTALL_ROOT="$(realpath -m -- "${PROJECT_ROOT}/install")"

if [[ "${CLEAN}" == true ]]; then
    case "${BUILD_DIR}" in
        "${ALLOWED_BUILD_ROOT}"/*)
            rm -rf -- "${BUILD_DIR}"
            ;;
        *)
            print_error "Refusing to clean build directory outside ${ALLOWED_BUILD_ROOT}: ${BUILD_DIR}"
            exit 1
            ;;
    esac
    case "${INSTALL_PREFIX}" in
        "${ALLOWED_INSTALL_ROOT}"/*)
            rm -rf -- "${INSTALL_PREFIX}"
            ;;
        *)
            print_error "Refusing to clean install directory outside ${ALLOWED_INSTALL_ROOT}: ${INSTALL_PREFIX}"
            exit 1
            ;;
    esac
fi

export RL_SAR_INFERENCE_RUNTIME_DIR="${INFERENCE_RUNTIME_DIR}"
export RL_SAR_INFERENCE_BACKEND="${INFERENCE_BACKEND}"
export RL_SAR_POLICY_DIR="${POLICY_DIR}"
"${SCRIPT_DIR}/check_go2_real_environment.sh"

print_header "Configuring standalone rl_real_go2"
print_info "Build directory: ${BUILD_DIR}"
print_info "Install prefix: ${INSTALL_PREFIX}"
print_info "Policy directory: ${POLICY_DIR}"
print_info "Inference runtime: ${INFERENCE_RUNTIME_DIR}"
print_info "Inference backend: ${INFERENCE_BACKEND}"
print_info "Python interpreter: ${PYTHON_EXECUTABLE}"

cmake \
    -U "Torch_DIR" \
    -U "ONNX_RUNTIME_LIB_PATH" \
    -S "${PROJECT_ROOT}/src/rl_sar" \
    -B "${BUILD_DIR}" \
    -DUSE_CMAKE=ON \
    -DUSE_MUJOCO=OFF \
    -DRL_SAR_GO2_REAL_ONLY=ON \
    -DRL_SAR_POLICY_DIR="${POLICY_DIR}" \
    -DRL_SAR_INFERENCE_RUNTIME_DIR="${INFERENCE_RUNTIME_DIR}" \
    -DRL_SAR_INFERENCE_BACKEND="${INFERENCE_BACKEND}" \
    -DPython3_EXECUTABLE="${PYTHON_EXECUTABLE}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"

print_header "Building rl_real_go2"
cmake --build "${BUILD_DIR}" --target rl_real_go2 go2_state_probe validate_policy_model validate_policy_config --parallel "${JOBS}"
cmake --install "${BUILD_DIR}"

for binary_name in rl_real_go2 go2_state_probe validate_policy_model validate_policy_config; do
    installed_binary="${INSTALL_PREFIX}/bin/${binary_name}"
    if [[ ! -x "${installed_binary}" ]]; then
        print_error "Expected installed executable is missing: ${installed_binary}"
        exit 1
    fi
done
INSTALLED_BINARY="${INSTALL_PREFIX}/bin/rl_real_go2"

if command -v ldd >/dev/null 2>&1; then
    LDD_OUTPUT="$(ldd "${INSTALLED_BINARY}")"
    if grep -q "not found" <<<"${LDD_OUTPUT}"; then
        print_error "The installed executable has unresolved shared libraries:"
        printf '%s\n' "${LDD_OUTPUT}"
        exit 1
    fi
fi

print_success "Locally staged rl_real_go2 at ${INSTALLED_BINARY}"
print_warning "Build success does not authorize a hardware run."
print_warning "Validate the selected policy contract and pass the read-only DDS probe before enabling motor commands."
