#!/usr/bin/env bash

# Initialize only the third-party components needed by rl_real_go2.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "${SCRIPT_DIR}")"
source "${SCRIPT_DIR}/common.sh"

RUNTIME_TARGET="libtorch"
SKIP_RUNTIME=false
SDK_RELATIVE_PATH="src/rl_sar/library/thirdparty/robot_sdk/unitree/unitree_sdk2"
SDK_PATH="${PROJECT_ROOT}/${SDK_RELATIVE_PATH}"

usage() {
    cat <<'EOF'
Usage: ./scripts/setup_go2_real.sh [options]

Options:
  --runtime <libtorch|onnx|all>  Runtime to download (default: libtorch)
  --skip-runtime                Initialize SDK2 without downloading a runtime
  -h, --help                    Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --runtime)
            if [[ $# -lt 2 ]]; then
                print_error "--runtime requires libtorch, onnx, or all."
                exit 2
            fi
            RUNTIME_TARGET="$2"
            shift 2
            ;;
        --skip-runtime)
            SKIP_RUNTIME=true
            shift
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

case "${RUNTIME_TARGET}" in
    libtorch|onnx|all) ;;
    *)
        print_error "Unsupported runtime '${RUNTIME_TARGET}'. Use libtorch, onnx, or all."
        exit 2
        ;;
esac

if ! GIT_ROOT="$(git -C "${PROJECT_ROOT}" rev-parse --show-toplevel 2>/dev/null)"; then
    print_error "${PROJECT_ROOT} is not a Git checkout."
    exit 1
fi

PROJECT_GIT_PATH="$(realpath --relative-to="${GIT_ROOT}" "${PROJECT_ROOT}")"
if [[ "${PROJECT_GIT_PATH}" == "." ]]; then
    SDK_GIT_PATH="${SDK_RELATIVE_PATH}"
else
    SDK_GIT_PATH="${PROJECT_GIT_PATH}/${SDK_RELATIVE_PATH}"
fi

print_info "Initializing the pinned Unitree SDK2 submodule only..."
git -C "${GIT_ROOT}" submodule update --init --recursive "${SDK_GIT_PATH}"

EXPECTED_SDK_COMMIT="$(git -C "${GIT_ROOT}" ls-tree HEAD "${SDK_GIT_PATH}" | awk '{print $3}')"
ACTUAL_SDK_COMMIT="$(git -C "${SDK_PATH}" rev-parse HEAD)"
if [[ -z "${EXPECTED_SDK_COMMIT}" || "${ACTUAL_SDK_COMMIT}" != "${EXPECTED_SDK_COMMIT}" ]]; then
    print_error "SDK2 is not at the commit pinned by this repository."
    print_error "Expected: ${EXPECTED_SDK_COMMIT:-unknown}"
    print_error "Actual:   ${ACTUAL_SDK_COMMIT}"
    exit 1
fi
print_success "Unitree SDK2 is pinned at ${ACTUAL_SDK_COMMIT}."

if [[ "${SKIP_RUNTIME}" == false ]]; then
    print_info "Preparing ${RUNTIME_TARGET} in library/inference_runtime..."
    "${SCRIPT_DIR}/download_inference_runtime.sh" "library/inference_runtime" "${RUNTIME_TARGET}"
else
    print_warning "Inference runtime download was skipped."
fi

print_success "Go2 deployment source setup is complete."
print_info "Next: ./scripts/check_go2_real_environment.sh"
print_info "Then: ./scripts/build_go2_real.sh"
