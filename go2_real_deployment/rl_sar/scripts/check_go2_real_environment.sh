#!/usr/bin/env bash

# Read-only checks for the standalone Unitree Go2 build environment.

set -u
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "${SCRIPT_DIR}")"
source "${SCRIPT_DIR}/common.sh"

SDK_RELATIVE_PATH="src/rl_sar/library/thirdparty/robot_sdk/unitree/unitree_sdk2"
SDK_PATH="${PROJECT_ROOT}/${SDK_RELATIVE_PATH}"
INFERENCE_RUNTIME_DIR="${RL_SAR_INFERENCE_RUNTIME_DIR:-${PROJECT_ROOT}/library/inference_runtime}"
POLICY_DIR="${RL_SAR_POLICY_DIR:-${PROJECT_ROOT}/policy}"
PYTHON_EXECUTABLE="${RL_SAR_PYTHON_EXECUTABLE:-/usr/bin/python3}"
INFERENCE_BACKEND="${RL_SAR_INFERENCE_BACKEND:-AUTO}"
INFERENCE_BACKEND="${INFERENCE_BACKEND^^}"
NETWORK_INTERFACE=""
ERROR_COUNT=0

usage() {
    cat <<'EOF'
Usage: ./scripts/check_go2_real_environment.sh [--interface <name>]

The interface check is optional during compilation. It is required before any
future hardware run, for example: --interface enp3s0
EOF
}

pass() {
    print_success "$1"
}

fail() {
    print_error "$1"
    ERROR_COUNT=$((ERROR_COUNT + 1))
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --interface)
            if [[ $# -lt 2 ]]; then
                print_error "--interface requires a network interface name."
                exit 2
            fi
            NETWORK_INTERFACE="$2"
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

print_header "Go2 real deployment environment check"

case "${INFERENCE_BACKEND}" in
    AUTO|TORCH|ONNX)
        pass "Requested inference backend: ${INFERENCE_BACKEND}"
        ;;
    *)
        fail "RL_SAR_INFERENCE_BACKEND must be AUTO, TORCH, or ONNX."
        ;;
esac

if [[ "$(uname -s)" == "Linux" ]]; then
    pass "Host operating system is Linux."
else
    fail "The real Go2 backend is supported only on Linux."
fi

case "$(uname -m)" in
    x86_64|amd64|aarch64|arm64)
        pass "Host architecture $(uname -m) is supported."
        ;;
    *)
        fail "Unsupported host architecture: $(uname -m)."
        ;;
esac

for command_name in cmake c++ git sha256sum; do
    if command -v "${command_name}" >/dev/null 2>&1; then
        pass "Found ${command_name}: $(command -v "${command_name}")"
    else
        fail "Missing required command: ${command_name}"
    fi
done

if [[ -x "${PYTHON_EXECUTABLE}" ]]; then
    pass "Found Python interpreter: ${PYTHON_EXECUTABLE} ($("${PYTHON_EXECUTABLE}" --version 2>&1))"
else
    fail "Python interpreter is not executable: ${PYTHON_EXECUTABLE}"
fi

if [[ -f "${SDK_PATH}/CMakeLists.txt" ]]; then
    EXPECTED_SDK_COMMIT="$(git -C "${PROJECT_ROOT}" ls-tree HEAD "${SDK_RELATIVE_PATH}" 2>/dev/null | awk '{print $3}')"
    ACTUAL_SDK_COMMIT="$(git -C "${SDK_PATH}" rev-parse HEAD 2>/dev/null || true)"
    if [[ -n "${EXPECTED_SDK_COMMIT}" && "${ACTUAL_SDK_COMMIT}" == "${EXPECTED_SDK_COMMIT}" ]]; then
        pass "Unitree SDK2 is initialized at the pinned commit ${ACTUAL_SDK_COMMIT}."
    else
        fail "Unitree SDK2 commit does not match the repository gitlink."
    fi
else
    fail "Unitree SDK2 is missing. Run ./scripts/setup_go2_real.sh."
fi

LIBTORCH_READY=false
ONNX_READY=false
if [[ -f "${INFERENCE_RUNTIME_DIR}/libtorch/share/cmake/Torch/TorchConfig.cmake" \
      && -e "${INFERENCE_RUNTIME_DIR}/libtorch/lib/libtorch.so" \
      && -e "${INFERENCE_RUNTIME_DIR}/libtorch/lib/libc10.so" ]]; then
    LIBTORCH_READY=true
fi
if [[ -f "${INFERENCE_RUNTIME_DIR}/onnxruntime/include/onnxruntime_cxx_api.h" \
      && -e "${INFERENCE_RUNTIME_DIR}/onnxruntime/lib/libonnxruntime.so" ]]; then
    ONNX_READY=true
fi

if [[ "${LIBTORCH_READY}" == true || "${ONNX_READY}" == true ]]; then
    READY_RUNTIMES=()
    [[ "${LIBTORCH_READY}" == true ]] && READY_RUNTIMES+=(LibTorch)
    [[ "${ONNX_READY}" == true ]] && READY_RUNTIMES+=(ONNX_Runtime)
    pass "Inference runtime ready at ${INFERENCE_RUNTIME_DIR}: ${READY_RUNTIMES[*]}"
else
    fail "No usable inference runtime found at ${INFERENCE_RUNTIME_DIR}."
fi

if [[ "${INFERENCE_BACKEND}" == "TORCH" && "${LIBTORCH_READY}" != true ]]; then
    fail "TORCH was selected, but LibTorch is not usable in ${INFERENCE_RUNTIME_DIR}."
elif [[ "${INFERENCE_BACKEND}" == "ONNX" && "${ONNX_READY}" != true ]]; then
    fail "ONNX was selected, but ONNX Runtime is not usable in ${INFERENCE_RUNTIME_DIR}."
fi

if [[ -f "${POLICY_DIR}/go2/base.yaml" ]]; then
    pass "Found Go2 base policy configuration in ${POLICY_DIR}."
    print_info "Go2 base config SHA-256: $(sha256sum "${POLICY_DIR}/go2/base.yaml" | awk '{print $1}')"
else
    fail "Missing ${POLICY_DIR}/go2/base.yaml."
fi

ACTIVE_PROFILE="${RL_SAR_POLICY_CONFIG_NAME:-}"
if [[ -z "${ACTIVE_PROFILE}" && -f "${POLICY_DIR}/go2/base.yaml" ]]; then
    ACTIVE_PROFILE="$(awk -F'"' '/^[[:space:]]*policy_config_name:/ {print $2; exit}' "${POLICY_DIR}/go2/base.yaml")"
fi
ACTIVE_PROFILE="${ACTIVE_PROFILE:-himloco}"
ACTIVE_CONFIG="${POLICY_DIR}/go2/${ACTIVE_PROFILE}/config.yaml"
if [[ -f "${ACTIVE_CONFIG}" ]]; then
    pass "Found selected Go2 profile: ${ACTIVE_PROFILE}."
    MODEL_NAME="$(awk -F'"' '/^[[:space:]]*model_name:/ {print $2; exit}' "${ACTIVE_CONFIG}")"
    if [[ -z "${MODEL_NAME}" ]]; then
        fail "Could not read model_name from ${ACTIVE_CONFIG}."
    else
        ACTIVE_MODEL="${POLICY_DIR}/go2/${ACTIVE_PROFILE}/${MODEL_NAME}"
        if [[ -f "${ACTIVE_MODEL}" ]]; then
            pass "Found selected model: ${ACTIVE_MODEL}"
            print_info "Policy config SHA-256: $(sha256sum "${ACTIVE_CONFIG}" | awk '{print $1}')"
            print_info "Policy model SHA-256:  $(sha256sum "${ACTIVE_MODEL}" | awk '{print $1}')"
            case "${MODEL_NAME}" in
                *.pt)
                    [[ "${LIBTORCH_READY}" == true ]] || fail "The selected .pt model requires LibTorch."
                    [[ "${INFERENCE_BACKEND}" != "ONNX" ]] || fail "The selected .pt model is incompatible with the ONNX backend."
                    ;;
                *.onnx)
                    [[ "${ONNX_READY}" == true ]] || fail "The selected .onnx model requires ONNX Runtime."
                    [[ "${INFERENCE_BACKEND}" != "TORCH" ]] || fail "The selected .onnx model is incompatible with the TORCH backend."
                    ;;
                *) fail "Unsupported selected model extension: ${MODEL_NAME}" ;;
            esac
        else
            fail "Selected model does not exist: ${ACTIVE_MODEL}"
        fi
    fi
else
    fail "Missing selected policy config: ${ACTIVE_CONFIG}"
fi

if [[ -n "${NETWORK_INTERFACE}" ]]; then
    if [[ -d "/sys/class/net/${NETWORK_INTERFACE}" ]]; then
        pass "Network interface ${NETWORK_INTERFACE} exists."
        OPERSTATE="$(<"/sys/class/net/${NETWORK_INTERFACE}/operstate")"
        if [[ "${OPERSTATE}" != "up" && "${OPERSTATE}" != "unknown" ]]; then
            print_warning "Interface ${NETWORK_INTERFACE} state is ${OPERSTATE}; it must be usable before deployment."
        fi
    else
        fail "Network interface ${NETWORK_INTERFACE} does not exist."
    fi
else
    print_warning "Network interface was not checked (not needed for compilation)."
fi

if [[ -z "$(git -C "${PROJECT_ROOT}" status --porcelain --untracked-files=all 2>/dev/null)" ]]; then
    pass "Git checkout is clean."
else
    print_warning "Git checkout has modified or untracked files; commit them before recording an experiment."
fi
if [[ -d "${SDK_PATH}/.git" || -f "${SDK_PATH}/.git" ]]; then
    if [[ -z "$(git -C "${SDK_PATH}" status --porcelain --untracked-files=all 2>/dev/null)" ]]; then
        pass "Unitree SDK2 worktree is clean."
    else
        print_warning "Unitree SDK2 worktree is dirty; do not use it for a recorded experiment."
    fi
fi

print_info "Repository commit: $(git -C "${PROJECT_ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)"
print_info "Branch: $(git -C "${PROJECT_ROOT}" branch --show-current 2>/dev/null || echo unknown)"

if [[ "${ERROR_COUNT}" -ne 0 ]]; then
    print_error "Environment check failed with ${ERROR_COUNT} problem(s)."
    exit 1
fi

print_success "Environment is ready to compile rl_real_go2."
