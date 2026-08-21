#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
source "${SCRIPT_DIR}/common.sh"

if [ "$(uname -m)" != "aarch64" ] || [ ! -f /etc/nv_tegra_release ]; then
    print_error "This builder is only for aarch64 Jetson systems"
    exit 1
fi

RUNTIME_ROOT="${1:-${PROJECT_ROOT}/library/inference_runtime}"
if [[ "$RUNTIME_ROOT" != /* ]]; then
    RUNTIME_ROOT="${PROJECT_ROOT}/${RUNTIME_ROOT}"
fi

ORT_VERSION="${ONNXRUNTIME_VERSION:-1.20.1}"
CMAKE_VERSION="${JETSON_CMAKE_VERSION:-3.30.5}"
BUILD_JOBS="${JETSON_BUILD_JOBS:-2}"
CC_BIN="${JETSON_CC:-gcc-10}"
CXX_BIN="${JETSON_CXX:-g++-10}"
EIGEN_COMMIT="e7248b26a1ed53fa030c5c459f7ea095dfd276ac"
EIGEN_ARCHIVE_SHA1="32b145f525a8308d7ab1c09388b2e288312d8eba"

if ! command -v "$CC_BIN" >/dev/null 2>&1 || ! command -v "$CXX_BIN" >/dev/null 2>&1; then
    print_error "GCC 10 is required for ONNX Runtime's ARM BFLOAT16 build check"
    print_info "Install it with: sudo apt-get install -y gcc-10 g++-10"
    exit 1
fi

CC_PATH="$(command -v "$CC_BIN")"
CXX_PATH="$(command -v "$CXX_BIN")"
COMPILER_TAG="$(basename "$CXX_PATH")"
SOURCE_DIR="${RUNTIME_ROOT}/src/onnxruntime-${ORT_VERSION}"
BUILD_DIR="${RUNTIME_ROOT}/build/onnxruntime-${ORT_VERSION}-jetson-${COMPILER_TAG}"
STAGE_DIR="${RUNTIME_ROOT}/onnxruntime.stage"
INSTALL_DIR="${RUNTIME_ROOT}/onnxruntime"
CMAKE_ROOT="${RUNTIME_ROOT}/tools/cmake-${CMAKE_VERSION}-linux-aarch64"
CMAKE_BIN="${CMAKE_ROOT}/bin/cmake"
EIGEN_ROOT="${RUNTIME_ROOT}/tools/eigen-${EIGEN_COMMIT}"

mkdir -p "$RUNTIME_ROOT" "$(dirname "$SOURCE_DIR")" "$(dirname "$BUILD_DIR")" "$(dirname "$CMAKE_ROOT")"

if [ ! -x "$CMAKE_BIN" ]; then
    CMAKE_ARCHIVE="cmake-${CMAKE_VERSION}-linux-aarch64.tar.gz"
    CMAKE_URL="https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${CMAKE_ARCHIVE}"
    CMAKE_TMP="${RUNTIME_ROOT}/${CMAKE_ARCHIVE}"
    print_info "Downloading local CMake ${CMAKE_VERSION} for aarch64..."
    curl -L --fail --progress-bar -o "$CMAKE_TMP" "$CMAKE_URL"
    tar -xzf "$CMAKE_TMP" -C "$(dirname "$CMAKE_ROOT")"
    rm -f "$CMAKE_TMP"
fi

if [ ! -f "${EIGEN_ROOT}/Eigen/Core" ]; then
    EIGEN_ARCHIVE="eigen-${EIGEN_COMMIT}.zip"
    EIGEN_URL="https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_COMMIT}/${EIGEN_ARCHIVE}"
    EIGEN_TMP="${RUNTIME_ROOT}/${EIGEN_ARCHIVE}"
    EIGEN_EXTRACT_DIR="${RUNTIME_ROOT}/eigen.extract"
    print_info "Downloading ONNX Runtime's pinned Eigen source..."
    curl -L --fail --progress-bar -o "$EIGEN_TMP" "$EIGEN_URL"
    printf '%s  %s\n' "$EIGEN_ARCHIVE_SHA1" "$EIGEN_TMP" | sha1sum --check --status || {
        print_error "Eigen archive checksum mismatch"
        rm -f "$EIGEN_TMP"
        exit 1
    }
    rm -rf "$EIGEN_EXTRACT_DIR" "$EIGEN_ROOT"
    mkdir -p "$EIGEN_EXTRACT_DIR"
    python3 -m zipfile -e "$EIGEN_TMP" "$EIGEN_EXTRACT_DIR"
    mv "${EIGEN_EXTRACT_DIR}/${EIGEN_ARCHIVE%.zip}" "$EIGEN_ROOT"
    rm -rf "$EIGEN_EXTRACT_DIR"
    rm -f "$EIGEN_TMP"
fi

if [ ! -d "${SOURCE_DIR}/.git" ]; then
    print_info "Cloning ONNX Runtime v${ORT_VERSION} with submodules..."
    git clone --branch "v${ORT_VERSION}" --depth 1 --recursive --shallow-submodules \
        https://github.com/microsoft/onnxruntime.git "$SOURCE_DIR"
else
    print_info "Reusing ONNX Runtime source at ${SOURCE_DIR}"
    git -C "$SOURCE_DIR" submodule update --init --recursive --depth 1
fi

print_info "Building ONNX Runtime ${ORT_VERSION} with cpuinfo disabled (${BUILD_JOBS} jobs)..."
print_info "Compiler: ${CXX_PATH}"
CC="$CC_PATH" CXX="$CXX_PATH" PATH="${CMAKE_ROOT}/bin:${PATH}" "${SOURCE_DIR}/build.sh" \
    --config Release \
    --update \
    --build \
    --build_shared_lib \
    --skip_tests \
    --parallel "$BUILD_JOBS" \
    --compile_no_warning_as_error \
    --use_preinstalled_eigen \
    --eigen_path "$EIGEN_ROOT" \
    --build_dir "$BUILD_DIR" \
    --cmake_extra_defines \
        onnxruntime_ENABLE_CPUINFO=OFF \
        CMAKE_DISABLE_FIND_PACKAGE_Eigen3=ON

LIB_DIR=""
while IFS= read -r candidate; do
    LIB_DIR="$(dirname "$candidate")"
    break
done < <(find "$BUILD_DIR" -type f -name "libonnxruntime.so.${ORT_VERSION}" | sort)

if [ -z "$LIB_DIR" ]; then
    print_error "Built libonnxruntime.so.${ORT_VERSION} was not found under ${BUILD_DIR}"
    exit 1
fi

rm -rf "$STAGE_DIR"
mkdir -p "$STAGE_DIR/include" "$STAGE_DIR/lib"

find "${SOURCE_DIR}/include/onnxruntime/core/session" -maxdepth 1 -type f \
    -name "*.h" -exec cp {} "$STAGE_DIR/include/" \;
if [ -f "${SOURCE_DIR}/include/onnxruntime/core/providers/cpu/cpu_provider_factory.h" ]; then
    cp "${SOURCE_DIR}/include/onnxruntime/core/providers/cpu/cpu_provider_factory.h" "$STAGE_DIR/include/"
fi
if [ -f "${SOURCE_DIR}/include/onnxruntime/core/providers/provider_options.h" ]; then
    cp "${SOURCE_DIR}/include/onnxruntime/core/providers/provider_options.h" "$STAGE_DIR/include/"
fi
cp -a "${LIB_DIR}"/libonnxruntime.so* "$STAGE_DIR/lib/"
printf '%s\n' "$ORT_VERSION" > "$STAGE_DIR/VERSION_NUMBER"
cat > "$STAGE_DIR/JETSON_CPUINFO_DISABLED" <<EOF
ONNX Runtime ${ORT_VERSION}
Built on $(date -u +%Y-%m-%dT%H:%M:%SZ)
onnxruntime_ENABLE_CPUINFO=OFF
EOF

if [ -d "$INSTALL_DIR" ]; then
    rm -rf "${INSTALL_DIR}.previous"
    mv "$INSTALL_DIR" "${INSTALL_DIR}.previous"
fi
mv "$STAGE_DIR" "$INSTALL_DIR"

if [ "${JETSON_KEEP_ORT_BUILD:-0}" != "1" ]; then
    print_info "Removing ONNX Runtime source/build cache after successful installation..."
    rm -rf "$SOURCE_DIR" "$BUILD_DIR" "$EIGEN_ROOT"
fi

print_success "Jetson-compatible ONNX Runtime ${ORT_VERSION} installed at ${INSTALL_DIR}"
