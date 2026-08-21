#!/usr/bin/env bash

# Install the host packages needed for a standalone Unitree Go2 build.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

if [[ "$(uname -s)" != "Linux" ]] || ! command -v apt-get >/dev/null 2>&1; then
    print_error "This helper currently supports Debian/Ubuntu systems with apt-get."
    exit 1
fi

if [[ "${EUID}" -eq 0 ]]; then
    SUDO=()
elif command -v sudo >/dev/null 2>&1; then
    SUDO=(sudo)
else
    print_error "sudo is required when this script is not run as root."
    exit 1
fi

PACKAGES=(
    build-essential
    cmake
    curl
    g++
    git
    libboost-all-dev
    libeigen3-dev
    libfmt-dev
    liblcm-dev
    libspdlog-dev
    libtbb-dev
    libyaml-cpp-dev
    pkg-config
    python3-dev
    unzip
)

print_info "Installing standalone Go2 deployment build dependencies..."
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y "${PACKAGES[@]}"
print_success "System dependencies are installed."
