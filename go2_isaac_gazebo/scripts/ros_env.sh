#!/usr/bin/env bash

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_HINT="./build.sh robot_msgs robot_joint_controller go2_description rl_sar"

source_with_nounset_disabled() {
    local nounset_was_enabled=0

    case "$-" in
        *u*) nounset_was_enabled=1 ;;
    esac

    set +u
    source "$1"

    if [ "${nounset_was_enabled}" -eq 1 ]; then
        set -u
    fi
}

setup_ros_workspace() {
    cd "${PROJECT_ROOT}"

    conda deactivate 2>/dev/null || true
    export PATH="/usr/bin:/bin:/opt/ros/humble/bin:${PATH}"
    export PYTHONNOUSERSITE=1

    if [ ! -f /opt/ros/humble/setup.bash ]; then
        echo "ROS Humble setup not found: /opt/ros/humble/setup.bash" >&2
        return 1
    fi

    source_with_nounset_disabled /opt/ros/humble/setup.bash

    if [ ! -f install/setup.bash ]; then
        echo "Workspace is not built yet. Run: ${BUILD_HINT}" >&2
        return 1
    fi

    source_with_nounset_disabled install/setup.bash
}

require_file() {
    local path="$1"
    local message="$2"

    if [ ! -f "${PROJECT_ROOT}/${path}" ]; then
        echo "Missing ${path}" >&2
        echo "${message}" >&2
        return 1
    fi
}
