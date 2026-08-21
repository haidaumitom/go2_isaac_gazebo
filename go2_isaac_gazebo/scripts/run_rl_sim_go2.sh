#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/ros_env.sh"
setup_ros_workspace
exec ros2 run rl_sar rl_sim "$@"
