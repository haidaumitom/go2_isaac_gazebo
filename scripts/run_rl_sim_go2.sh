#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

conda deactivate 2>/dev/null || true
export PATH=/usr/bin:/bin:/opt/ros/humble/bin:${PATH}
export PYTHONNOUSERSITE=1

source /opt/ros/humble/setup.bash

if [ ! -f install/setup.bash ]; then
    echo "Workspace is not built yet. Run: ./build.sh robot_msgs robot_joint_controller go2_description rl_sar" >&2
    exit 1
fi

source install/setup.bash
exec ros2 run rl_sar rl_sim "$@"
