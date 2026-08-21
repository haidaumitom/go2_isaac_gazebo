#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/ros_env.sh"

require_file \
    "policy/go2/ftnet_contact_history/policy.pt" \
    "Place your exported TorchScript FTNet policy at that path first."

export RL_SAR_POLICY_CONFIG_NAME="${RL_SAR_POLICY_CONFIG_NAME:-ftnet_contact_history}"
exec "${PROJECT_ROOT}/scripts/run_rl_sim_go2.sh" "$@"
