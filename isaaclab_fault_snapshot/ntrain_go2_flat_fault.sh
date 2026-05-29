#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

source /home/rinderudon/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab

RUN_TIME="${1:-30m}"
NUM_ENVS="${2:-512}"
MAX_ITERS="${3:-300}"
TASK="Isaac-Velocity-Flat-Unitree-Go2-Fault-v0"

echo "Training Go2 flat fault task"
echo "Runtime=$RUN_TIME NumEnvs=$NUM_ENVS MaxIterations=$MAX_ITERS"
echo "Task=$TASK"

TERM=xterm timeout "$RUN_TIME" ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task "$TASK" \
  --num_envs "$NUM_ENVS" \
  --max_iterations "$MAX_ITERS" \
  --headless
