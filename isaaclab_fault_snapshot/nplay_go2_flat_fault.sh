#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

source /home/rinderudon/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab

NUM_ENVS="${1:-32}"
EXP_DIR="logs/rsl_rl/unitree_go2_flat_fault"
TASK="Isaac-Velocity-Flat-Unitree-Go2-Fault-Play-v0"

target="$(find "$EXP_DIR" -mindepth 2 -maxdepth 2 -name "model_*.pt" -printf "%T@ %p\n" | sort -nr | sed -n '1p')"

if [[ -z "$target" ]]; then
  echo "No fault checkpoint found under $EXP_DIR"
  echo "Train first with ./ntrain_go2_flat_fault.sh"
  exit 1
fi

checkpoint="$(awk '{print $2}' <<< "$target")"

echo "Playing latest Go2 flat fault checkpoint:"
echo "$checkpoint"
echo "NumEnvs=$NUM_ENVS"
echo "Task=$TASK"
echo "Close the Isaac Sim window to stop playback."

TERM=xterm ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task "$TASK" \
  --num_envs "$NUM_ENVS" \
  --checkpoint "$checkpoint" \
  --real-time
