# Go2 Real Policy Deployment

This workspace runs approved reinforcement-learning policies directly on the
Go2 onboard Jetson. The Jetson owns policy inference and the 200 Hz Unitree
`LowCmd` loop. A laptop is used for synchronization, SSH, monitoring, and
emergency supervision.

Training belongs in IsaacLab. Gazebo validation belongs in
`go2_isaac_gazebo`. Do not use this workspace as the first test of a newly
exported model.

## Included Go2 Policies

| Name | Contract | Status |
| --- | --- | --- |
| `model_2999` | 45 observations, 5-frame two-input history | Experimental; harness validation required |
| `adaptive_fault_d2` | 45 observations, 30-frame flattened history | Experimental adaptive policy |
| `policy_history` | 45 observations, 30-frame two-input history | Experimental history policy |
| `fault_history_b5` | 45 observations, 30-frame flattened history | Guarded tucked-leg experiment |

The runtime also includes model/config validators, embedded hand-controller
decoding, stale-state checks, a read-only DDS probe, and prone-first shutdown
on `Ctrl+C`.

## Clone And Initialize

From the combined repository root:

```bash
git submodule update --init --recursive \
  go2_real_deployment/rl_sar/src/rl_sar/library/thirdparty/robot_sdk/unitree/unitree_sdk2

cd go2_real_deployment/rl_sar
```

Or use the workspace helper:

```bash
./scripts/setup_go2_real.sh --runtime onnx
```

## Build On The Jetson

```bash
cd ~/go2_real_deployment/rl_sar
RL_SAR_POLICY_CONFIG_NAME=model_2999 \
RL_SAR_INFERENCE_BACKEND=ONNX \
./scripts/build_go2_real.sh --clean --jobs 2
```

If the previously tested Jetson runtime is retained elsewhere:

```bash
RL_SAR_POLICY_CONFIG_NAME=model_2999 \
RL_SAR_INFERENCE_BACKEND=ONNX \
RL_SAR_INFERENCE_RUNTIME_DIR=$HOME/go2_isaac_gazebo/library/inference_runtime \
./scripts/build_go2_real.sh --clean --jobs 2
```

## Required Preflight

Run on the Jetson and move the hand-controller sticks during the probe:

```bash
cd ~/go2_real_deployment/rl_sar
./install/go2_real/bin/validate_policy_config go2 model_2999
./install/go2_real/bin/go2_state_probe eth0 10
echo "exit_code=$?"
```

Continue only after:

```text
PASS: LowState and embedded hand-controller data are live.
```

## Harnessed Policy Test

```bash
cd ~/go2_real_deployment/rl_sar
unset RL_SAR_ENABLE_REAL_TUCK_TEST
unset RL_SAR_ENABLE_REAL_GAIN_FAULT_TEST
RL_SAR_POLICY_CONFIG_NAME=model_2999 \
./install/go2_real/bin/rl_real_go2 eth0
```

Change only the policy name to run another included bundle:

```bash
RL_SAR_POLICY_CONFIG_NAME=adaptive_fault_d2 ./install/go2_real/bin/rl_real_go2 eth0
RL_SAR_POLICY_CONFIG_NAME=policy_history ./install/go2_real/bin/rl_real_go2 eth0
RL_SAR_POLICY_CONFIG_NAME=fault_history_b5 ./install/go2_real/bin/rl_real_go2 eth0
```

## Controller

```text
A                 get up
RB + D-pad Up     enter policy locomotion
Left stick Y      forward/backward
Left stick X      lateral
Right stick X     yaw
B                 get down
```

Begin with centered sticks and an overhead harness. Confirm stable standing
before applying a small command. Do not inject a fault during a model's first
hardware validation.

Use `Ctrl+C` for normal shutdown. Require these messages before approaching the
joints:

```text
Safe shutdown: moving to prone before releasing motor stiffness.
Controlled prone posture reached.
RL_Real safe shutdown complete
```

More implementation details are in `docs/go2_real_deployment.md` and
`docs/onboard_go2_deployment_checkpoint.md`.

This project uses and modifies the Apache-2.0 licensed `rl_sar` runtime by
Ziqi Fan. Original copyright and license notices are retained.
