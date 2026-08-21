# Go2 Real Deployment

This is the standalone real-robot deployment workspace. IsaacLab training and
Gazebo validation remain separate. The onboard Jetson runs inference and the
200 Hz Unitree `LowCmd` loop; the laptop provides synchronization, SSH,
monitoring, and emergency supervision.

## Included Runtime

The restored runtime provides runtime policy selection, single-frame and
history-model inference, policy-contract validators, embedded controller
decoding, stale-state checks, a read-only DDS probe, and controlled prone-first
shutdown on `Ctrl+C`.

| Policy | Contract |
| --- | --- |
| `model_2999` | 45 observations, 5-frame two-input history |
| `adaptive_fault_d2` | 45 observations, 30-frame flattened history |
| `policy_history` | 45 observations, 30-frame two-input history |
| `fault_history_b5` | 45 observations, 30-frame flattened history |

`model_2999` is the default, but select a policy explicitly for every run.

## Laptop Build

```bash
cd ~/ws/go2_real_deployment/rl_sar
RL_SAR_POLICY_CONFIG_NAME=model_2999 \
RL_SAR_INFERENCE_BACKEND=ONNX \
./scripts/build_go2_real.sh --clean --jobs 2

./install/go2_real/bin/validate_policy_config go2 model_2999
```

Installed tools:

```text
install/go2_real/bin/rl_real_go2
install/go2_real/bin/go2_state_probe
install/go2_real/bin/validate_policy_model
install/go2_real/bin/validate_policy_config
```

Build success and contract validation do not authorize a hardware run.

## Synchronize To Jetson

Run on the laptop:

```bash
rsync -az \
  --exclude='.git/' \
  --exclude='cmake_build/' \
  --exclude='install/' \
  --exclude='library/inference_runtime/' \
  ~/ws/go2_real_deployment/rl_sar/ \
  unitree@192.168.0.192:~/go2_real_deployment/rl_sar/

ssh unitree@192.168.0.192
```

The x86 laptop runtime must not be copied to the aarch64 Jetson. The command
intentionally omits `--delete`.

## Jetson Build

Run on the Jetson:

```bash
cd ~/go2_real_deployment/rl_sar
```

Reuse the already tested Jetson runtime from the previous workspace:

```bash
RL_SAR_POLICY_CONFIG_NAME=model_2999 \
RL_SAR_INFERENCE_BACKEND=ONNX \
RL_SAR_INFERENCE_RUNTIME_DIR=$HOME/go2_isaac_gazebo/library/inference_runtime \
./scripts/build_go2_real.sh --clean --jobs 2
```

If that runtime no longer exists, build a Jetson-compatible one locally:

```bash
bash scripts/download_inference_runtime.sh onnx
RL_SAR_POLICY_CONFIG_NAME=model_2999 \
RL_SAR_INFERENCE_BACKEND=ONNX \
./scripts/build_go2_real.sh --clean --jobs 2
```

The Jetson helper builds ONNX Runtime 1.20.1 with CPU topology discovery
disabled, matching the previously validated Orin NX setup.

## Required Preflight

Run on the Jetson:

```bash
cd ~/go2_real_deployment/rl_sar
./install/go2_real/bin/validate_policy_config go2 model_2999
./install/go2_real/bin/go2_state_probe eth0 10
echo "exit_code=$?"
```

Move the hand-controller sticks during the probe. Continue only when it says:

```text
PASS: LowState and embedded hand-controller data are live.
```

Do not run a policy with stale LowState, IMU, joint, or controller data.

## Harnessed Test

Keep the Go2 on its overhead harness. Run on the Jetson:

```bash
cd ~/go2_real_deployment/rl_sar
unset RL_SAR_ENABLE_REAL_TUCK_TEST
unset RL_SAR_ENABLE_REAL_GAIN_FAULT_TEST
RL_SAR_POLICY_CONFIG_NAME=model_2999 \
./install/go2_real/bin/rl_real_go2 eth0
```

Controller:

```text
A                 get up
RB + D-pad Up     enter policy locomotion
Left stick Y      forward/backward
Left stick X      lateral
Right stick X     yaw
B                 get down
```

Begin with centered sticks. Confirm stable standing, then use only a small
command. Do not inject a fault during a new model's first hardware validation.
Use `Ctrl+C` for normal shutdown and require:

```text
Safe shutdown: moving to prone before releasing motor stiffness.
Controlled prone posture reached.
RL_Real safe shutdown complete
```

Keep the harness supporting the robot until the process exits. Never force a
stiff joint by hand.

## Other Policies

Use the same validation, probe, and run sequence, changing only the name:

```bash
RL_SAR_POLICY_CONFIG_NAME=adaptive_fault_d2 ./install/go2_real/bin/rl_real_go2 eth0
RL_SAR_POLICY_CONFIG_NAME=policy_history ./install/go2_real/bin/rl_real_go2 eth0
RL_SAR_POLICY_CONFIG_NAME=fault_history_b5 ./install/go2_real/bin/rl_real_go2 eth0
```

Guarded fault injection is a later, separately reviewed experiment and is not
part of normal deployment.

Before each experiment record the Git commit, policy name, model SHA-256,
runtime version, SDK2 commit, interface, firmware, and control configuration.
The source base is upstream `376d42c9b128f963ab08579762d5a216a976ce39`.
