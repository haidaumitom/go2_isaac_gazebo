# Onboard Go2 Deployment Checkpoint

Last updated: 2026-08-18

## Goal

Run normal and observation-history policies directly on the Go2's onboard
Jetson. The laptop connects over Wi-Fi/SSH for supervision, while the Jetson
uses `eth0` for the robot's internal Unitree DDS network.

## Onboard System

- SSH: `unitree@192.168.0.192`
- Jetson internal robot interface: `eth0` (`192.168.123.18`)
- Ubuntu 20.04, aarch64, JetPack/L4T R35.3.1
- Four CPU cores online, 15 GiB RAM
- Workspace: `~/go2_real_deployment/rl_sar`

## Completed

- Installed GCC/G++ 10 and the required native build dependencies.
- Built ONNX Runtime 1.20.1 on the Jetson with CPU topology discovery disabled.
- Confirmed `JETSON_CPUINFO_DISABLED` contains
  `onnxruntime_ENABLE_CPUINFO=OFF`.
- Built the standalone `rl_sar` runtime with `USE_ONNX=ON`.
- Validated `model_2999` as a 45-value, five-frame two-input history policy.
- Validated `fault_history_b5.onnx`: 30 x 45 history to 12 actions.
- Validated both complete YAML/model contracts.
- Ran the read-only DDS probe on `eth0`:
  - LowState was live at about 500 Hz.
  - IMU quaternion norm was 1.0.
  - Joint positions and foot forces were valid.
  - Embedded hand-controller axes covered their full range.
  - The probe reported PASS and exited cleanly.

## Shutdown Safety Fix

Stopping `rl_real_go2` with `Ctrl+C` previously closed the publisher immediately.
The Go2 then retained the last policy joint targets and nonzero gains, leaving
the joints hard-locked until reboot.

The runtime now performs an idempotent prone-first shutdown:

- stop keyboard input and policy inference while leaving the 200 Hz control
  loop active;
- command zero base velocity and ignore new controller mode requests;
- run the existing two-second `RLFSMStateGetDown` position trajectory back to
  the pose recorded before get-up;
- wait until the FSM reaches `RLFSMStatePassive`;
- hold the measured prone pose while ramping position stiffness and
  feed-forward torque down;
- enter a damping-only phase;
- ramp damping to zero;
- repeatedly publish Unitree's zero-gain `PosStopF`/`VelStopF` command with
  motor mode `0x00` (stop/standby) before closing DDS.

If LowState is stale or the prone transition does not finish within five
seconds, shutdown uses the gradual motor-release sequence as a fallback.

The shared loop helper no longer detaches its worker thread, so `shutdown()`
now waits for each loop to finish before the final motor-release commands are
published.

The changes compile successfully on the laptop and the onboard Jetson. Physical
verification still requires the overhead harness.

Files involved:

- `src/rl_sar/src/rl_real_go2.cpp`
- `src/rl_sar/include/rl_real_go2.hpp`
- `src/rl_sar/fsm_robot/fsm_go2.hpp`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp`
- `src/rl_sar/library/core/loop/loop.hpp`
- `policy/go2/base.yaml`

## Verify Shutdown

On the Jetson, the updated binary is built with:

```bash
cd ~/go2_real_deployment/rl_sar
cmake --build cmake_build --target rl_real_go2 -j2
```

First verify shutdown without standing or entering policy mode:

```bash
RL_SAR_POLICY_CONFIG_NAME=adaptive_fault_d2 \
  ./cmake_build/bin/rl_real_go2 eth0
```

Leave the FSM in its initial passive state and press `Ctrl+C`. Required output:

```text
Safe shutdown: moving to prone before releasing motor stiffness.
Controlled prone posture reached.
RL_Real safe shutdown complete
```

Then test on the harness after get-up and policy entry. Press `Ctrl+C` while the
robot is standing. The console must show the `Getting down` progress reaching
100% before `Controlled prone posture reached.` Confirm that the robot is prone
before the gains are released.

Check the process result:

```bash
echo "exit_code=$?"
```

## Hardware Test Order

1. Corrected read-only DDS probe exits cleanly.
2. Test passive-only `Ctrl+C` shutdown on the overhead harness.
3. Test `adaptive_fault_d2` get-up, policy entry, and prone-first `Ctrl+C` shutdown.
4. Test `fault_history_b5` normally, with no injected fault.
5. Only after all normal tests pass, run a guarded fault experiment.

Normal-policy command, when cleared to proceed:

```bash
cd ~/go2_real_deployment/rl_sar
unset RL_SAR_ENABLE_REAL_TUCK_TEST
RL_SAR_POLICY_CONFIG_NAME=adaptive_fault_d2 \
  ./cmake_build/bin/rl_real_go2 eth0
```

Guarded fault testing must remain the final step and must use the overhead
harness. Do not skip the normal-policy checks.
