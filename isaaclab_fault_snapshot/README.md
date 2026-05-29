# IsaacLab Go2 Fault Training Snapshot

This folder is a source-code snapshot from the local IsaacLab workspace used for the current Go2 broken-joint / weak-actuator training experiment.

The live IsaacLab workspace is:

```text
/home/rinderudon/Projects/ws/IsaacLab
```

The active training run checked on 2026-05-29 is:

```text
logs/rsl_rl/go2_broken_joints/2026-05-29_15-14-46_continue_fault_8_15_from_223800
```

Latest checkpoint seen at snapshot time:

```text
model_262450.pt
```

Checkpoints and TensorBoard logs are not included here because they are generated training outputs and can grow quickly. This folder is only meant to show the experiment code/configuration.

Important entry points:

```text
ntrain_go2_flat_fault.sh
nplay_go2_flat_fault.sh
source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/go2/fault_env_cfg.py
source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/go2/fault_actuator.py
```

The fault model scales commanded actuator torque before applying it:

```text
tau_actual = alpha * tau_command
```

where `alpha = 1.0` is healthy and lower values represent weak or broken joints.
