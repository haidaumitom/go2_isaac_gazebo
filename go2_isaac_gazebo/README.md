# Go2 IsaacLab Policies in Gazebo

This workspace validates exported Go2 reinforcement-learning policies in
Gazebo through ROS 2 before any real-robot deployment. IsaacLab training and
physical Go2 deployment are maintained in separate workspaces.

## Included Policies

| Name | Contract | Purpose |
| --- | --- | --- |
| `model_2999` | 45 observations, 5-frame two-input history | Quadloco/CENet history actor |
| `equivgcn` | 45 observations, 30-frame two-input history | Equivariant GCN fault-adaptive actor |
| `fault_history_b5` | 45 observations, 30-frame flattened history | Scripted tucked-leg experiment |
| `adaptive_fault_d2` | 45 observations, 30-frame flattened history | Residual actuator-loss adaptation |

`policy/go2/base.yaml` defaults to `model_2999`. Select every experiment
explicitly with `RL_SAR_POLICY_CONFIG_NAME` so the terminal log records the
policy that was run.

## Build

```bash
cd ~/ws/go2_isaac_gazebo
source /opt/ros/humble/setup.bash

colcon build --merge-install --symlink-install \
  --packages-select robot_msgs robot_joint_controller rl_sar \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## Start Gazebo

Terminal 1:

```bash
cd ~/ws/go2_isaac_gazebo
./scripts/run_gazebo_go2.sh
```

Wait for the Go2 model and joint controllers to finish spawning.

## Run A Policy

Open Terminal 2 and run exactly one command:

```bash
cd ~/ws/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=model_2999 ./scripts/run_rl_sim_go2.sh
```

```bash
cd ~/ws/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=equivgcn ./scripts/run_rl_sim_go2.sh
```

```bash
cd ~/ws/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=fault_history_b5 ./scripts/run_rl_sim_go2.sh
```

```bash
cd ~/ws/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=adaptive_fault_d2 ./scripts/run_rl_sim_go2.sh
```

## Keyboard Controls

Use the policy terminal:

```text
0      get up
1      enter policy locomotion
9      get down
P      enter passive immediately
W/S    increase/decrease forward command
A/D    increase/decrease lateral command
Q/E    increase/decrease yaw command
Space  reset velocity commands to zero
```

Normal sequence:

```text
press 0 and wait for get-up to reach 100%
press 1 and confirm stable standing
apply small commands first
press 9 before ending the simulation
```

## Validate A Policy

After building, validate the model/YAML contract before running:

```bash
cd ~/ws/go2_isaac_gazebo
./install/lib/rl_sar/validate_policy_config go2 model_2999
./install/lib/rl_sar/validate_policy_config go2 equivgcn
./install/lib/rl_sar/validate_policy_config go2 fault_history_b5
./install/lib/rl_sar/validate_policy_config go2 adaptive_fault_d2
```

Validation checks observation shape, history layout, model input count, and the
12-action output. It does not replace visual Gazebo testing.

## Workspace Boundary

```text
IsaacLab workspace
    training and checkpoint export
            |
            v
go2_isaac_gazebo
    ROS 2 and Gazebo validation
            |
            v
go2_real_deployment/rl_sar
    onboard Jetson and physical Go2 deployment
```

This project is based on and modifies the Apache-2.0 licensed `rl_sar` runtime
by Ziqi Fan. Original copyright and license notices are retained.
