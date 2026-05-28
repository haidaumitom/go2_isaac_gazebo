# Go2 IsaacLab Gazebo ROS2 Policy Deployment

This workspace is a ROS2/Gazebo Classic simulation project for testing a Unitree Go2 reinforcement learning policy trained in IsaacLab before trying it on a real robot.

The project is based on `rl_sar`, but this workspace is focused on the Go2 pipeline:

- spawn a Go2 model in Gazebo Classic
- read simulated joint states and IMU data through ROS2
- run an IsaacLab policy with `rl_sar`
- publish low-level joint commands through `robot_joint_controller`
- log policy, state, IMU, and command topics to CSV for debugging

The current active policy is:

```text
policy/go2/isaaclab48/policy.pt
policy/go2/isaaclab48/config.yaml
```

## Workspace Layout

```text
.
├── policy/
│   └── go2/
│       ├── base.yaml                 # Go2 base control config
│       ├── himloco/                  # reference stable policy
│       ├── robot_lab/                # upstream Go2 policy config
│       └── isaaclab48/               # current IsaacLab policy
│           ├── config.yaml
│           └── policy.pt
├── scripts/
│   ├── run_gazebo_go2.sh             # start Gazebo with ROS2 environment
│   ├── run_rl_sim_go2.sh             # start the RL controller with ROS2 environment
│   └── log_policy_csv.py             # CSV logger for debugging runs
├── src/
│   ├── rl_sar/                       # RL controller, FSM, policy runtime
│   ├── robot_joint_controller/       # ROS2 controller bridge to Gazebo joints
│   ├── robot_msgs/                   # RobotState / RobotCommand messages
│   └── rl_sar_zoo/go2_description/   # Go2 URDF/xacro, Gazebo config, worlds
├── debug_logs/                       # generated CSV logs
├── build/
├── install/
└── README.md
```

## How This Repo Works

This repo is the deployment side of the pipeline. IsaacLab trains the policy; this workspace loads that trained policy and checks whether the same behavior survives in Gazebo Classic before moving toward the real Go2.

```text
IsaacLab training config
    ↓ defines observation/action contract
trained policy.pt
    ↓ copied into policy/go2/isaaclab48/
rl_sim
    ↓ builds IsaacLab-style observations from Gazebo state
policy inference
    ↓ action to joint target conversion
robot_joint_controller
    ↓ PD control into Gazebo joint effort commands
Gazebo Go2 model
```

## Main ROS2 Flow

```text
Gazebo Go2 model
    ↓ joint state / IMU
robot_joint_controller/state + /imu
    ↓
rl_sim
    ↓ policy observation
policy/go2/isaaclab48/policy.pt
    ↓ target joint positions
robot_joint_controller/command
    ↓
Gazebo joints
```

The important executable is:

```bash
ros2 run rl_sar rl_sim
```

It creates the FSM, loads the policy, receives robot state, builds the 48-dimensional IsaacLab observation, runs inference, and sends joint commands.

## Sim-To-Sim Contract

The goal of sim-to-sim is to make Gazebo look like IsaacLab from the policy's point of view. The policy only knows a vector of observations and produces a vector of actions, so these details must match training closely:

| IsaacLab training item | Gazebo deployment mirror |
| --- | --- |
| policy file exported from training | `policy/go2/isaaclab48/policy.pt` |
| observation order | `observations` in `policy/go2/isaaclab48/config.yaml` |
| observation size | `num_observations: 48` |
| action type: joint position action | `ComputeOutput()` in `rl_sdk.cpp` creates target joint positions |
| action scale `0.25` | `action_scale: [0.25, ...]` |
| policy rate: `dt=0.005`, `decimation=4` | `policy/go2/base.yaml` uses the same values |
| Go2 motor stiffness/damping | `rl_kp: 25.0`, `rl_kd: 0.5` |
| torque limit | `torque_limits: 23.5` |
| default joint pose | `default_dof_pos` |
| IsaacLab joint order | `joint_mapping` maps policy order to Gazebo joint order |

A mismatch in any row can make the policy stand badly, drift, walk in a curve, or fall even if it worked in IsaacLab.

## Requirements

This project is currently used with:

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic
- `colcon`
- `gazebo_ros2_control`
- `xacro`
- Python 3 from the system ROS installation

Install common ROS2 dependencies if needed:

```bash
sudo apt install \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-control-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-joy
```

## Build

From the workspace root:

```bash
cd ~/Projects/go2_isaac_gazebo

conda deactivate 2>/dev/null || true
export PATH=/usr/bin:/bin:/opt/ros/humble/bin:$PATH
export PYTHONNOUSERSITE=1

source /opt/ros/humble/setup.bash

colcon build --merge-install --symlink-install \
  --packages-select robot_msgs robot_joint_controller go2_description rl_sar
```

After building, source the workspace:

```bash
source install/setup.bash
```

You should rebuild when you change C++ files such as:

```text
src/rl_sar/library/core/rl_sdk/rl_sdk.cpp
src/rl_sar/fsm_robot/fsm_go2.hpp
src/robot_joint_controller/ros2/src/robot_joint_controller_group.cpp
```

You usually do not need to rebuild for YAML-only policy config changes, but you must restart `rl_sim` so it reloads the config.

## Run Gazebo And Policy

Use two terminals. The helper scripts below set the ROS2 environment for you and avoid copy-paste issues with invisible characters.

### Terminal 1: Gazebo

```bash
cd ~/Projects/go2_isaac_gazebo
./scripts/run_gazebo_go2.sh
```

### Terminal 2: RL Controller

```bash
cd ~/Projects/go2_isaac_gazebo
./scripts/run_rl_sim_go2.sh
```

Then use the keyboard inside the `rl_sim` terminal.

If you ever see `source: command not found` or `ros2: command not found` after copying commands, retype the command manually or run the scripts above. That error usually means an invisible Unicode character was pasted before `source` or `ros2`.

## Keyboard Control

Basic flow:

```text
0    get up into the default standing pose
1    enter the IsaacLab policy controller
W    increase forward command
S    decrease forward command
A    increase lateral-left command
D    increase lateral-right command
Q    increase yaw-left command
E    increase yaw-right command
Space reset x/y/yaw command to zero
R    reset Gazebo world
Enter pause/unpause simulation
```

Recommended first test:

```text
0
wait until get-up completes
1
confirm it says: RL Controller [isaaclab48] holding default pose
W
```

The current keyboard command step is `0.05`. The current forward/lateral command limit is `1.0`, and yaw is limited to `0.50`.

## IsaacLab Sources To Compare

For the current Go2 velocity policy, the important IsaacLab-side files are usually:

```text
/home/rinderudon/Projects/ws/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/velocity_env_cfg.py
/home/rinderudon/Projects/ws/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/go2/flat_env_cfg.py
/home/rinderudon/Projects/ws/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/go2/rough_env_cfg.py
/home/rinderudon/Projects/ws/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/unitree.py
```

Those files define the original robot asset, command ranges, observations, action scale, simulation timing, rewards, randomization, and terrain setup. This repo mirrors the parts needed for inference: observation order, action scale, default pose, joint order, gains, limits, and control rate.

## Current IsaacLab Policy Behavior

The active Go2 FSM loads:

```text
policy/go2/isaaclab48/config.yaml
policy/go2/isaaclab48/policy.pt
```

Important config fields:

```yaml
num_observations: 48
observations: ["lin_vel", "ang_vel", "gravity_vec", "commands", "dof_pos", "dof_vel", "actions"]
action_scale: [...]
default_dof_pos: [...]
joint_mapping: [...]
hold_zero_command: true
joint_pos_limit_lower: [...]
joint_pos_limit_upper: [...]
```

`hold_zero_command: true` means pressing `1` will hold the clean default pose while `x/y/yaw` are zero. The policy starts acting once a nonzero command is given, for example by pressing `W`.

The joint position limits are used to keep policy targets inside the Gazebo Go2 URDF limits. This was added because the policy often commanded calf targets beyond the Gazebo calf joint upper limit, which can create asymmetric tracking and curved walking.

## Replace The Policy

To test a new IsaacLab policy:

1. Export the policy as a TorchScript `.pt` file.
2. Put it here:

```text
policy/go2/isaaclab48/policy.pt
```

3. Make sure `policy/go2/isaaclab48/config.yaml` matches the training contract:

```text
observation order
observation size
joint order
default joint positions
action scale
command scale
joint limits
```

4. Restart `rl_sim`.

If only the `.pt` or YAML changed, rebuilding is usually not needed.

## CSV Logging

Use a third terminal while Gazebo and `rl_sim` are running:

```bash
cd ~/Projects/go2_isaac_gazebo

conda deactivate 2>/dev/null || true
export PATH=/usr/bin:/bin:/opt/ros/humble/bin:$PATH
export PYTHONNOUSERSITE=1

source /opt/ros/humble/setup.bash
source install/setup.bash

/usr/bin/python3 scripts/log_policy_csv.py
```

Stop with `Ctrl+C`. Logs are saved in:

```text
debug_logs/
```

The CSV logger is useful for checking:

- commanded joint position vs actual joint position
- joint tracking error
- IMU roll/pitch/yaw
- yaw drift while commanding straight walking
- whether a leg is stuck or saturating

## Common Debug Checklist

If the robot stands badly after pressing `1`:

- confirm `hold_zero_command: true`
- confirm terminal shows `holding default pose`
- check `default_dof_pos`
- check `joint_mapping`

If the robot walks in a curve:

- check yaw command is zero
- check lateral command `y` is zero
- inspect latest CSV for left/right joint tracking mismatch
- check calf targets are not constantly hitting joint limits
- reduce calf `action_scale` if needed

If the robot falls immediately:

- restart Gazebo and `rl_sim`
- press `0` and wait for get-up to finish before pressing `1`
- check that the correct policy is loaded
- rebuild if C++ or xacro files changed

## Notes

This workspace is still a sim-to-sim deployment environment. Passing Gazebo does not mean the policy is ready for the real Go2. Before real robot deployment, re-check:

- action scaling
- joint order
- observation order
- command scaling
- control frequency
- joint limits
- torque limits
- emergency stop behavior
