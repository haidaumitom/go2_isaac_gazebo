# Go2 IsaacLab Policy Deployment in Gazebo

This workspace runs Unitree Go2 policies exported from IsaacLab inside Gazebo Classic through ROS2.

The repo is the deployment and validation side of the workflow:

```text
IsaacLab training
    -> exported TorchScript policy.pt
    -> this ROS2/Gazebo workspace
    -> rl_sar policy runner
    -> robot_joint_controller
    -> Gazebo Go2 joints
```

The main goal is to check that an IsaacLab policy still behaves correctly when its observations, actions, joint order, gains, and timing are reproduced in Gazebo.


## Repository Layout

```text
.
|-- build.sh                         # workspace build helper
|-- docs/                            # focused debug notes
|-- policy/go2/
|   |-- base.yaml                    # shared Go2 deployment config
|   |-- isaaclab48/                  # default IsaacLab baseline policy
|   |-- ftnet_contact_history/       # experimental history/contact policy
|   |-- fault_adaptive/              # experimental recovery policy
|   |-- fault_adaptive_180800/
|   |-- broken_joints/               # experimental fault policy
|   |-- himloco/                     # reference policy
|   `-- robot_lab/                   # reference policy
|-- scripts/
|   |-- ros_env.sh                   # shared ROS2 workspace setup
|   |-- run_gazebo_go2.sh            # start Gazebo
|   |-- run_rl_sim_go2.sh            # start default rl_sim policy runner
|   |-- run_rl_sim_go2_ftnet_contact_history.sh
|   |-- log_policy_csv.py            # optional runtime CSV logger
|   `-- analyze_policy_debug_csv.py  # optional CSV analysis helper
`-- src/
    |-- rl_sar/                      # FSM, policy runtime, launch files
    |-- robot_joint_controller/      # ROS2 controller for Gazebo joints
    `-- robot_msgs/                  # RobotState / RobotCommand messages
```

Generated or downloaded folders:

```text
build/       generated colcon build output
install/     generated ROS2 install output
log/         generated colcon logs
debug_logs/  generated policy/debug CSVs
library/     downloaded inference runtimes
```

Do not treat those generated folders as source code.

## Runtime Flow

```text
Gazebo Go2 model
    -> joint state, IMU, contact topics
robot_joint_controller/state + /imu
    -> rl_sim
policy/go2/<policy_name>/policy.pt
    -> target joint commands
robot_joint_controller/command
    -> Gazebo joint effort control
```

`rl_sim` is the important runtime executable:

```bash
ros2 run rl_sar rl_sim
```

The wrapper scripts are preferred because they set up ROS2, source the workspace, and avoid shell-environment mistakes.

## Requirements

This workspace is currently used with:

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic
- `colcon`
- `gazebo_ros2_control`
- `xacro`
- system Python from ROS2

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

Use the build helper from the workspace root:

```bash
cd ~/Projects/go2_isaac_gazebo
source /opt/ros/humble/setup.bash
./build.sh robot_msgs robot_joint_controller go2_description rl_sar
```

Source the built workspace if you run ROS commands manually:

```bash
source install/setup.bash
```

Rebuild after changing C++, launch install rules, package manifests, xacro, or controller code.

You usually do not need to rebuild after changing only:

```text
policy/go2/*/config.yaml
policy/go2/*/policy.pt
```

Restart `rl_sim` after policy or YAML changes so the config is reloaded.

## Run Gazebo

Terminal 1:

```bash
cd ~/Projects/go2_isaac_gazebo
./scripts/run_gazebo_go2.sh
```

Terminal 2:

```bash
cd ~/Projects/go2_isaac_gazebo
./scripts/run_rl_sim_go2.sh
```

Recommended first test in the `rl_sim` terminal:

```text
0
wait until get-up completes
1
confirm it says: RL Controller [isaaclab48] holding default pose
W
```

## Keyboard Control

```text
0      get up into the default standing pose
1      enter policy controller
9      get down
P      passive mode
W/S    increase/decrease forward command
A/D    increase lateral-left/lateral-right command
Q/E    increase yaw-left/yaw-right command
Space  reset x/y/yaw command to zero
R      reset Gazebo world
Enter  pause/unpause simulation
```

Current keyboard command step: `0.05`.

Current command limits:

```text
forward/lateral: 1.0
yaw:             0.50
```

## Run FTNet Contact-History Policy

Start Gazebo as usual:

```bash
cd ~/Projects/go2_isaac_gazebo
./scripts/run_gazebo_go2.sh
```

Then run:

```bash
cd ~/Projects/go2_isaac_gazebo
./scripts/run_rl_sim_go2_ftnet_contact_history.sh
```

The FTNet policy expects:

```text
obs      shape [1, 49]
obs_hist shape [1, 30, 49]
```

It uses foot contacts in this order:

```text
[FL, FR, RL, RR]
```

More FTNet notes live in:

```text
docs/ftnet_contact_history_debug_notes.md
policy/go2/ftnet_contact_history/README.md
```

## Policies

Each runnable policy should live in its own folder:

```text
policy/go2/<policy_name>/
  config.yaml
  policy.pt
```

`base.yaml` selects the default policy:

```yaml
policy_config_name: "isaaclab48"
```

To test a new IsaacLab policy:

1. Export it as TorchScript.
2. Put it in a policy folder as `policy.pt`.
3. Make `config.yaml` match the training contract.
4. Restart `rl_sim`.

The important contract fields are:

```text
num_observations
observations
model_forward_mode
history_length
action_scale
commands_scale
default_dof_pos
joint_mapping
rl_kp / rl_kd
torque_limits
joint_pos_limit_lower / joint_pos_limit_upper
```

## IsaacLab Files To Compare

Useful IsaacLab-side references:

```text
/home/rinderudon/Projects/ws/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/velocity_env_cfg.py
/home/rinderudon/Projects/ws/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/go2/flat_env_cfg.py
/home/rinderudon/Projects/ws/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/go2/rough_env_cfg.py
/home/rinderudon/Projects/ws/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/unitree.py
```

Those files define the training-side robot asset, observation order, action scale, command ranges, timing, randomization, and terrain setup.

## CSV Logging

Use this only when debugging behavior:

```bash
cd ~/Projects/go2_isaac_gazebo
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 scripts/log_policy_csv.py
```

Logs are written to:

```text
debug_logs/
```

CSV logs are generated artifacts. They can be deleted when no longer needed.

## Fault Testing

The repo includes optional helpers for experimental fault scenarios:

```bash
ros2 run rl_sar run_fault_scenario.py
ros2 run rl_sar run_gain_fault_scenario.py
```

These publish per-joint alpha values to the fault topics used by the controller/policy runner.

Use them only after the normal Gazebo and policy flow is already working.

## Troubleshooting

If `ros2` or package lookup fails, use the wrapper scripts first:

```bash
./scripts/run_gazebo_go2.sh
./scripts/run_rl_sim_go2.sh
```

If the robot stands badly after pressing `1`:

- confirm the terminal says `holding default pose`
- check `hold_zero_command`
- check `default_dof_pos`
- check `joint_mapping`

If the robot walks in a curve:

- check yaw command is zero
- check lateral command is zero
- inspect latest CSV for left/right joint tracking mismatch
- check whether joint targets are hitting limits

If the robot falls immediately:

- restart Gazebo and `rl_sim`
- press `0` and wait for get-up to finish before pressing `1`
- confirm the expected policy is loaded
- rebuild if C++ or xacro files changed

## Cleanup Rules

Safe generated folders to remove when you want a smaller workspace:

```text
debug_logs/
log/
build/
install/
```

After removing `build/` or `install/`, rebuild before running.

Avoid deleting `library/` unless you are okay redownloading or restoring inference runtimes.

Loose policy exports should not stay directly under `policy/go2/`. Move them into a named policy folder or remove them if obsolete.

## Real Robot Caution

Passing Gazebo does not mean the policy is ready for hardware. Before any real Go2 deployment, re-check:

- action scaling
- joint order
- observation order
- command scaling
- control frequency
- joint limits
- torque limits
- emergency stop behavior
