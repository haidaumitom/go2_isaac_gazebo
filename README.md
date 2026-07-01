# Go2 IsaacLab Policy Deployment

This workspace runs Unitree Go2 locomotion policies exported from IsaacLab in
Gazebo Classic and on the real Go2 through the `rl_sar` deployment runtime.


## Repository Layout

```text
.
|-- build.sh                         # workspace build helper
|-- policy/go2/                      # exported policies and deployment configs
|   |-- base.yaml                    # shared Go2 runtime config
|   |-- isaaclab45_real/             # flat IsaacLab policy, 45 observations
|   |-- isaaclab48/                  # flat IsaacLab policy, 48 observations
|   |-- rough45/                     # rough-terrain IsaacLab policy
|   |-- himloco/                     # reference policy
|   `-- robot_lab/                   # reference policy
|-- scripts/
|   |-- ros_env.sh                   # ROS2 workspace setup helper
|   |-- run_gazebo_go2.sh            # start Gazebo
|   |-- run_rl_sim_go2.sh            # start Gazebo policy runner
|   |-- log_policy_csv.py            # optional runtime CSV logger
|   `-- analyze_policy_debug_csv.py  # CSV analysis helper
`-- src/
    |-- rl_sar/                      # policy runtime, FSM, real/Gazebo runners
    |-- robot_joint_controller/      # Gazebo joint command backend
    |-- robot_msgs/                  # RobotState / RobotCommand messages
    `-- rl_sar_zoo/                  # Go2 description assets
```


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

Initialize submodules after cloning:

```bash
git submodule update --init --recursive
```

## Build

Build the ROS2 packages from the workspace root:

```bash
cd ~/Projects/go2_isaac_gazebo
source /opt/ros/humble/setup.bash
colcon build --merge-install --symlink-install \
  --packages-select robot_msgs robot_joint_controller rl_sar \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

The helper script can also be used:

```bash
./build.sh robot_msgs robot_joint_controller go2_description rl_sar
```

Rebuild after changing C++, package manifests, launch install rules, xacro, or
controller code.

You usually do not need to rebuild after changing only:

```text
policy/go2/*/config.yaml
policy/go2/*/*.pt
```

Restart `rl_sim` or `rl_real_go2` after policy or YAML changes so the config is
loaded again.

## Policy Folders

Each deployable policy lives in its own folder:

```text
policy/go2/<policy_name>/
  config.yaml
  <model file>.pt
```

The model filename is defined in that folder's `config.yaml`:

```yaml
model_name: "policy.pt"
```

or, for the current rough policy:

```yaml
model_name: "rough45_policy.pt"
```

Important policy contract fields:

```text
num_observations
observations
action_scale
commands_scale
default_dof_pos
joint_mapping
rl_kp / rl_kd
torque_limits
joint_pos_limit_lower / joint_pos_limit_upper
hold_zero_command
```

The current policies are:

```text
isaaclab45_real  flat IsaacLab policy with no base linear velocity observation
rough45_9150     rough-terrain IsaacLab policy with 45 actor observations
isaaclab48       older baseline policy with 48 observations
```

Use `RL_SAR_POLICY_CONFIG_NAME` to select a policy without editing
`base.yaml`.

## Run Gazebo

Terminal 1:

```bash
cd ~/Projects/go2_isaac_gazebo
./scripts/run_gazebo_go2.sh
```

Terminal 2, default policy from `policy/go2/base.yaml`:

```bash
cd ~/Projects/go2_isaac_gazebo
./scripts/run_rl_sim_go2.sh
```

Terminal 2, rough policy:

```bash
cd ~/Projects/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=rough45_9150 ./scripts/run_rl_sim_go2.sh
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
```

## Real Go2 Deployment

Use the same exported policy folders for real deployment. IsaacLab is not needed
at runtime once the policy has been exported and copied into `policy/go2/`.

Example:

```bash
cd ~/Projects/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=rough45 ./install/lib/rl_sar/rl_real_go2 enp8s0
```


## CSV Debugging

Policy debug CSVs are written to:

```text
debug_logs/
```


