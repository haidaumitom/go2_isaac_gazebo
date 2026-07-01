# Go2 IsaacLab Policy Deployment

This workspace runs Unitree Go2 locomotion policies exported from IsaacLab in
Gazebo Classic and on the real Go2 through the `rl_sar` deployment runtime.

The project covers the deployment side of the workflow:

```text
IsaacLab training
    -> exported TorchScript policy
    -> this ROS2/Gazebo workspace
    -> rl_sar policy runner
    -> Gazebo joints or real Go2 lowcmd
```

The main goal is to validate that an IsaacLab policy still behaves correctly
when its observation order, action scale, joint mapping, gains, command scale,
and timing are reproduced outside IsaacLab.

## Repository Layout

```text
.
|-- build.sh                         # workspace build helper
|-- policy/go2/                      # exported policies and deployment configs
|   |-- base.yaml                    # shared Go2 runtime config
|   |-- isaaclab45_real/             # flat IsaacLab policy, 45 observations
|   |-- isaaclab48/                  # older IsaacLab baseline policy
|   |-- rough45/                # rough-terrain IsaacLab policy
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

Generated or downloaded folders are not source code:

```text
build/       colcon build output
install/     ROS2 install output
log/         colcon logs
debug_logs/  generated policy/debug CSVs
library/     downloaded inference runtimes
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

The current useful policies are:

```text
isaaclab45_real  flat IsaacLab policy with no base linear velocity observation
rough45     rough-terrain IsaacLab policy with 45 actor observations
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
RL_SAR_POLICY_CONFIG_NAME=rough45 ./scripts/run_rl_sim_go2.sh
```

Recommended test sequence in the `rl_sim` terminal:

```text
0
wait until get-up completes
1
test small W/S/A/D/Q/E commands
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

Keyboard command step:

```text
linear: 0.05
yaw:    0.05
```

Command limits:

```text
forward/lateral: 1.0
yaw:             0.50
```

## Real Go2 Deployment

Use the same exported policy folders for real deployment. IsaacLab is not needed
at runtime once the policy has been exported and copied into `policy/go2/`.

Example:

```bash
cd ~/Projects/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=rough45 ./install/lib/rl_sar/rl_real_go2 enp8s0
```

Use the correct network interface for your robot instead of `enp8s0` if needed.

Recommended real-robot test sequence:

```text
0
wait until get-up completes
1
leave joystick centered and check idle stability
test very small joystick commands first
```

Do real-robot testing on a stand or with support until the idle and low-command
behavior are stable. Keep the Unitree controller available for emergency stop.

## Gazebo And Real-Robot Difference

Gazebo and real deployment use the same policy interface, but the sources differ:

```text
Gazebo:
  ROS2 joint state, IMU, Gazebo model velocity, simulated contacts

Real Go2:
  Unitree DDS lowstate, IMU, motor state, joystick, lowcmd
```

Policies intended for real deployment should avoid actor observations that are
not available on the real robot unless a real estimator provides them. The
`isaaclab45_real` and `rough45` policies use 45 actor observations:

```text
ang_vel
gravity_vec
commands
dof_pos
dof_vel
actions
```

## CSV Debugging

Policy debug CSVs are written to:

```text
debug_logs/
```

They are generated artifacts and should not be committed.

Useful columns for diagnosing real-robot behavior:

```text
control_x / control_y / control_yaw
obs_cmd_x / obs_cmd_y / obs_cmd_yaw
action_*
target_q_*
robot_cmd_q_*
lowcmd_q_*
actual_q_*
actual_tau_est_*
```

Analyze a CSV with:

```bash
cd ~/Projects/go2_isaac_gazebo
/usr/bin/python3 scripts/analyze_policy_debug_csv.py debug_logs/<file>.csv
```

## Exporting A New IsaacLab Policy

IsaacLab is only needed for training, IsaacLab play, and export.

General workflow:

```text
train or resume in IsaacLab
play the checkpoint in IsaacLab
export TorchScript policy
copy exported policy into policy/go2/<new_policy_name>/
create or update config.yaml
test in Gazebo
test on real robot only after Gazebo and CSV checks look sane
```

Keep the deployment config aligned with the training contract:

```text
observation order
action scale
default joint pose
joint order / joint mapping
PD gains
control decimation and timing
command ranges
```

## Troubleshooting

If `ros2` or package lookup fails, use the wrapper scripts first:

```bash
./scripts/run_gazebo_go2.sh
./scripts/run_rl_sim_go2.sh
```

If the robot stands badly after pressing `1`:

- confirm the expected policy name is loaded
- check `default_dof_pos`
- check `joint_mapping`
- check `hold_zero_command`
- inspect the latest debug CSV

If the robot walks in a curve:

- confirm yaw and lateral commands are near zero
- inspect left/right joint target symmetry in the CSV
- check whether joint targets are hitting limits

If the robot pulses when joystick returns to center:

- check whether `hold_zero_command` is enabled
- compare `control_*` with `robot_cmd_q_*` in the CSV
- consider command smoothing or letting the policy handle zero command

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

Avoid deleting `library/` unless you are okay redownloading or restoring
inference runtimes.

Loose policy exports should not stay directly under `policy/go2/`. Move them
into a named policy folder or remove them if obsolete.

## Safety Note

Passing Gazebo does not mean a policy is automatically safe for hardware. Before
real Go2 deployment, re-check:

- action scaling
- joint order
- observation order
- command scaling
- control frequency
- joint limits
- torque limits
- joystick behavior
- emergency stop behavior
