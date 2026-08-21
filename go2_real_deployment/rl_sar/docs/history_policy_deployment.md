# Observation-History Policy Deployment

The deployment runtime supports three model signatures:

```text
single          forward(obs)
flat_history    forward(flattened_history)
obs_history     forward(obs, obs_history)
```

For an IsaacLab observation group configured with `history_length = 30` and
`flatten_history_dim = False`, the usual TorchScript signature is:

```text
forward(obs: [1, D], obs_history: [1, 30, D]) -> actions: [1, 12]
```

IsaacLab exposes history from oldest to newest. The runtime buffer uses index
`0` for the newest frame, so the deployment order must be `[29, 28, ..., 0]`.

## Reusable Quadloco Profile

`policy/go2/profiles/quadloco45_history30.yaml` contains the complete shared
contract for a 45-observation, 30-frame, dual-input Quadloco policy. A new
policy using that exact training contract needs only a model and a small
descriptor:

```text
policy/go2/my_history_policy/
|-- config.yaml
`-- policy.pt
```

```yaml
go2/my_history_policy:
  profile: "quadloco45_history30"
  model_name: "policy.pt"
```

Do not use this profile solely because a model has 45 inputs. Its observation
order, scaling, history order, default pose, action scale, and joint mapping
must all match training.

The public Quadloco `ckpt/exported/policy.pt` currently has one 45-value input.
Although its environment file defines a 30-frame `HistoryCfg`, that group is not
enabled in the committed `ObservationsCfg`. Use this profile only for a separate
export whose model schema actually accepts current observations and history.

## Check The Exported Signature

After building `rl_sar`, validate through the same C++ inference runtime used by
Gazebo and the real robot:

```bash
./install/lib/rl_sar/validate_policy_model \
  policy/go2/my_history_policy/policy.pt obs_history 45 30 12
```

This command does not start Gazebo and does not communicate with the robot.

After creating the policy folder and `config.yaml`, validate the complete
merged base/profile/policy contract:

```bash
./install/lib/rl_sar/validate_policy_config go2 my_history_policy
```

This catches unsupported observation terms, incorrect observation counts,
history-shape errors, model-signature errors, and action-count errors before a
simulation or real-robot controller is started.

You can also inspect the TorchScript signature in a Python environment with
PyTorch:

```bash
python3 -c 'import torch; m=torch.jit.load("policy/go2/my_history_policy/policy.pt", map_location="cpu"); print(m.forward.schema); print(m(torch.zeros(1,45), torch.zeros(1,30,45)).shape)'
```

Expected shape:

```text
torch.Size([1, 12])
```

If the model instead accepts one flattened history tensor, override the mode
in the policy descriptor:

```yaml
go2/my_history_policy:
  profile: "quadloco45_history30"
  model_name: "policy.pt"
  model_forward_mode: "flat_history"
```

That model must accept `30 * 45 = 1350` input values.

## Runtime Validation

When policy mode starts, the runtime performs a dry inference before any
policy-generated joint target is used. It checks:

```text
configured observation count matches the generated observation
history length matches the selected history frames
model input count and tensor shapes are accepted
model returns exactly 12 finite actions
```

If validation fails, `InitRL()` fails and the FSM returns to passive mode.
Never bypass this check for a real-robot test.
