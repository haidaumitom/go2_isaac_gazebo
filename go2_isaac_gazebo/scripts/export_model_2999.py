#!/usr/bin/env python3
"""Export the model_2999 RSL-RL checkpoint for rl_sar deployment.

The checkpoint contains only state dictionaries, so its deterministic actor
graph must be reconstructed before TorchScript or ONNX can load it. The graph
contract recovered from the tensor shapes is:

    current observation:       [batch, 45]
    observation history:       [batch, 5, 45]
    history context: velocity (3), latent (16), fault probability (12)
    actor input:               [batch, 76]
    actions:                   [batch, 12]

The defaults use the conventions of the repository's HIM-style Go2 policy.
Keep the original training source/config with the exported model if it later
becomes available, since parameter tensors do not encode activation names or
observation semantics.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch
from torch import Tensor, nn


def activation_module(name: str) -> nn.Module:
    if name == "elu":
        return nn.ELU()
    if name == "relu":
        return nn.ReLU()
    if name == "tanh":
        return nn.Tanh()
    raise ValueError(f"Unsupported activation: {name}")


def make_mlp(input_dim: int, hidden_dims: list[int], output_dim: int, activation: str) -> nn.Sequential:
    layers: list[nn.Module] = []
    previous_dim = input_dim
    for hidden_dim in hidden_dims:
        layers.extend((nn.Linear(previous_dim, hidden_dim), activation_module(activation)))
        previous_dim = hidden_dim
    layers.append(nn.Linear(previous_dim, output_dim))
    return nn.Sequential(*layers)


def load_prefixed(module: nn.Module, state: dict[str, Tensor], prefix: str) -> None:
    marker = prefix + "."
    selected = {key[len(marker) :]: value for key, value in state.items() if key.startswith(marker)}
    if not selected:
        raise RuntimeError(f"Checkpoint has no parameters for {prefix}")
    module.load_state_dict(selected, strict=True)


class Model2999DeploymentPolicy(nn.Module):
    """Deterministic actor reconstructed from model_2999 tensor dimensions."""

    def __init__(
        self,
        actor_state: dict[str, Tensor],
        activation: str,
        code_first: bool,
        raw_fault_logits: bool,
    ) -> None:
        super().__init__()
        self.code_first = code_first
        self.raw_fault_logits = raw_fault_logits

        self.history_encoder = make_mlp(225, [512, 256], 128, activation)
        self.velocity_head = nn.Linear(128, 3)
        self.latent_head = nn.Linear(128, 16)
        self.fault_head = nn.Linear(128, 12)
        self.actor = make_mlp(76, [512, 256, 128], 12, activation)

        load_prefixed(self.history_encoder, actor_state, "hist_encoder_mlp")
        load_prefixed(self.velocity_head, actor_state, "mean_vel_encoder_mlp")
        load_prefixed(self.latent_head, actor_state, "mean_latent_encoder_mlp")
        load_prefixed(self.fault_head, actor_state, "fault_logit_encoder_mlp")
        load_prefixed(self.actor, actor_state, "actor_mlp")

    def forward(self, current_obs: Tensor, obs_history: Tensor) -> Tensor:
        history_features = self.history_encoder(torch.flatten(obs_history, start_dim=1))
        velocity = self.velocity_head(history_features)
        latent = self.latent_head(history_features)
        fault = self.fault_head(history_features)
        if not self.raw_fault_logits:
            fault = torch.sigmoid(fault)

        code = torch.cat((velocity, latent, fault), dim=-1)
        if self.code_first:
            actor_input = torch.cat((code, current_obs), dim=-1)
        else:
            actor_input = torch.cat((current_obs, code), dim=-1)
        return self.actor(actor_input)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--activation", choices=("elu", "relu", "tanh"), default="elu")
    input_order = parser.add_mutually_exclusive_group()
    input_order.add_argument("--code-first", dest="code_first", action="store_true")
    input_order.add_argument("--obs-first", dest="code_first", action="store_false")
    parser.set_defaults(code_first=True)
    parser.add_argument("--raw-fault-logits", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    if not isinstance(checkpoint, dict) or "actor_state_dict" not in checkpoint:
        raise RuntimeError("Expected an RSL-RL checkpoint containing actor_state_dict")

    actor_state = checkpoint["actor_state_dict"]
    expected_shapes = {
        "actor_mlp.0.weight": (512, 76),
        "actor_mlp.6.weight": (12, 128),
        "hist_encoder_mlp.0.weight": (512, 225),
        "mean_vel_encoder_mlp.weight": (3, 128),
        "mean_latent_encoder_mlp.weight": (16, 128),
        "fault_logit_encoder_mlp.weight": (12, 128),
    }
    for name, expected_shape in expected_shapes.items():
        actual_shape = tuple(actor_state[name].shape)
        if actual_shape != expected_shape:
            raise RuntimeError(f"{name} has shape {actual_shape}; expected {expected_shape}")

    policy = Model2999DeploymentPolicy(
        actor_state,
        activation=args.activation,
        code_first=args.code_first,
        raw_fault_logits=args.raw_fault_logits,
    ).eval()

    current_obs = torch.zeros(1, 45, dtype=torch.float32)
    obs_history = torch.zeros(1, 5, 45, dtype=torch.float32)
    with torch.inference_mode():
        actions = policy(current_obs, obs_history)
    if actions.shape != (1, 12) or not torch.isfinite(actions).all():
        raise RuntimeError(f"Invalid policy output: shape={tuple(actions.shape)}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    torchscript_path = args.output_dir / "policy.pt"
    onnx_path = args.output_dir / "policy.onnx"

    scripted = torch.jit.script(policy)
    torch.jit.save(scripted, torchscript_path)
    torch.onnx.export(
        policy,
        (current_obs, obs_history),
        onnx_path,
        input_names=("current_obs", "obs_history"),
        output_names=("actions",),
        dynamic_axes={"current_obs": {0: "batch"}, "obs_history": {0: "batch"}, "actions": {0: "batch"}},
        opset_version=17,
        dynamo=False,
    )

    print(f"Checkpoint iteration: {checkpoint.get('iter', 'unknown')}")
    print(f"Activation: {args.activation}")
    print(f"Actor input order: {'code,current_obs' if args.code_first else 'current_obs,code'}")
    print(f"Fault representation: {'raw logits' if args.raw_fault_logits else 'sigmoid probability'}")
    print(f"Zero-input action range: [{actions.min().item():.4f}, {actions.max().item():.4f}]")
    print(f"TorchScript: {torchscript_path}")
    print(f"ONNX: {onnx_path}")


if __name__ == "__main__":
    main()
