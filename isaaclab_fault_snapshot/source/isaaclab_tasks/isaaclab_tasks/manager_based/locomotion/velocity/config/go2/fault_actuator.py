# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from collections.abc import Sequence

import torch

from isaaclab.actuators import DCMotor, DCMotorCfg
from isaaclab.utils import configclass
from isaaclab.utils.types import ArticulationActions


class FaultyDCMotor(DCMotor):
    """DC motor with per-episode actuator effectiveness faults.

    The normal DC motor computes the commanded torque from PD control. This
    model then applies an effectiveness multiplier:

        tau_actual = alpha * tau_command

    Healthy joints use alpha = 1.0. Faulted joints use a sampled alpha from
    ``fault_alpha_range`` after ``fault_start_time_range_s`` has elapsed.
    """

    cfg: "FaultyDCMotorCfg"

    def __init__(self, cfg: "FaultyDCMotorCfg", *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)

        self._step_count = torch.zeros(self._num_envs, dtype=torch.long, device=self._device)
        self._fault_start_step = torch.zeros_like(self._step_count)
        self._fault_alpha = torch.ones(self._num_envs, self.num_joints, device=self._device)
        self.current_alpha = torch.ones_like(self._fault_alpha)

        self.reset(slice(None))

    def reset(self, env_ids: Sequence[int] | slice | None):
        env_ids = self._resolve_env_ids(env_ids)
        num_reset = env_ids.numel()

        self._step_count[env_ids] = 0
        self.current_alpha[env_ids] = 1.0

        min_start_s, max_start_s = self.cfg.fault_start_time_range_s
        min_start_step = max(0, int(round(min_start_s / self.cfg.sim_dt)))
        max_start_step = max(min_start_step, int(round(max_start_s / self.cfg.sim_dt)))
        self._fault_start_step[env_ids] = torch.randint(
            min_start_step, max_start_step + 1, (num_reset,), device=self._device
        )

        env_has_fault = torch.rand(num_reset, device=self._device) < self.cfg.fault_probability
        joint_has_fault = torch.rand(num_reset, self.num_joints, device=self._device) < self.cfg.joint_fault_probability
        joint_has_fault &= env_has_fault.unsqueeze(1)

        empty_fault_rows = torch.nonzero(env_has_fault & ~joint_has_fault.any(dim=1), as_tuple=False).flatten()
        if empty_fault_rows.numel() > 0:
            chosen_joints = torch.randint(0, self.num_joints, (empty_fault_rows.numel(),), device=self._device)
            joint_has_fault[empty_fault_rows, chosen_joints] = True

        alpha_low, alpha_high = self.cfg.fault_alpha_range
        sampled_alpha = alpha_low + (alpha_high - alpha_low) * torch.rand(
            num_reset, self.num_joints, device=self._device
        )
        fault_alpha = torch.ones(num_reset, self.num_joints, device=self._device)
        fault_alpha[joint_has_fault] = sampled_alpha[joint_has_fault]

        self._fault_alpha[env_ids] = fault_alpha

    def compute(
        self, control_action: ArticulationActions, joint_pos: torch.Tensor, joint_vel: torch.Tensor
    ) -> ArticulationActions:
        control_action = super().compute(control_action, joint_pos, joint_vel)

        active_fault = self._step_count >= self._fault_start_step
        self.current_alpha[:] = torch.where(active_fault.unsqueeze(1), self._fault_alpha, 1.0)
        self.applied_effort *= self.current_alpha
        control_action.joint_efforts = self.applied_effort

        self._step_count += 1
        return control_action

    def _resolve_env_ids(self, env_ids: Sequence[int] | slice | None) -> torch.Tensor:
        if env_ids is None or isinstance(env_ids, slice):
            return torch.arange(self._num_envs, device=self._device)
        if isinstance(env_ids, torch.Tensor):
            return env_ids.to(device=self._device, dtype=torch.long)
        return torch.tensor(env_ids, device=self._device, dtype=torch.long)


@configclass
class FaultyDCMotorCfg(DCMotorCfg):
    """Configuration for random actuator-effectiveness faults."""

    class_type: type = FaultyDCMotor

    fault_probability: float = 0.75
    """Probability that an environment will contain at least one actuator fault."""

    joint_fault_probability: float = 0.18
    """Independent probability that each joint is faulted when the environment has a fault."""

    fault_alpha_range: tuple[float, float] = (0.0, 0.35)
    """Range for faulted joint effectiveness. 0.0 is no torque; 1.0 is healthy."""

    fault_start_time_range_s: tuple[float, float] = (1.0, 10.0)
    """Time range for when the fault activates inside each episode."""

    sim_dt: float = 0.005
    """Physics timestep used to convert fault activation time to actuator steps."""
