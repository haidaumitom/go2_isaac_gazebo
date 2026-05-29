# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from .fault_actuator import FaultyDCMotorCfg
from .flat_env_cfg import UnitreeGo2FlatEnvCfg


@configclass
class UnitreeGo2FlatFaultEnvCfg(UnitreeGo2FlatEnvCfg):
    """Flat Go2 locomotion task with random actuator-effectiveness faults.

    The policy still outputs desired joint positions, just like the normal Go2
    flat task. The custom actuator changes the physical result by scaling the
    actual torque after the PD/DC-motor calculation.
    """

    def __post_init__(self):
        super().__post_init__()

        nominal_actuator = self.scene.robot.actuators["base_legs"]
        self.scene.robot.actuators["base_legs"] = FaultyDCMotorCfg(
            joint_names_expr=nominal_actuator.joint_names_expr,
            effort_limit=nominal_actuator.effort_limit,
            velocity_limit=nominal_actuator.velocity_limit,
            effort_limit_sim=nominal_actuator.effort_limit_sim,
            velocity_limit_sim=nominal_actuator.velocity_limit_sim,
            stiffness=nominal_actuator.stiffness,
            damping=nominal_actuator.damping,
            armature=nominal_actuator.armature,
            friction=nominal_actuator.friction,
            dynamic_friction=nominal_actuator.dynamic_friction,
            viscous_friction=nominal_actuator.viscous_friction,
            saturation_effort=nominal_actuator.saturation_effort,
            fault_probability=0.75,
            joint_fault_probability=0.18,
            fault_alpha_range=(0.0, 0.05),
            fault_start_time_range_s=(1.0, 10.0),
            sim_dt=self.sim.dt,
        )

        # Make the fault task favor survival/recovery over aggressive speed.
        self.commands.base_velocity.ranges.lin_vel_x = (-0.8, 0.8)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.5, 0.5)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.8, 0.8)
        self.rewards.lin_vel_z_l2.weight = -3.0
        self.rewards.ang_vel_xy_l2.weight = -0.08
        self.rewards.flat_orientation_l2.weight = -3.0
        self.rewards.dof_torques_l2.weight = -0.0001


class UnitreeGo2FlatFaultEnvCfg_PLAY(UnitreeGo2FlatFaultEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()

        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None

        actuator = self.scene.robot.actuators["base_legs"]
        actuator.fault_probability = 1.0
        actuator.joint_fault_probability = 0.12
        actuator.fault_start_time_range_s = (10.0, 10.0)


@configclass
class Go2FlatBrokenJointsEnvCfg(UnitreeGo2FlatFaultEnvCfg):
    """Flat Go2 task for recovering after one or more joints become ineffective.

    This is the first exact scenario: the robot starts with normal walking or
    standing commands, then a random subset of joints suddenly produces almost
    no usable torque. The policy should learn that staying upright is more
    important than perfectly following the commanded velocity after the fault.
    """

    def __post_init__(self):
        super().__post_init__()

        actuator = self.scene.robot.actuators["base_legs"]

        # Scenario timing and severity:
        # - most environments receive a fault
        # - each joint has a small independent chance of being selected
        # - selected joints become nearly passive after the start time
        actuator.fault_probability = 0.9
        actuator.joint_fault_probability = 0.1
        actuator.fault_alpha_range = (0.0, 0.05)
        actuator.fault_start_time_range_s = (8.0, 15.0)

        # Keep commands moderate so the first policy learns survival/recovery
        # before we ask it to walk aggressively with broken joints.
        self.commands.base_velocity.ranges.lin_vel_x = (-0.5, 0.5)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.3, 0.3)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.5, 0.5)

        # Recovery rewards: strongly punish falling, tilting, and violent body
        # rotation. This makes "do not lie down" the main training signal.
        self.rewards.lin_vel_z_l2.weight = -4.0
        self.rewards.ang_vel_xy_l2.weight = -0.15
        self.rewards.flat_orientation_l2.weight = -5.0
        self.rewards.dof_torques_l2.weight = -0.0001
        self.rewards.action_rate_l2.weight = -0.02


class Go2FlatBrokenJointsEnvCfg_PLAY(Go2FlatBrokenJointsEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()

        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None

        # Make play mode easy to observe: every robot gets a fault between
        # 8 and 15 seconds, but with fewer failed joints so behavior is readable.
        actuator = self.scene.robot.actuators["base_legs"]
        actuator.fault_probability = 1.0
        actuator.joint_fault_probability = 0.08
        actuator.fault_alpha_range = (0.0, 0.02)
        actuator.fault_start_time_range_s = (8.0, 15.0)
