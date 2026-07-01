/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GO2_FSM_HPP
#define GO2_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"
#include <cstdlib>

namespace go2_fsm
{

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            // fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 8;
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float percent_pre_getup = 0.0f;
    float percent_getup = 0.0f;
    std::vector<float> pre_running_pos = {
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65,
        0.00, 0.00, 0.00, 0.00
    };
    bool stand_from_passive = true;

    void Enter() override
    {
        percent_pre_getup = 0.0f;
        percent_getup = 0.0f;
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive")
        {
            stand_from_passive = true;
        }
        else
        {
            stand_from_passive = false;
        }
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        if(stand_from_passive)
        {

            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 1.0f, "Pre Getting up", true)) return;
            if (Interpolate(percent_getup, pre_running_pos, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "Getting up", true)) return;
        }
        else
        {
            if (Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 1.0f, "Getting up", true)) return;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRLLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.0f, "Getting down", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;
        rl.rl_init_done = false;

        const auto previous_joint_mapping = rl.params.Get<std::vector<int>>("joint_mapping");
        const RobotState<float> previous_state = *fsm_state;

        // read params from yaml
        const char* policy_config_override = std::getenv("RL_SAR_POLICY_CONFIG_NAME");
        rl.config_name = (policy_config_override && std::string(policy_config_override).size() > 0)
            ? std::string(policy_config_override)
            : rl.params.Get<std::string>("policy_config_name", "isaaclab48");
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            const auto current_joint_mapping = rl.params.Get<std::vector<int>>("joint_mapping");
            auto remap_to_current_order = [&](const std::vector<float>& values)
            {
                if (values.size() != previous_joint_mapping.size() || values.size() != current_joint_mapping.size())
                {
                    return values;
                }

                std::vector<float> controller_order(values.size(), 0.0f);
                for (size_t i = 0; i < values.size(); ++i)
                {
                    const int controller_index = previous_joint_mapping[i];
                    if (controller_index >= 0 && controller_index < static_cast<int>(values.size()))
                    {
                        controller_order[controller_index] = values[i];
                    }
                }

                std::vector<float> current_order(values.size(), 0.0f);
                for (size_t i = 0; i < values.size(); ++i)
                {
                    const int controller_index = current_joint_mapping[i];
                    if (controller_index >= 0 && controller_index < static_cast<int>(values.size()))
                    {
                        current_order[i] = controller_order[controller_index];
                    }
                }
                return current_order;
            };

            rl.now_state = previous_state;
            rl.now_state.motor_state.q = remap_to_current_order(previous_state.motor_state.q);
            rl.now_state.motor_state.dq = remap_to_current_order(previous_state.motor_state.dq);
            rl.now_state.motor_state.tau_est = remap_to_current_order(previous_state.motor_state.tau_est);

            std::vector<float> stale_output;
            while (rl.output_dof_pos_queue.try_pop(stale_output)) {}
            while (rl.output_dof_vel_queue.try_pop(stale_output)) {}
            while (rl.output_dof_tau_queue.try_pop(stale_output)) {}
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        try
        {
            // position transition from last default_dof_pos to current default_dof_pos
            if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 1.5f, "Policy transition", true)) return;

            const float stand_command_threshold = rl.params.Get<float>("stand_command_threshold", 0.02f);
            const bool zero_command =
                std::abs(rl.control.x) < stand_command_threshold &&
                std::abs(rl.control.y) < stand_command_threshold &&
                std::abs(rl.control.yaw) < stand_command_threshold;
            if (rl.params.Get<bool>("hold_zero_command", false) && zero_command)
            {
                rl.rl_init_done = false;
                std::vector<float> stale_output;
                while (rl.output_dof_pos_queue.try_pop(stale_output)) {}
                while (rl.output_dof_vel_queue.try_pop(stale_output)) {}
                while (rl.output_dof_tau_queue.try_pop(stale_output)) {}

                const auto default_dof_pos = rl.params.Get<std::vector<float>>("default_dof_pos");
                const auto kp = rl.params.Get<std::vector<float>>("rl_kp");
                const auto kd = rl.params.Get<std::vector<float>>("rl_kd");
                for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
                {
                    fsm_command->motor_command.q[i] = default_dof_pos[i];
                    fsm_command->motor_command.dq[i] = 0;
                    fsm_command->motor_command.kp[i] = kp[i];
                    fsm_command->motor_command.kd[i] = kd[i];
                    fsm_command->motor_command.tau[i] = 0;
                }

                std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] holding default pose" << std::flush;
                return;
            }

            if (!rl.rl_init_done) rl.rl_init_done = true;

            std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
            RLControl();
        }
        catch (const std::exception& e)
        {
            std::cout << std::endl << LOGGER::ERROR << "RLLocomotion failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }
        return state_name_;
    }
};

} // namespace go2_fsm

class Go2FSMFactory : public FSMFactory
{
public:
    Go2FSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<go2_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<go2_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<go2_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<go2_fsm::RLFSMStateRLLocomotion>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "go2"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLLocomotion"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(Go2FSMFactory, "RLFSMStatePassive")

#endif // GO2_FSM_HPP
