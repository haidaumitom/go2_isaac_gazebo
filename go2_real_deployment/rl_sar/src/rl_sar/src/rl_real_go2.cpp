/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_go2.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <thread>

namespace
{
constexpr float kRadiansToDegrees = 57.2957795f;

int64_t SteadyClockNowNanoseconds()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

RL_Real::RL_Real(int argc, char **argv)
{
#if defined(USE_ROS)
    ros2_node = std::make_shared<rclcpp::Node>("rl_real_node");
    this->cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) {this->CmdvelCallback(msg);}
    );
#endif

    // read params from yaml
    this->ang_vel_axis = "body";
    this->robot_name = "go2";
    this->ReadYaml(this->robot_name, "base.yaml");

    const char* real_tuck_gate = std::getenv("RL_SAR_ENABLE_REAL_TUCK_TEST");
    this->real_tuck_launch_enabled = real_tuck_gate && std::string(real_tuck_gate) == "1";
    this->real_tuck_state = this->real_tuck_launch_enabled ? RealTuckState::Idle : RealTuckState::Disabled;
    if (this->real_tuck_launch_enabled)
    {
        std::cout << LOGGER::WARNING
                  << "Real tucked-leg test gate enabled. F arms; T triggers; P aborts to passive."
                  << std::endl;
    }

    const char* real_gain_fault_gate = std::getenv("RL_SAR_ENABLE_REAL_GAIN_FAULT_TEST");
    this->real_gain_fault_launch_enabled =
        real_gain_fault_gate && std::string(real_gain_fault_gate) == "1";
    this->real_gain_fault_state = this->real_gain_fault_launch_enabled
        ? RealGainFaultState::Idle : RealGainFaultState::Disabled;
    if (this->real_gain_fault_launch_enabled)
    {
        std::cout << LOGGER::WARNING
                  << "Real gain-fault test gate enabled. F arms; T triggers; P aborts to GetDown."
                  << std::endl;
    }
    if (this->real_tuck_launch_enabled && this->real_gain_fault_launch_enabled)
    {
        throw std::runtime_error(
            "Real tuck and real gain-fault test gates cannot be enabled together");
    }

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->InitLowCmd();
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();
    // create lowcmd publisher
    this->lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    this->lowcmd_publisher->InitChannel();
    // create lowstate subscriber
    this->lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    this->lowstate_subscriber->InitChannel(std::bind(&RL_Real::LowStateMessageHandler, this, std::placeholders::_1), 1);
    // create joystick subscriber
    // init MotionSwitcherClient
    this->msc.SetTimeout(10.0f);
    this->msc.Init();
    // Shut down motion control-related service
    while(this->QueryMotionStatus())
    {
        std::cout << "Try to deactivate the motion control-related service." << std::endl;
        int32_t ret = this->msc.ReleaseMode();
        if (ret == 0)
        {
            std::cout << "ReleaseMode succeeded." << std::endl;
        }
        else
        {
            std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
        }
        sleep(1);
    }

    // loop
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this));
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
}

RL_Real::~RL_Real()
{
    this->Shutdown();
}

void RL_Real::Shutdown()
{
    if (this->shutdown_started.exchange(true))
    {
        return;
    }

    std::cout << std::endl << LOGGER::WARNING
              << "Safe shutdown: moving to prone before releasing motor stiffness." << std::endl;
    this->rl_init_done = false;

    if (this->real_gain_fault_launch_enabled)
    {
        std::lock_guard<std::mutex> lock(this->real_gain_fault_mutex);
        this->SetGainAlpha(std::vector<float>(this->params.Get<int>("num_of_dofs"), 1.0f));
        this->real_gain_fault_current_alpha = 1.0f;
    }

    if (this->loop_keyboard)
    {
        this->loop_keyboard->shutdown();
    }
    if (this->loop_rl)
    {
        this->loop_rl->shutdown();
    }
#ifdef PLOT
    if (this->loop_plot)
    {
        this->loop_plot->shutdown();
    }
#endif

    const bool reached_prone = this->MoveToProneForShutdown();
    if (!reached_prone)
    {
        std::cout << LOGGER::WARNING
                  << "Controlled prone shutdown did not complete; using the gradual motor-release fallback."
                  << std::endl;
    }

    if (this->loop_control)
    {
        this->loop_control->shutdown();
    }
    this->shutdown_prone_requested.store(false);

    this->PublishSafeMotorRelease();
    this->ClosePolicyDebugCsv();

    if (this->lowstate_subscriber)
    {
        this->lowstate_subscriber->CloseChannel();
        this->lowstate_subscriber.reset();
    }
    if (this->lowcmd_publisher)
    {
        this->lowcmd_publisher->CloseChannel();
        this->lowcmd_publisher.reset();
    }
    std::cout << LOGGER::INFO << "RL_Real safe shutdown complete" << std::endl;
}

bool RL_Real::MoveToProneForShutdown()
{
    if (!this->params.Get<bool>("shutdown_prone_enabled", true))
    {
        std::cout << LOGGER::WARNING << "Controlled prone shutdown is disabled by configuration." << std::endl;
        return false;
    }
    if (!this->loop_control || !this->lowcmd_publisher)
    {
        return false;
    }

    const double lowstate_age_s = this->LowStateAgeSeconds();
    if (!std::isfinite(lowstate_age_s) || lowstate_age_s > 0.25)
    {
        std::cout << LOGGER::WARNING
                  << "Controlled prone shutdown skipped: LowState is stale (age "
                  << lowstate_age_s << " s)." << std::endl;
        return false;
    }

    const float configured_timeout_s = this->params.Get<float>("shutdown_prone_timeout_s", 5.0f);
    const float timeout_s = std::max(2.5f, configured_timeout_s);
    this->shutdown_prone_complete.store(false);
    this->shutdown_getdown_started.store(false);
    this->shutdown_prone_requested.store(true);

    std::cout << LOGGER::INFO
              << "Requesting controlled GetDown trajectory; timeout "
              << timeout_s << " s." << std::endl;

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<float>(timeout_s));
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (this->shutdown_prone_complete.load())
        {
            std::cout << LOGGER::INFO << "Controlled prone posture reached." << std::endl;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return false;
}

void RL_Real::GetState(RobotState<float> *state)
{
    if (this->unitree_joy.components.A) this->control.SetGamepad(Input::Gamepad::A);
    if (this->unitree_joy.components.B) this->control.SetGamepad(Input::Gamepad::B);
    if (this->unitree_joy.components.X) this->control.SetGamepad(Input::Gamepad::X);
    if (this->unitree_joy.components.Y) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->unitree_joy.components.L1) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->unitree_joy.components.R1) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->unitree_joy.components.F1) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->unitree_joy.components.F2) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->unitree_joy.components.up) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->unitree_joy.components.down) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->unitree_joy.components.left) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->unitree_joy.components.right) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.A) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.B) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.X) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.Y) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.F1) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.F2) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.up) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.down) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.left) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.right) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.A) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.B) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.X) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.Y) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.F1) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.F2) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.up) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.down) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.left) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.right) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.R1) this->control.SetGamepad(Input::Gamepad::LB_RB);

    this->control.x = this->joystick.ly();
    this->control.y = -this->joystick.lx();
    this->control.yaw = -this->joystick.rx();

    state->imu.quaternion[0] = this->unitree_low_state.imu_state().quaternion()[0]; // w
    state->imu.quaternion[1] = this->unitree_low_state.imu_state().quaternion()[1]; // x
    state->imu.quaternion[2] = this->unitree_low_state.imu_state().quaternion()[2]; // y
    state->imu.quaternion[3] = this->unitree_low_state.imu_state().quaternion()[3]; // z

    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = this->unitree_low_state.imu_state().gyroscope()[i];
    }
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        state->motor_state.q[i] = this->unitree_low_state.motor_state()[this->params.Get<std::vector<int>>("joint_mapping")[i]].q();
        state->motor_state.dq[i] = this->unitree_low_state.motor_state()[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq();
        state->motor_state.tau_est[i] = this->unitree_low_state.motor_state()[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau_est();
    }
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    unitree_go::msg::dds_::LowCmd_ dds_low_command;
    dds_low_command.head()[0] = 0xFE;
    dds_low_command.head()[1] = 0xEF;
    dds_low_command.level_flag() = 0xFF;
    dds_low_command.gpio() = 0;

    for (int i = 0; i < 20; ++i)
    {
        dds_low_command.motor_cmd()[i].mode() = 0x01;
        dds_low_command.motor_cmd()[i].q() = PosStopF;
        dds_low_command.motor_cmd()[i].kp() = 0;
        dds_low_command.motor_cmd()[i].dq() = VelStopF;
        dds_low_command.motor_cmd()[i].kd() = 0;
        dds_low_command.motor_cmd()[i].tau() = 0;
    }

    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].mode() = 0x01;
        dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].q() = command->motor_command.q[i];
        dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq() = command->motor_command.dq[i];
        dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].kp() = command->motor_command.kp[i];
        dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].kd() = command->motor_command.kd[i];
        dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau() = command->motor_command.tau[i];
    }

    dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    this->unitree_low_command = dds_low_command;
    lowcmd_publisher->Write(this->unitree_low_command);
}

void RL_Real::PublishSafeMotorRelease()
{
    if (!this->lowcmd_publisher)
    {
        std::cout << LOGGER::WARNING
                  << "Safe shutdown could not publish: LowCmd publisher is unavailable."
                  << std::endl;
        return;
    }

    constexpr int kMotorCount = 20;
    constexpr int kRampCycles = 100;
    constexpr int kDampingCycles = 60;
    constexpr int kReleaseCycles = 100;
    constexpr int kFinalCycles = 20;
    constexpr float kShutdownDamping = 8.0f;
    constexpr auto kPublishPeriod = std::chrono::milliseconds(5);

    std::array<float, kMotorCount> hold_q{};
    std::array<float, kMotorCount> initial_kp{};
    std::array<float, kMotorCount> initial_kd{};
    std::array<float, kMotorCount> initial_tau{};
    std::array<bool, kMotorCount> controlled{};

    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    for (int policy_index = 0;
         policy_index < static_cast<int>(joint_mapping.size()) &&
         policy_index < this->params.Get<int>("num_of_dofs");
         ++policy_index)
    {
        const int motor_index = joint_mapping[policy_index];
        if (motor_index < 0 || motor_index >= kMotorCount)
        {
            continue;
        }

        controlled[motor_index] = true;
        const float measured_q = this->unitree_low_state.motor_state()[motor_index].q();
        hold_q[motor_index] = std::isfinite(measured_q)
            ? measured_q
            : this->unitree_low_command.motor_cmd()[motor_index].q();
        initial_kp[motor_index] = std::max(
            0.0f, this->unitree_low_command.motor_cmd()[motor_index].kp());
        initial_kd[motor_index] = std::max(
            0.0f, this->unitree_low_command.motor_cmd()[motor_index].kd());
        initial_tau[motor_index] = this->unitree_low_command.motor_cmd()[motor_index].tau();
    }

    auto make_neutral_command = [&]()
    {
        unitree_go::msg::dds_::LowCmd_ command{};
        command.head()[0] = 0xFE;
        command.head()[1] = 0xEF;
        command.level_flag() = 0xFF;
        command.gpio() = 0;
        for (int i = 0; i < kMotorCount; ++i)
        {
            command.motor_cmd()[i].mode() = 0x01;
            command.motor_cmd()[i].q() = PosStopF;
            command.motor_cmd()[i].kp() = 0.0f;
            command.motor_cmd()[i].dq() = VelStopF;
            command.motor_cmd()[i].kd() = 0.0f;
            command.motor_cmd()[i].tau() = 0.0f;
        }
        return command;
    };

    auto publish = [&](unitree_go::msg::dds_::LowCmd_& command)
    {
        command.crc() = Crc32Core(
            reinterpret_cast<uint32_t*>(&command),
            (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
        this->unitree_low_command = command;
        this->lowcmd_publisher->Write(this->unitree_low_command);
        std::this_thread::sleep_for(kPublishPeriod);
    };

    // Hold the measured pose while smoothly removing position stiffness and
    // feed-forward torque. Damping rises during this phase to limit a drop.
    for (int cycle = 0; cycle < kRampCycles; ++cycle)
    {
        const float alpha = static_cast<float>(cycle + 1) / kRampCycles;
        auto command = make_neutral_command();
        for (int i = 0; i < kMotorCount; ++i)
        {
            if (!controlled[i])
            {
                continue;
            }
            command.motor_cmd()[i].q() = hold_q[i];
            command.motor_cmd()[i].kp() = initial_kp[i] * (1.0f - alpha);
            command.motor_cmd()[i].dq() = 0.0f;
            command.motor_cmd()[i].kd() =
                initial_kd[i] * (1.0f - alpha) + kShutdownDamping * alpha;
            command.motor_cmd()[i].tau() = initial_tau[i] * (1.0f - alpha);
        }
        publish(command);
    }

    // Brief damping-only phase, followed by a smooth reduction to zero.
    for (int cycle = 0; cycle < kDampingCycles; ++cycle)
    {
        auto command = make_neutral_command();
        for (int i = 0; i < kMotorCount; ++i)
        {
            if (controlled[i])
            {
                command.motor_cmd()[i].dq() = 0.0f;
                command.motor_cmd()[i].kd() = kShutdownDamping;
            }
        }
        publish(command);
    }

    for (int cycle = 0; cycle < kReleaseCycles; ++cycle)
    {
        const float alpha = static_cast<float>(cycle + 1) / kReleaseCycles;
        auto command = make_neutral_command();
        for (int i = 0; i < kMotorCount; ++i)
        {
            if (controlled[i])
            {
                command.motor_cmd()[i].dq() = 0.0f;
                command.motor_cmd()[i].kd() = kShutdownDamping * (1.0f - alpha);
            }
        }
        publish(command);
    }

    // Finish in Unitree stop/standby mode. The preceding phases remain in FOC
    // mode so stiffness is removed gradually before the motors are disabled.
    for (int cycle = 0; cycle < kFinalCycles; ++cycle)
    {
        auto command = make_neutral_command();
        for (int i = 0; i < kMotorCount; ++i)
        {
            command.motor_cmd()[i].mode() = 0x00;
        }
        publish(command);
    }
}

void RL_Real::RobotControl()
{
    this->GetState(&this->robot_state);

    const bool prone_shutdown = this->shutdown_prone_requested.load();
    if (prone_shutdown)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->control.current_keyboard = Input::Keyboard::None;
        this->control.last_keyboard = Input::Keyboard::None;
        this->control.current_gamepad = Input::Gamepad::None;
        this->control.last_gamepad = Input::Gamepad::None;
        this->real_tuck_abort_requested.store(false);

        if (this->fsm.current_state_)
        {
            const std::string& state_name = this->fsm.current_state_->GetStateName();
            if (state_name == "RLFSMStateGetDown")
            {
                this->shutdown_getdown_started.store(true);
            }
            else if (state_name != "RLFSMStatePassive" ||
                     (this->getup_reference_valid && !this->shutdown_getdown_started.load()))
            {
                this->fsm.RequestStateChange("RLFSMStateGetDown");
                this->shutdown_getdown_started.store(true);
            }
        }
    }
    else if (this->real_tuck_abort_requested.exchange(false))
    {
        this->control.SetKeyboard(Input::Keyboard::P);
    }
    else if (this->real_gain_fault_getdown_requested.exchange(false))
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->fsm.RequestStateChange("RLFSMStateGetDown");
    }

    this->StateController(&this->robot_state, &this->robot_command);

    if (!prone_shutdown)
    {
        this->HandleRealTuckControlInput();
        this->HandleRealGainFaultControlInput();
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);

    if (prone_shutdown && this->fsm.current_state_ &&
        this->fsm.current_state_->GetStateName() == "RLFSMStatePassive" &&
        (this->shutdown_getdown_started.load() || !this->getup_reference_valid))
    {
        this->shutdown_prone_complete.store(true);
    }

    this->WritePolicyDebugCsv();
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
#if !defined(USE_CMAKE) && defined(USE_ROS)
        if (this->control.navigation_mode)
        {
            this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};

        }
#endif
        this->ApplyRealTuckCommandGuard();
        this->ApplyRealGainFaultCommandGuard();
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->UpdateRealGainFault();

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);
        this->ApplyRealTuckedLegFault();

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

void RL_Real::InitPolicyDebugCsv()
{
    this->policy_debug_csv_enabled = this->params.Get<bool>("policy_debug_csv_enabled", false);
    this->policy_debug_csv_stride = std::max(1, this->params.Get<int>("policy_debug_csv_stride", 1));
    this->policy_debug_csv_counter = 0;
    this->policy_debug_csv_initialized = true;

    if (!this->policy_debug_csv_enabled)
    {
        return;
    }

    std::string csv_path = this->params.Get<std::string>("policy_debug_csv_path", "");
    if (csv_path.empty())
    {
        const auto now = std::chrono::system_clock::now();
        const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm = *std::localtime(&now_time);
        std::ostringstream timestamp;
        timestamp << std::put_time(&now_tm, "%Y%m%d_%H%M%S");

        const std::string active_config_name = this->config_name.empty() ? "unknown" : this->config_name;
        const std::filesystem::path debug_dir = std::filesystem::path(POLICY_DIR).parent_path() / "debug_logs";
        std::filesystem::create_directories(debug_dir);
        csv_path = (debug_dir / ("real_policy_debug_" + active_config_name + "_" + timestamp.str() + ".csv")).string();
    }
    else
    {
        const auto parent_path = std::filesystem::path(csv_path).parent_path();
        if (!parent_path.empty())
        {
            std::filesystem::create_directories(parent_path);
        }
    }

    this->policy_debug_csv_file.open(csv_path);
    if (!this->policy_debug_csv_file.is_open())
    {
        std::cout << LOGGER::ERROR << "Could not open real policy debug CSV: " << csv_path << std::endl;
        this->policy_debug_csv_enabled = false;
        return;
    }

    this->policy_debug_csv_file << std::fixed << std::setprecision(6);
    this->policy_debug_csv_file
        << "episode_step,wall_time_s,lowstate_tick,"
        << "real_tuck_state,real_tuck_blend,real_tuck_lowstate_age_s,"
        << "real_gain_fault_state,real_gain_fault_policy_index,"
        << "real_gain_fault_hardware_index,real_gain_fault_alpha,"
        << "control_x,control_y,control_yaw,"
        << "obs_cmd_x,obs_cmd_y,obs_cmd_yaw,"
        << "lin_vel_x,lin_vel_y,lin_vel_z,"
        << "ang_vel_x,ang_vel_y,ang_vel_z,"
        << "gravity_body_x,gravity_body_y,gravity_body_z,"
        << "base_quat_w,base_quat_x,base_quat_y,base_quat_z,"
        << "joy_lx,joy_ly,joy_rx,joy_ry,joy_keys,"
        << "foot_force_0,foot_force_1,foot_force_2,foot_force_3,"
        << "foot_force_est_0,foot_force_est_1,foot_force_est_2,foot_force_est_3";

    const auto joint_names = this->params.Get<std::vector<std::string>>("joint_names");
    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    const int num_dofs = this->params.Get<int>("num_of_dofs");
    for (int i = 0; i < num_dofs; ++i)
    {
        std::string joint_name = "joint_" + std::to_string(i);
        if (i < static_cast<int>(joint_mapping.size()) &&
            joint_mapping[i] >= 0 &&
            joint_mapping[i] < static_cast<int>(joint_names.size()))
        {
            joint_name = joint_names[joint_mapping[i]];
        }

        this->policy_debug_csv_file
            << ",action_" << joint_name
            << ",target_q_" << joint_name
            << ",target_dq_" << joint_name
            << ",target_tau_" << joint_name
            << ",actual_q_" << joint_name
            << ",actual_dq_" << joint_name
            << ",actual_tau_est_" << joint_name
            << ",motor_temp_c_" << joint_name
            << ",robot_cmd_q_" << joint_name
            << ",robot_cmd_dq_" << joint_name
            << ",robot_cmd_kp_" << joint_name
            << ",robot_cmd_kd_" << joint_name
            << ",robot_cmd_tau_" << joint_name
            << ",lowcmd_q_" << joint_name
            << ",lowcmd_dq_" << joint_name
            << ",lowcmd_kp_" << joint_name
            << ",lowcmd_kd_" << joint_name
            << ",lowcmd_tau_" << joint_name;
    }
    this->policy_debug_csv_file << "\n";
    this->policy_debug_csv_file.flush();

    std::cout << LOGGER::INFO << "Real policy debug CSV logging to: " << csv_path << std::endl;
}

void RL_Real::WritePolicyDebugCsv()
{
    if (!this->params.Has("policy_debug_csv_enabled"))
    {
        return;
    }
    if (!this->policy_debug_csv_initialized)
    {
        this->InitPolicyDebugCsv();
    }
    if (!this->policy_debug_csv_enabled || !this->policy_debug_csv_file.is_open())
    {
        return;
    }
    if ((this->policy_debug_csv_counter++ % this->policy_debug_csv_stride) != 0)
    {
        return;
    }

    auto value_at = [](const std::vector<float>& values, int index) -> float
    {
        return (index >= 0 && index < static_cast<int>(values.size())) ? values[index] : 0.0f;
    };

    std::vector<float> gravity_body = {0.0f, 0.0f, 0.0f};
    if (this->obs.base_quat.size() == 4 && this->obs.gravity_vec.size() == 3)
    {
        gravity_body = QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec);
    }

    const auto now = std::chrono::system_clock::now();
    const double wall_time_s = std::chrono::duration<double>(now.time_since_epoch()).count();

    int real_tuck_state = 0;
    float real_tuck_blend = 0.0f;
    {
        std::lock_guard<std::mutex> lock(this->real_tuck_mutex);
        real_tuck_state = static_cast<int>(this->real_tuck_state);
        real_tuck_blend = this->real_tuck_blend;
    }
    int real_gain_fault_state = 0;
    int real_gain_fault_policy_index = -1;
    int real_gain_fault_hardware_index = -1;
    float real_gain_fault_alpha = 1.0f;
    {
        std::lock_guard<std::mutex> lock(this->real_gain_fault_mutex);
        real_gain_fault_state = static_cast<int>(this->real_gain_fault_state);
        real_gain_fault_policy_index = this->real_gain_fault_policy_index;
        real_gain_fault_hardware_index = this->real_gain_fault_hardware_index;
        real_gain_fault_alpha = this->real_gain_fault_current_alpha;
    }

    this->policy_debug_csv_file
        << this->episode_length_buf << ","
        << wall_time_s << ","
        << this->unitree_low_state.tick() << ","
        << real_tuck_state << ","
        << real_tuck_blend << ","
        << this->LowStateAgeSeconds() << ","
        << real_gain_fault_state << ","
        << real_gain_fault_policy_index << ","
        << real_gain_fault_hardware_index << ","
        << real_gain_fault_alpha << ","
        << this->control.x << ","
        << this->control.y << ","
        << this->control.yaw << ","
        << value_at(this->obs.commands, 0) << ","
        << value_at(this->obs.commands, 1) << ","
        << value_at(this->obs.commands, 2) << ","
        << value_at(this->obs.lin_vel, 0) << ","
        << value_at(this->obs.lin_vel, 1) << ","
        << value_at(this->obs.lin_vel, 2) << ","
        << value_at(this->obs.ang_vel, 0) << ","
        << value_at(this->obs.ang_vel, 1) << ","
        << value_at(this->obs.ang_vel, 2) << ","
        << value_at(gravity_body, 0) << ","
        << value_at(gravity_body, 1) << ","
        << value_at(gravity_body, 2) << ","
        << value_at(this->obs.base_quat, 0) << ","
        << value_at(this->obs.base_quat, 1) << ","
        << value_at(this->obs.base_quat, 2) << ","
        << value_at(this->obs.base_quat, 3) << ","
        << this->joystick.lx() << ","
        << this->joystick.ly() << ","
        << this->joystick.rx() << ","
        << this->joystick.ry() << ","
        << this->joystick.keys() << ","
        << this->unitree_low_state.foot_force()[0] << ","
        << this->unitree_low_state.foot_force()[1] << ","
        << this->unitree_low_state.foot_force()[2] << ","
        << this->unitree_low_state.foot_force()[3] << ","
        << this->unitree_low_state.foot_force_est()[0] << ","
        << this->unitree_low_state.foot_force_est()[1] << ","
        << this->unitree_low_state.foot_force_est()[2] << ","
        << this->unitree_low_state.foot_force_est()[3];

    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    const int num_dofs = this->params.Get<int>("num_of_dofs");
    for (int i = 0; i < num_dofs; ++i)
    {
        const int lowcmd_index = (i < static_cast<int>(joint_mapping.size())) ? joint_mapping[i] : i;
        const bool valid_lowcmd_index = lowcmd_index >= 0 && lowcmd_index < static_cast<int>(this->unitree_low_command.motor_cmd().size());
        const auto& motor_cmd = valid_lowcmd_index ? this->unitree_low_command.motor_cmd()[lowcmd_index] : this->unitree_low_command.motor_cmd()[0];

        const float action = value_at(this->obs.actions, i);
        const float target_q = value_at(this->output_dof_pos, i);
        const float target_dq = value_at(this->output_dof_vel, i);
        const float target_tau = value_at(this->output_dof_tau, i);
        const float actual_q = value_at(this->robot_state.motor_state.q, i);
        const float actual_dq = value_at(this->robot_state.motor_state.dq, i);
        const float actual_tau_est = value_at(this->robot_state.motor_state.tau_est, i);
        const float motor_temperature = valid_lowcmd_index
            ? static_cast<float>(this->unitree_low_state.motor_state()[lowcmd_index].temperature())
            : 0.0f;
        const float robot_cmd_q = value_at(this->robot_command.motor_command.q, i);
        const float robot_cmd_dq = value_at(this->robot_command.motor_command.dq, i);
        const float robot_cmd_kp = value_at(this->robot_command.motor_command.kp, i);
        const float robot_cmd_kd = value_at(this->robot_command.motor_command.kd, i);
        const float robot_cmd_tau = value_at(this->robot_command.motor_command.tau, i);

        this->policy_debug_csv_file
            << "," << action
            << "," << target_q
            << "," << target_dq
            << "," << target_tau
            << "," << actual_q
            << "," << actual_dq
            << "," << actual_tau_est
            << "," << motor_temperature
            << "," << robot_cmd_q
            << "," << robot_cmd_dq
            << "," << robot_cmd_kp
            << "," << robot_cmd_kd
            << "," << robot_cmd_tau
            << "," << motor_cmd.q()
            << "," << motor_cmd.dq()
            << "," << motor_cmd.kp()
            << "," << motor_cmd.kd()
            << "," << motor_cmd.tau();
    }
    this->policy_debug_csv_file << "\n";

    if ((this->policy_debug_csv_counter % 25) == 0)
    {
        this->policy_debug_csv_file.flush();
    }
}

void RL_Real::ClosePolicyDebugCsv()
{
    if (this->policy_debug_csv_file.is_open())
    {
        this->policy_debug_csv_file.flush();
        this->policy_debug_csv_file.close();
    }
}

double RL_Real::LowStateAgeSeconds() const
{
    const int64_t last_message_ns = this->last_lowstate_time_ns.load();
    if (last_message_ns <= 0)
    {
        return std::numeric_limits<double>::infinity();
    }
    return static_cast<double>(SteadyClockNowNanoseconds() - last_message_ns) * 1.0e-9;
}

bool RL_Real::CheckRealTuckArmConditions(std::string& reason) const
{
    if (!this->real_tuck_launch_enabled)
    {
        reason = "launch gate RL_SAR_ENABLE_REAL_TUCK_TEST=1 is missing";
        return false;
    }
    if (!this->params.Get<bool>("real_tucked_leg_fault_enabled", false))
    {
        reason = "selected policy config does not enable the real tuck test";
        return false;
    }
    const std::string required_profile = this->params.Get<std::string>(
        "real_tuck_required_policy_config", "fault_history_b5");
    if (this->config_name != required_profile)
    {
        reason = "active policy is '" + this->config_name + "', expected '" + required_profile + "'";
        return false;
    }
    if (!this->fsm.current_state_ ||
        this->fsm.current_state_->GetStateName() != "RLFSMStateRLLocomotion" ||
        !this->rl_init_done)
    {
        reason = "B5 policy locomotion is not fully active";
        return false;
    }

    const float command_threshold = this->params.Get<float>("real_tuck_arm_command_threshold", 0.03f);
    if (std::abs(this->control.x) > command_threshold ||
        std::abs(this->control.y) > command_threshold ||
        std::abs(this->control.yaw) > command_threshold)
    {
        reason = "center the physical controller sticks before arming or triggering";
        return false;
    }

    const double lowstate_timeout = this->params.Get<double>("real_tuck_lowstate_timeout_s", 0.10);
    if (this->LowStateAgeSeconds() > lowstate_timeout)
    {
        reason = "LowState is stale";
        return false;
    }

    if (this->robot_state.imu.quaternion.size() != 4)
    {
        reason = "IMU quaternion is unavailable";
        return false;
    }
    const auto euler = QuaternionToEuler(this->robot_state.imu.quaternion);
    const float arm_tilt_deg = this->params.Get<float>("real_tuck_arm_tilt_deg", 15.0f);
    if (euler.size() < 2 ||
        !std::isfinite(euler[0]) || !std::isfinite(euler[1]) ||
        std::abs(euler[0] * kRadiansToDegrees) > arm_tilt_deg ||
        std::abs(euler[1] * kRadiansToDegrees) > arm_tilt_deg)
    {
        reason = "robot tilt exceeds the arming limit";
        return false;
    }
    return true;
}

bool RL_Real::ResolveRealTuckJoints(std::string& reason)
{
    const auto requested_names = this->params.Get<std::vector<std::string>>(
        "real_tuck_leg_joint_names",
        {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"});
    const auto joint_names = this->params.Get<std::vector<std::string>>("joint_names");
    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    const auto default_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    const auto offsets = this->params.Get<std::vector<float>>("real_tuck_joint_offsets");

    if (requested_names.size() != 3 || offsets.size() != 3)
    {
        reason = "real tuck test requires exactly three joint names and offsets";
        return false;
    }

    this->real_tuck_joint_indices.clear();
    this->real_tuck_start_q.clear();
    this->real_tuck_target_q.clear();
    for (size_t requested_index = 0; requested_index < requested_names.size(); ++requested_index)
    {
        int policy_index = -1;
        for (size_t i = 0; i < joint_mapping.size(); ++i)
        {
            const int hardware_index = joint_mapping[i];
            if (hardware_index >= 0 && hardware_index < static_cast<int>(joint_names.size()) &&
                joint_names[hardware_index] == requested_names[requested_index])
            {
                policy_index = static_cast<int>(i);
                break;
            }
        }
        if (policy_index < 0 ||
            policy_index >= static_cast<int>(this->robot_state.motor_state.q.size()) ||
            policy_index >= static_cast<int>(default_dof_pos.size()))
        {
            reason = "could not resolve policy index for " + requested_names[requested_index];
            return false;
        }

        this->real_tuck_joint_indices.push_back(policy_index);
        this->real_tuck_start_q.push_back(this->robot_state.motor_state.q[policy_index]);
        this->real_tuck_target_q.push_back(default_dof_pos[policy_index] + offsets[requested_index]);
    }
    return true;
}

void RL_Real::HandleRealTuckControlInput()
{
    if (!this->real_tuck_launch_enabled)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(this->real_tuck_mutex);
    const auto key = this->control.current_keyboard;
    const bool locomotion_active = this->fsm.current_state_ &&
        this->fsm.current_state_->GetStateName() == "RLFSMStateRLLocomotion";

    if ((this->real_tuck_state == RealTuckState::Armed ||
         this->real_tuck_state == RealTuckState::Tucking ||
         this->real_tuck_state == RealTuckState::Locked) &&
        (!locomotion_active || key == Input::Keyboard::P))
    {
        this->real_tuck_state = RealTuckState::Aborted;
        this->real_tuck_abort_reason = key == Input::Keyboard::P
            ? "manual passive request" : "left policy locomotion";
        this->real_tuck_blend = 0.0f;
        std::cout << std::endl << LOGGER::WARNING
                  << "[REAL TUCK] Aborted: " << this->real_tuck_abort_reason << std::endl;
        return;
    }

    if (this->real_tuck_state == RealTuckState::Armed &&
        std::chrono::steady_clock::now() > this->real_tuck_arm_deadline)
    {
        this->real_tuck_state = RealTuckState::Idle;
        std::cout << std::endl << LOGGER::INFO << "[REAL TUCK] Arm window expired." << std::endl;
    }

    if (key == Input::Keyboard::F)
    {
        if (this->real_tuck_state == RealTuckState::Aborted ||
            this->real_tuck_state == RealTuckState::Tucking ||
            this->real_tuck_state == RealTuckState::Locked)
        {
            std::cout << std::endl << LOGGER::WARNING
                      << "[REAL TUCK] Restart the program before another test." << std::endl;
            return;
        }

        std::string reason;
        if (!this->CheckRealTuckArmConditions(reason))
        {
            std::cout << std::endl << LOGGER::WARNING
                      << "[REAL TUCK] Cannot arm: " << reason << std::endl;
            return;
        }

        const float arm_timeout_s = this->params.Get<float>("real_tuck_arm_timeout_s", 5.0f);
        this->real_tuck_arm_deadline = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(static_cast<int>(arm_timeout_s * 1000.0f));
        this->real_tuck_state = RealTuckState::Armed;
        std::cout << std::endl << LOGGER::WARNING
                  << "[REAL TUCK] ARMED for " << arm_timeout_s
                  << " s. Keep sticks centered; press T to trigger or P to abort." << std::endl;
        return;
    }

    if (key == Input::Keyboard::T)
    {
        if (this->real_tuck_state != RealTuckState::Armed)
        {
            std::cout << std::endl << LOGGER::WARNING
                      << "[REAL TUCK] Ignored T: press F and satisfy the arm checks first." << std::endl;
            return;
        }

        std::string reason;
        if (!this->CheckRealTuckArmConditions(reason) || !this->ResolveRealTuckJoints(reason))
        {
            this->real_tuck_state = RealTuckState::Idle;
            std::cout << std::endl << LOGGER::WARNING
                      << "[REAL TUCK] Trigger rejected: " << reason << std::endl;
            return;
        }

        this->real_tuck_elapsed_s = 0.0f;
        this->real_tuck_locked_elapsed_s = 0.0f;
        this->real_tuck_blend = 0.0f;
        this->real_tuck_violation_elapsed_s = 0.0f;
        this->real_tuck_abort_reason.clear();
        this->real_tuck_state = RealTuckState::Tucking;
        std::cout << std::endl << LOGGER::WARNING
                  << "[REAL TUCK] TRIGGERED. Folding and holding FR leg; P aborts to passive."
                  << std::endl;
    }
}

void RL_Real::ApplyRealTuckCommandGuard()
{
    if (!this->real_tuck_launch_enabled)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(this->real_tuck_mutex);
    if (this->real_tuck_state == RealTuckState::Tucking)
    {
        this->obs.commands = {0.0f, 0.0f, 0.0f};
        return;
    }
    if (this->real_tuck_state != RealTuckState::Locked || this->obs.commands.size() < 3)
    {
        return;
    }

    const auto max_commands = this->params.Get<std::vector<float>>(
        "real_tuck_max_commands", {0.15f, 0.10f, 0.15f});
    if (max_commands.size() != 3)
    {
        return;
    }
    for (size_t i = 0; i < 3; ++i)
    {
        this->obs.commands[i] = std::clamp(this->obs.commands[i], -max_commands[i], max_commands[i]);
    }
}

void RL_Real::RequestRealTuckAbort(const std::string& reason)
{
    this->real_tuck_state = RealTuckState::Aborted;
    this->real_tuck_abort_reason = reason;
    this->real_tuck_blend = 0.0f;
    this->real_tuck_abort_requested.store(true);
    std::cout << std::endl << LOGGER::ERROR
              << "[REAL TUCK] SAFETY ABORT: " << reason << ". Requesting passive mode."
              << std::endl;
}

void RL_Real::ApplyRealTuckedLegFault()
{
    if (!this->real_tuck_launch_enabled)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(this->real_tuck_mutex);
    if (this->real_tuck_state != RealTuckState::Tucking &&
        this->real_tuck_state != RealTuckState::Locked)
    {
        return;
    }

    const float policy_period_s = this->params.Get<float>("dt") * this->params.Get<int>("decimation");
    const double lowstate_timeout_s = this->params.Get<double>("real_tuck_lowstate_timeout_s", 0.10);
    if (this->LowStateAgeSeconds() > lowstate_timeout_s)
    {
        this->RequestRealTuckAbort("LowState timeout");
        return;
    }

    const auto euler = QuaternionToEuler(this->robot_state.imu.quaternion);
    const float abort_tilt_deg = this->params.Get<float>("real_tuck_abort_tilt_deg", 30.0f);
    if (euler.size() < 2 ||
        !std::isfinite(euler[0]) || !std::isfinite(euler[1]) ||
        std::abs(euler[0] * kRadiansToDegrees) > abort_tilt_deg ||
        std::abs(euler[1] * kRadiansToDegrees) > abort_tilt_deg)
    {
        this->RequestRealTuckAbort("roll or pitch exceeded limit");
        return;
    }

    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    const float max_temperature_c = this->params.Get<float>("real_tuck_max_motor_temperature_c", 70.0f);
    for (int hardware_index : joint_mapping)
    {
        if (hardware_index >= 0 &&
            hardware_index < static_cast<int>(this->unitree_low_state.motor_state().size()) &&
            this->unitree_low_state.motor_state()[hardware_index].temperature() > max_temperature_c)
        {
            this->RequestRealTuckAbort("motor temperature exceeded limit");
            return;
        }
    }

    if (this->real_tuck_state == RealTuckState::Tucking)
    {
        const float duration_s = std::max(0.05f, this->params.Get<float>("real_tuck_duration_s", 0.75f));
        this->real_tuck_elapsed_s += policy_period_s;
        this->real_tuck_blend = std::clamp(this->real_tuck_elapsed_s / duration_s, 0.0f, 1.0f);
    }
    else
    {
        this->real_tuck_blend = 1.0f;
        this->real_tuck_locked_elapsed_s += policy_period_s;
    }

    const auto rl_kp = this->params.Get<std::vector<float>>("rl_kp");
    const auto rl_kd = this->params.Get<std::vector<float>>("rl_kd");
    const auto torque_limits = this->params.Get<std::vector<float>>("torque_limits");
    for (size_t joint = 0; joint < this->real_tuck_joint_indices.size(); ++joint)
    {
        const int index = this->real_tuck_joint_indices[joint];
        if (index < 0 || index >= static_cast<int>(this->output_dof_pos.size()))
        {
            this->RequestRealTuckAbort("tuck output index became invalid");
            return;
        }
        const float target = this->real_tuck_start_q[joint] +
            this->real_tuck_blend * (this->real_tuck_target_q[joint] - this->real_tuck_start_q[joint]);
        this->output_dof_pos[index] = target;
        if (index < static_cast<int>(this->output_dof_vel.size()))
        {
            this->output_dof_vel[index] = 0.0f;
        }
        if (index < static_cast<int>(this->output_dof_tau.size()) &&
            index < static_cast<int>(rl_kp.size()) &&
            index < static_cast<int>(rl_kd.size()) &&
            index < static_cast<int>(torque_limits.size()))
        {
            const float estimated_command_tau =
                rl_kp[index] * (target - this->obs.dof_pos[index]) - rl_kd[index] * this->obs.dof_vel[index];
            this->output_dof_tau[index] = std::clamp(
                estimated_command_tau, -torque_limits[index], torque_limits[index]);
        }
    }

    if (this->real_tuck_state == RealTuckState::Tucking && this->real_tuck_blend >= 1.0f)
    {
        this->real_tuck_state = RealTuckState::Locked;
        this->real_tuck_locked_elapsed_s = 0.0f;
        std::cout << std::endl << LOGGER::WARNING
                  << "[REAL TUCK] FR leg is folded and held. Use only gentle hand-controller commands."
                  << std::endl;
    }

    std::string soft_violation;
    const float max_joint_velocity = this->params.Get<float>("real_tuck_max_joint_velocity", 15.0f);
    const float max_torque = this->params.Get<float>("real_tuck_max_torque", 23.0f);
    for (size_t i = 0; i < this->robot_state.motor_state.dq.size(); ++i)
    {
        if (!std::isfinite(this->robot_state.motor_state.dq[i]) ||
            !std::isfinite(this->robot_state.motor_state.tau_est[i]))
        {
            this->RequestRealTuckAbort("non-finite motor state");
            return;
        }
        if (std::abs(this->robot_state.motor_state.dq[i]) > max_joint_velocity)
        {
            soft_violation = "joint velocity exceeded limit";
            break;
        }
        if (std::abs(this->robot_state.motor_state.tau_est[i]) > max_torque)
        {
            soft_violation = "estimated motor torque exceeded limit";
            break;
        }
    }

    const float tracking_grace_s = this->params.Get<float>("real_tuck_tracking_grace_s", 0.25f);
    if (soft_violation.empty() &&
        this->real_tuck_state == RealTuckState::Locked &&
        this->real_tuck_locked_elapsed_s >= tracking_grace_s)
    {
        const float max_tracking_error = this->params.Get<float>("real_tuck_max_tracking_error", 0.45f);
        for (size_t joint = 0; joint < this->real_tuck_joint_indices.size(); ++joint)
        {
            const int index = this->real_tuck_joint_indices[joint];
            if (std::abs(this->real_tuck_target_q[joint] - this->obs.dof_pos[index]) > max_tracking_error)
            {
                soft_violation = "folded-leg tracking error exceeded limit";
                break;
            }
        }
    }

    if (soft_violation.empty())
    {
        this->real_tuck_violation_elapsed_s = 0.0f;
        return;
    }

    this->real_tuck_violation_elapsed_s += policy_period_s;
    const float hold_s = this->params.Get<float>("real_tuck_violation_hold_s", 0.15f);
    if (this->real_tuck_violation_elapsed_s >= hold_s)
    {
        this->RequestRealTuckAbort(soft_violation);
    }
}

bool RL_Real::CheckRealGainFaultArmConditions(std::string& reason) const
{
    if (!this->real_gain_fault_launch_enabled)
    {
        reason = "launch gate RL_SAR_ENABLE_REAL_GAIN_FAULT_TEST=1 is missing";
        return false;
    }
    if (!this->params.Get<bool>("real_gain_fault_enabled", false))
    {
        reason = "selected policy config does not enable the real gain-fault test";
        return false;
    }

    const auto allowed_configs = this->params.Get<std::vector<std::string>>(
        "real_gain_fault_allowed_policy_configs",
        {"adaptive_fault_d2", "adaptive_fault_d22"});
    if (std::find(allowed_configs.begin(), allowed_configs.end(), this->config_name) == allowed_configs.end())
    {
        reason = "active policy '" + this->config_name + "' is not approved for this test";
        return false;
    }
    if (!this->fsm.current_state_ ||
        this->fsm.current_state_->GetStateName() != "RLFSMStateRLLocomotion" ||
        !this->rl_init_done)
    {
        reason = "adaptive policy locomotion is not fully active";
        return false;
    }

    const float command_threshold = this->params.Get<float>(
        "real_gain_fault_arm_command_threshold", 0.03f);
    if (std::abs(this->control.x) > command_threshold ||
        std::abs(this->control.y) > command_threshold ||
        std::abs(this->control.yaw) > command_threshold)
    {
        reason = "center the physical controller sticks before arming or triggering";
        return false;
    }

    const double lowstate_timeout = this->params.Get<double>(
        "real_gain_fault_lowstate_timeout_s", 0.10);
    if (this->LowStateAgeSeconds() > lowstate_timeout)
    {
        reason = "LowState is stale";
        return false;
    }
    if (this->robot_state.imu.quaternion.size() != 4)
    {
        reason = "IMU quaternion is unavailable";
        return false;
    }

    const auto euler = QuaternionToEuler(this->robot_state.imu.quaternion);
    const float arm_tilt_deg = this->params.Get<float>("real_gain_fault_arm_tilt_deg", 12.0f);
    if (euler.size() < 2 ||
        !std::isfinite(euler[0]) || !std::isfinite(euler[1]) ||
        std::abs(euler[0] * kRadiansToDegrees) > arm_tilt_deg ||
        std::abs(euler[1] * kRadiansToDegrees) > arm_tilt_deg)
    {
        reason = "robot tilt exceeds the arming limit";
        return false;
    }
    return true;
}

bool RL_Real::SelectRealGainFaultJoint(std::string& reason)
{
    struct Candidate
    {
        std::string name;
        int policy_index;
        int hardware_index;
    };

    const auto requested_names = this->params.Get<std::vector<std::string>>(
        "real_gain_fault_candidate_joint_names",
        {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
         "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
         "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"});
    const auto joint_names = this->params.Get<std::vector<std::string>>("joint_names");
    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");

    std::vector<Candidate> candidates;
    for (const auto& requested_name : requested_names)
    {
        // The first guarded hardware campaign intentionally excludes the FR leg.
        if (requested_name.rfind("FR_", 0) == 0)
        {
            continue;
        }
        for (size_t policy_index = 0; policy_index < joint_mapping.size(); ++policy_index)
        {
            const int hardware_index = joint_mapping[policy_index];
            if (hardware_index >= 0 && hardware_index < static_cast<int>(joint_names.size()) &&
                joint_names[hardware_index] == requested_name)
            {
                candidates.push_back(
                    {requested_name, static_cast<int>(policy_index), hardware_index});
                break;
            }
        }
    }
    if (candidates.empty())
    {
        reason = "no valid non-FR candidate joints were configured";
        return false;
    }

    const auto seed = static_cast<unsigned int>(SteadyClockNowNanoseconds());
    std::mt19937 generator(seed);
    std::uniform_int_distribution<size_t> distribution(0, candidates.size() - 1);
    const Candidate& selected = candidates[distribution(generator)];
    this->real_gain_fault_joint_name = selected.name;
    this->real_gain_fault_policy_index = selected.policy_index;
    this->real_gain_fault_hardware_index = selected.hardware_index;

    const float minimum_alpha = std::clamp(
        this->params.Get<float>("real_gain_fault_min_alpha", 0.20f), 0.20f, 0.95f);
    this->real_gain_fault_target_alpha = std::clamp(
        this->params.Get<float>("real_gain_fault_target_alpha", 0.50f),
        minimum_alpha,
        0.95f);
    return true;
}

void RL_Real::RestoreRealGainFault(const std::string& reason, bool request_getdown)
{
    this->SetGainAlpha(std::vector<float>(this->params.Get<int>("num_of_dofs"), 1.0f));
    this->real_gain_fault_current_alpha = 1.0f;
    this->real_gain_fault_violation_elapsed_s = 0.0f;
    this->real_gain_fault_abort_reason = reason;
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
    if (this->obs.commands.size() >= 3)
    {
        this->obs.commands = {0.0f, 0.0f, 0.0f};
    }

    if (reason == "test duration complete")
    {
        this->real_gain_fault_state = RealGainFaultState::Completed;
        if (request_getdown)
        {
            this->real_gain_fault_getdown_requested.store(true);
        }
        std::cout << std::endl << LOGGER::INFO
                  << "[REAL GAIN FAULT] Test complete; full joint authority restored"
                  << (request_getdown ? "; requesting GetDown." : ".")
                  << std::endl;
        return;
    }

    this->real_gain_fault_state = RealGainFaultState::Aborted;
    if (request_getdown)
    {
        this->real_gain_fault_getdown_requested.store(true);
    }
    std::cout << std::endl << LOGGER::ERROR
              << "[REAL GAIN FAULT] SAFETY ABORT: " << reason
              << ". Full authority restored"
              << (request_getdown ? "; requesting GetDown." : ".")
              << std::endl;
}

void RL_Real::HandleRealGainFaultControlInput()
{
    if (!this->real_gain_fault_launch_enabled)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(this->real_gain_fault_mutex);
    const auto key = this->control.current_keyboard;
    const bool locomotion_active = this->fsm.current_state_ &&
        this->fsm.current_state_->GetStateName() == "RLFSMStateRLLocomotion";

    if ((this->real_gain_fault_state == RealGainFaultState::Armed ||
         this->real_gain_fault_state == RealGainFaultState::Ramping ||
         this->real_gain_fault_state == RealGainFaultState::Active) &&
        (!locomotion_active || key == Input::Keyboard::P))
    {
        const bool manual_abort = key == Input::Keyboard::P;
        this->RestoreRealGainFault(
            manual_abort ? "manual abort" : "left policy locomotion",
            manual_abort);
        return;
    }

    if (this->real_gain_fault_state == RealGainFaultState::Armed &&
        std::chrono::steady_clock::now() > this->real_gain_fault_arm_deadline)
    {
        this->real_gain_fault_state = RealGainFaultState::Idle;
        this->real_gain_fault_policy_index = -1;
        this->real_gain_fault_hardware_index = -1;
        this->real_gain_fault_joint_name.clear();
        std::cout << std::endl << LOGGER::INFO
                  << "[REAL GAIN FAULT] Arm window expired." << std::endl;
    }

    if (key == Input::Keyboard::F)
    {
        // Treat arming as a one-shot operation. Terminal key repeat can emit
        // several F characters from one physical press; accepting another one
        // would silently replace the selected joint.
        if (this->real_gain_fault_arm_attempted)
        {
            return;
        }
        this->real_gain_fault_arm_attempted = true;

        if (this->real_gain_fault_state != RealGainFaultState::Idle)
        {
            std::cerr << std::endl << LOGGER::WARNING
                      << "[REAL GAIN FAULT] Restart the program before another test."
                      << std::endl;
            return;
        }

        std::string reason;
        if (!this->CheckRealGainFaultArmConditions(reason) ||
            !this->SelectRealGainFaultJoint(reason))
        {
            std::cerr << std::endl << LOGGER::WARNING
                      << "[REAL GAIN FAULT] Cannot arm: " << reason << std::endl;
            return;
        }

        const float arm_timeout_s = this->params.Get<float>(
            "real_gain_fault_arm_timeout_s", 5.0f);
        this->real_gain_fault_arm_deadline = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(static_cast<int>(arm_timeout_s * 1000.0f));
        this->real_gain_fault_state = RealGainFaultState::Armed;
        std::cerr << std::endl << LOGGER::WARNING
                  << "[REAL GAIN FAULT] ARMED " << this->real_gain_fault_joint_name
                  << " at alpha=" << this->real_gain_fault_target_alpha
                  << " for " << arm_timeout_s
                  << " s. Keep sticks centered; press T to trigger or P to GetDown."
                  << std::endl;
        return;
    }

    if (key == Input::Keyboard::T)
    {
        // A held T key can also auto-repeat. Once the ramp has started, those
        // repeats are not new trigger requests.
        if (this->real_gain_fault_state == RealGainFaultState::Ramping ||
            this->real_gain_fault_state == RealGainFaultState::Active)
        {
            return;
        }
        if (this->real_gain_fault_state != RealGainFaultState::Armed)
        {
            std::cerr << std::endl << LOGGER::WARNING
                      << "[REAL GAIN FAULT] Ignored T: press F and satisfy the arm checks first."
                      << std::endl;
            return;
        }

        std::string reason;
        if (!this->CheckRealGainFaultArmConditions(reason))
        {
            this->real_gain_fault_state = RealGainFaultState::Idle;
            std::cerr << std::endl << LOGGER::WARNING
                      << "[REAL GAIN FAULT] Trigger rejected: " << reason << std::endl;
            return;
        }

        this->real_gain_fault_ramp_elapsed_s = 0.0f;
        this->real_gain_fault_active_elapsed_s = 0.0f;
        this->real_gain_fault_violation_elapsed_s = 0.0f;
        this->real_gain_fault_current_alpha = 1.0f;
        this->real_gain_fault_abort_reason.clear();
        this->SetGainAlpha(std::vector<float>(this->params.Get<int>("num_of_dofs"), 1.0f));
        this->real_gain_fault_state = RealGainFaultState::Ramping;
        std::cerr << std::endl << LOGGER::WARNING
                  << "[REAL GAIN FAULT] TRIGGERED " << this->real_gain_fault_joint_name
                  << ". Ramping to alpha=" << this->real_gain_fault_target_alpha
                  << "; P restores authority and requests GetDown."
                  << std::endl;
    }
}

void RL_Real::ApplyRealGainFaultCommandGuard()
{
    if (!this->real_gain_fault_launch_enabled)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(this->real_gain_fault_mutex);
    if (this->real_gain_fault_state == RealGainFaultState::Ramping)
    {
        this->obs.commands = {0.0f, 0.0f, 0.0f};
        return;
    }
    if (this->real_gain_fault_state != RealGainFaultState::Active ||
        this->obs.commands.size() < 3)
    {
        return;
    }

    const auto max_commands = this->params.Get<std::vector<float>>(
        "real_gain_fault_max_commands", {0.15f, 0.10f, 0.15f});
    if (max_commands.size() != 3)
    {
        return;
    }
    for (size_t i = 0; i < 3; ++i)
    {
        this->obs.commands[i] = std::clamp(this->obs.commands[i], -max_commands[i], max_commands[i]);
    }
}

void RL_Real::UpdateRealGainFault()
{
    if (!this->real_gain_fault_launch_enabled)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(this->real_gain_fault_mutex);
    if (this->real_gain_fault_state != RealGainFaultState::Ramping &&
        this->real_gain_fault_state != RealGainFaultState::Active)
    {
        return;
    }

    const float policy_period_s = this->params.Get<float>("dt") * this->params.Get<int>("decimation");
    const double lowstate_timeout_s = this->params.Get<double>(
        "real_gain_fault_lowstate_timeout_s", 0.10);
    if (this->LowStateAgeSeconds() > lowstate_timeout_s)
    {
        this->RestoreRealGainFault("LowState timeout", true);
        return;
    }

    const auto euler = QuaternionToEuler(this->robot_state.imu.quaternion);
    const float abort_tilt_deg = this->params.Get<float>(
        "real_gain_fault_abort_tilt_deg", 25.0f);
    if (euler.size() < 2 ||
        !std::isfinite(euler[0]) || !std::isfinite(euler[1]) ||
        std::abs(euler[0] * kRadiansToDegrees) > abort_tilt_deg ||
        std::abs(euler[1] * kRadiansToDegrees) > abort_tilt_deg)
    {
        this->RestoreRealGainFault("roll or pitch exceeded limit", true);
        return;
    }

    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    const float max_temperature_c = this->params.Get<float>(
        "real_gain_fault_max_motor_temperature_c", 70.0f);
    for (int hardware_index : joint_mapping)
    {
        if (hardware_index >= 0 &&
            hardware_index < static_cast<int>(this->unitree_low_state.motor_state().size()) &&
            this->unitree_low_state.motor_state()[hardware_index].temperature() > max_temperature_c)
        {
            this->RestoreRealGainFault("motor temperature exceeded limit", true);
            return;
        }
    }

    std::string soft_violation;
    const float max_joint_velocity = this->params.Get<float>(
        "real_gain_fault_max_joint_velocity", 15.0f);
    const float max_torque = this->params.Get<float>("real_gain_fault_max_torque", 32.0f);
    for (size_t i = 0; i < this->robot_state.motor_state.dq.size(); ++i)
    {
        if (!std::isfinite(this->robot_state.motor_state.dq[i]) ||
            !std::isfinite(this->robot_state.motor_state.tau_est[i]))
        {
            this->RestoreRealGainFault("non-finite motor state", true);
            return;
        }
        if (std::abs(this->robot_state.motor_state.dq[i]) > max_joint_velocity)
        {
            soft_violation = "joint velocity exceeded limit";
            break;
        }
        if (std::abs(this->robot_state.motor_state.tau_est[i]) > max_torque)
        {
            soft_violation = "estimated motor torque exceeded limit";
            break;
        }
    }

    if (soft_violation.empty())
    {
        this->real_gain_fault_violation_elapsed_s = 0.0f;
    }
    else
    {
        this->real_gain_fault_violation_elapsed_s += policy_period_s;
        const float hold_s = this->params.Get<float>(
            "real_gain_fault_violation_hold_s", 0.10f);
        if (this->real_gain_fault_violation_elapsed_s >= hold_s)
        {
            this->RestoreRealGainFault(soft_violation, true);
            return;
        }
    }

    if (this->real_gain_fault_state == RealGainFaultState::Ramping)
    {
        const float ramp_s = std::max(1.0f, this->params.Get<float>("real_gain_fault_ramp_s", 3.0f));
        this->real_gain_fault_ramp_elapsed_s += policy_period_s;
        const float blend = std::clamp(this->real_gain_fault_ramp_elapsed_s / ramp_s, 0.0f, 1.0f);
        this->real_gain_fault_current_alpha =
            1.0f + blend * (this->real_gain_fault_target_alpha - 1.0f);
        if (blend >= 1.0f)
        {
            this->real_gain_fault_state = RealGainFaultState::Active;
            this->real_gain_fault_active_elapsed_s = 0.0f;
            std::cout << std::endl << LOGGER::WARNING
                      << "[REAL GAIN FAULT] " << this->real_gain_fault_joint_name
                      << " reached alpha=" << this->real_gain_fault_current_alpha
                      << ". Use only gentle controller commands."
                      << std::endl;
        }
    }
    else
    {
        this->real_gain_fault_active_elapsed_s += policy_period_s;
    }

    std::vector<float> alpha(this->params.Get<int>("num_of_dofs"), 1.0f);
    if (this->real_gain_fault_policy_index < 0 ||
        this->real_gain_fault_policy_index >= static_cast<int>(alpha.size()))
    {
        this->RestoreRealGainFault("selected joint index became invalid", true);
        return;
    }
    alpha[this->real_gain_fault_policy_index] = this->real_gain_fault_current_alpha;
    this->SetGainAlpha(alpha);

    const float max_hold_s = std::max(
        1.0f, this->params.Get<float>("real_gain_fault_max_hold_s", 15.0f));
    if (this->real_gain_fault_state == RealGainFaultState::Active &&
        this->real_gain_fault_active_elapsed_s >= max_hold_s)
    {
        this->RestoreRealGainFault("test duration complete", true);
    }
}

std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    const PolicyModelInput model_input = this->BuildPolicyModelInput(clamped_obs);
    std::vector<float> actions = this->model->forward_with_shapes(model_input.values, model_input.shapes);

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Real::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->unitree_low_state.motor_state()[i].q());
        this->plot_target_joint_pos[i].push_back(this->unitree_low_command.motor_cmd()[i].q());
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.0001);
}

uint32_t RL_Real::Crc32Core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; ++i)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
            {
                CRC32 ^= dwPolynomial;
            }
            xbit >>= 1;
        }
    }

    return CRC32;
}

void RL_Real::InitLowCmd()
{
    this->unitree_low_command.head()[0] = 0xFE;
    this->unitree_low_command.head()[1] = 0xEF;
    this->unitree_low_command.level_flag() = 0xFF;
    this->unitree_low_command.gpio() = 0;

    for (int i = 0; i < 20; ++i)
    {
        this->unitree_low_command.motor_cmd()[i].mode() = (0x01); // motor switch to servo (PMSM) mode
        this->unitree_low_command.motor_cmd()[i].q() = (PosStopF);
        this->unitree_low_command.motor_cmd()[i].kp() = (0);
        this->unitree_low_command.motor_cmd()[i].dq() = (VelStopF);
        this->unitree_low_command.motor_cmd()[i].kd() = (0);
        this->unitree_low_command.motor_cmd()[i].tau() = (0);
    }
}

int RL_Real::QueryMotionStatus()
{
    std::string robotForm, motionName;
    int motionStatus;
    int32_t ret = this->msc.CheckMode(robotForm, motionName);
    if (ret == 0)
    {
        std::cout << "CheckMode succeeded." << std::endl;
    }
    else
    {
        std::cout << "CheckMode failed. Error code: " << ret << std::endl;
    }
    if (motionName.empty())
    {
        std::cout << "The motion control-related service is deactivated." << std::endl;
        motionStatus = 0;
    }
    else
    {
        std::string serviceName = QueryServiceName(robotForm, motionName);
        std::cout << "Service: " << serviceName << " is activate" << std::endl;
        motionStatus = 1;
    }
    return motionStatus;
}

std::string RL_Real::QueryServiceName(std::string form, std::string name)
{
    if (form == "0")
    {
        if (name == "normal" )   return "sport_mode";
        if (name == "ai" )       return "ai_sport";
        if (name == "advanced" ) return "advanced_sport";
    }
    else
    {
        if (name == "ai-w" )     return "wheeled_sport(go2W)";
        if (name == "normal-w" ) return "wheeled_sport(b2W)";
    }
    return "";
}

void RL_Real::LowStateMessageHandler(const void *message)
{
    this->unitree_low_state = *(unitree_go::msg::dds_::LowState_ *)message;
    this->joystick = go2_wireless_remote::Decode(this->unitree_low_state.wireless_remote());
    this->unitree_joy.value = this->joystick.keys();
    this->last_lowstate_time_ns.store(SteadyClockNowNanoseconds());
}

#if !defined(USE_CMAKE) && defined(USE_ROS)
void RL_Real::CmdvelCallback(
const geometry_msgs::msg::Twist::SharedPtr msg
)
{
    this->cmd_vel = *msg;
}
#endif

#if defined(USE_CMAKE) || !defined(USE_ROS)
// Signal handler for CMAKE mode
volatile sig_atomic_t g_shutdown_requested = 0;
void signalHandler(int signum)
{
    (void)signum;
    g_shutdown_requested = 1;
}
#endif

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " networkInterface" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }
    ChannelFactory::Instance()->Init(0, argv[1]);

    {
#if defined(USE_ROS)
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Real>(argc, argv);
    rclcpp::spin(rl_sar->ros2_node);
    rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    RL_Real rl_sar(argc, argv);
    while (!g_shutdown_requested) { sleep(1); }
    std::cout << LOGGER::INFO << "Shutdown requested." << std::endl;
    rl_sar.Shutdown();
#endif
    }

    ChannelFactory::Instance()->Release();
    return 0;
}
