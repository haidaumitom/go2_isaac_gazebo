/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_go2.hpp"

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
    this->joystick_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
    this->joystick_subscriber->InitChannel(std::bind(&RL_Real::JoystickHandler, this, std::placeholders::_1), 1);
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
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    this->ClosePolicyDebugCsv();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
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

void RL_Real::RobotControl()
{
    this->GetState(&this->robot_state);

    this->StateController(&this->robot_state, &this->robot_command);

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);

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
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

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

    this->policy_debug_csv_file
        << this->episode_length_buf << ","
        << wall_time_s << ","
        << this->unitree_low_state.tick() << ","
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

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

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
}

void RL_Real::JoystickHandler(const void *message)
{
    joystick = *(unitree_go::msg::dds_::WirelessController_ *)message;
    this->unitree_joy.value = joystick.keys();
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
    std::cout << LOGGER::INFO << "Received signal " << signum << ", shutting down..." << std::endl;
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

#if defined(USE_ROS)
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Real>(argc, argv);
    rclcpp::spin(rl_sar->ros2_node);
    rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    signal(SIGINT, signalHandler);
    RL_Real rl_sar(argc, argv);
    while (!g_shutdown_requested) { sleep(1); }
    std::cout << LOGGER::INFO << "Exiting..." << std::endl;
#endif

    return 0;
}
