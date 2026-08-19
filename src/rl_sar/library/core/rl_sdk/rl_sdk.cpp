#include "rl_sdk.hpp"

#include <cctype>
#include <cmath>

namespace
{
void MergeYamlMap(YAML::Node& destination, const YAML::Node& source)
{
    if (!source || !source.IsMap())
    {
        throw std::runtime_error("Expected a YAML mapping");
    }

    for (auto it = source.begin(); it != source.end(); ++it)
    {
        destination[it->first.as<std::string>()] = it->second;
    }
}

bool IsSafeProfileName(const std::string& profile_name)
{
    return !profile_name.empty() && std::all_of(
        profile_name.begin(), profile_name.end(), [](unsigned char character) {
            return std::isalnum(character) || character == '_' || character == '-';
        });
}
}

void RL::StateController(const RobotState<float>* state, RobotCommand<float>* command)
{
    constexpr float linear_command_step = 0.05f;
    constexpr float yaw_command_step = 0.05f;
    constexpr float max_linear_command = 1.0f;
    constexpr float max_yaw_command = 0.50f;

    auto updateState = [&](std::shared_ptr<FSMState> statePtr)
    {
        if (auto rl_fsm_state = std::dynamic_pointer_cast<RLFSMState>(statePtr))
        {
            rl_fsm_state->fsm_state = state;
            rl_fsm_state->fsm_command = command;
        }
    };
    for (auto& pair : fsm.states_)
    {
        updateState(pair.second);
    }

    fsm.Run();

    this->motiontime++;

    if (this->control.current_keyboard == Input::Keyboard::W)
    {
        this->control.x = std::clamp(this->control.x + linear_command_step, -max_linear_command, max_linear_command);
    }
    if (this->control.current_keyboard == Input::Keyboard::S)
    {
        this->control.x = std::clamp(this->control.x - linear_command_step, -max_linear_command, max_linear_command);
    }
    if (this->control.current_keyboard == Input::Keyboard::A)
    {
        this->control.y = std::clamp(this->control.y + linear_command_step, -max_linear_command, max_linear_command);
    }
    if (this->control.current_keyboard == Input::Keyboard::D)
    {
        this->control.y = std::clamp(this->control.y - linear_command_step, -max_linear_command, max_linear_command);
    }
    if (this->control.current_keyboard == Input::Keyboard::Q)
    {
        this->control.yaw = std::clamp(this->control.yaw + yaw_command_step, -max_yaw_command, max_yaw_command);
    }
    if (this->control.current_keyboard == Input::Keyboard::E)
    {
        this->control.yaw = std::clamp(this->control.yaw - yaw_command_step, -max_yaw_command, max_yaw_command);
    }
    if (this->control.current_keyboard == Input::Keyboard::Space)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
    }
    if (this->control.current_keyboard == Input::Keyboard::N || this->control.current_gamepad == Input::Gamepad::X)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << std::endl << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
    }
}

std::vector<float> RL::ComputeObservation()
{
    std::vector<std::vector<float>> obs_list;

    for (const std::string &observation : this->params.Get<std::vector<std::string>>("observations"))
    {
        // ============= Base Observations =============
        if (observation == "lin_vel")
        {
            obs_list.push_back(this->obs.lin_vel * this->params.Get<float>("lin_vel_scale"));
        }
        else if (observation == "ang_vel")
        {
            // Gazebo publishes angular velocity in the configured observation frame.
            // Keep this branch explicit because IsaacLab policies are sensitive to angular-velocity frame conventions.
            if (this->ang_vel_axis == "body")
            {
                obs_list.push_back(this->obs.ang_vel * this->params.Get<float>("ang_vel_scale"));
            }
            else if (this->ang_vel_axis == "world")
            {
                obs_list.push_back(QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel) * this->params.Get<float>("ang_vel_scale"));
            }
        }
        else if (observation == "gravity_vec")
        {
            obs_list.push_back(QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec));
        }
        else if (observation == "commands")
        {
            obs_list.push_back(this->obs.commands * this->params.Get<std::vector<float>>("commands_scale"));
        }
        else if (observation == "dof_pos")
        {
            std::vector<float> dof_pos_rel = this->obs.dof_pos - this->params.Get<std::vector<float>>("default_dof_pos");
            for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
            {
                dof_pos_rel[i] = 0.0f;
            }
            obs_list.push_back(dof_pos_rel * this->params.Get<float>("dof_pos_scale"));
        }
        else if (observation == "dof_vel")
        {
            obs_list.push_back(this->obs.dof_vel * this->params.Get<float>("dof_vel_scale"));
        }
        else if (observation == "actions")
        {
            obs_list.push_back(this->obs.actions);
        }
        else if (observation == "foot_contacts")
        {
            obs_list.push_back(this->obs.foot_contacts);
        }
        // ============= Other Observations =============
        else if (observation == "whole_body_tracking/motion_command")
        {
            std::vector<float> motion_cmd;
            if (this->motion_loader)
            {
                auto joint_pos_sdk = this->motion_loader->GetJointPos();
                auto joint_vel_sdk = this->motion_loader->GetJointVel();
                auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
                std::vector<float> joint_pos_training(joint_mapping.size());
                std::vector<float> joint_vel_training(joint_mapping.size());
                for (size_t i = 0; i < joint_mapping.size(); ++i)
                {
                    joint_pos_training[i] = joint_pos_sdk[joint_mapping[i]];
                    joint_vel_training[i] = joint_vel_sdk[joint_mapping[i]];
                }
                motion_cmd.insert(motion_cmd.end(), joint_pos_training.begin(), joint_pos_training.end());
                motion_cmd.insert(motion_cmd.end(), joint_vel_training.begin(), joint_vel_training.end());
            }
            else
            {
                motion_cmd.resize(this->params.Get<int>("num_of_dofs") * 2, 0.0f);
            }
            obs_list.push_back(motion_cmd);
        }
        else if (observation == "whole_body_tracking/motion_anchor_ori_b")
        {
            std::vector<float> anchor_ori(6, 0.0f);
            if (this->motion_loader)
            {
                auto waist_sdk_indices = this->params.Get<std::vector<int>>("waist_joint_indices");
                std::vector<float> waist_angles = {
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[0])],
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[1])],
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[2])]
                };
                std::vector<float> robot_torso_quat_w = MotionLoader::ComputeTorsoQuat(this->obs.base_quat, waist_angles);
                std::vector<float> ref_torso_quat_w = this->motion_loader->GetAnchorQuat();
                std::vector<float> init_quat = this->motion_loader->GetInitQuat();
                std::vector<float> motion_anchor_quat_w = QuaternionMultiply(init_quat, ref_torso_quat_w);
                std::vector<float> robot_quat_inv = QuaternionConjugate(robot_torso_quat_w);
                std::vector<float> relative_quat = QuaternionMultiply(robot_quat_inv, motion_anchor_quat_w);
                std::vector<float> rot_matrix = QuaternionToRotationMatrix(relative_quat);
                anchor_ori = MatrixFirstTwoColumns(rot_matrix);
            }
            obs_list.push_back(anchor_ori);
        }
        else if (observation == "RoboMimic_Deploy/phase")
        {
            float motion_time = this->episode_length_buf * this->params.Get<float>("dt") * this->params.Get<int>("decimation");
            float count = motion_time;
            float phase = count / this->motion_length;
            std::vector<float> phase_vec = {phase};
            obs_list.push_back(phase_vec);
        }
    }

    this->obs_dims.clear();
    for (const auto& obs : obs_list)
    {
       this->obs_dims.push_back(obs.size());
    }

    std::vector<float> obs;
    for (const auto& obs_vec : obs_list)
    {
        obs.insert(obs.end(), obs_vec.begin(), obs_vec.end());
    }
    std::vector<float> clamped_obs = clamp(obs, -this->params.Get<float>("clip_obs"), this->params.Get<float>("clip_obs"));
    return clamped_obs;
}

void RL::InitObservations()
{
    this->obs.lin_vel = {0.0f, 0.0f, 0.0f};
    this->obs.ang_vel = {0.0f, 0.0f, 0.0f};
    this->obs.gravity_vec = {0.0f, 0.0f, -1.0f};
    this->obs.commands = {0.0f, 0.0f, 0.0f};
    this->obs.base_quat = {0.0f, 0.0f, 0.0f, 1.0f};
    this->obs.dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->obs.dof_vel.clear();
    this->obs.dof_vel.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    this->obs.actions.clear();
    this->obs.actions.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    this->obs.foot_contacts = {0.0f, 0.0f, 0.0f, 0.0f};
    this->ComputeObservation();
}

void RL::InitOutputs()
{
    int num_of_dofs = this->params.Get<int>("num_of_dofs");
    this->output_dof_tau.clear();
    this->output_dof_tau.resize(num_of_dofs, 0.0f);
    this->output_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->output_dof_vel.clear();
    this->output_dof_vel.resize(num_of_dofs, 0.0f);
}

void RL::InitControl()
{
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
}

void RL::ResetPolicyHistory()
{
    this->history_obs.clear();
    this->policy_history_initialized = false;

    const auto observations_history = this->params.Get<std::vector<int>>("observations_history");
    if (observations_history.empty())
    {
        this->history_obs_buf = ObservationBuffer();
        return;
    }

    if (std::any_of(observations_history.begin(), observations_history.end(), [](int index) { return index < 0; }))
    {
        throw std::runtime_error("observations_history cannot contain negative indices");
    }

    const std::string priority = this->params.Get<std::string>("observations_history_priority", "time");
    const int buffer_length = *std::max_element(observations_history.begin(), observations_history.end()) + 1;
    this->history_obs_buf = ObservationBuffer(1, this->obs_dims, buffer_length, priority);
}

PolicyModelInput RL::BuildPolicyModelInput(const std::vector<float>& clamped_obs)
{
    const int configured_obs_dim = this->params.Get<int>("num_observations", -1);
    if (configured_obs_dim <= 0)
    {
        throw std::runtime_error("num_observations must be a positive integer");
    }
    if (clamped_obs.size() != static_cast<size_t>(configured_obs_dim))
    {
        std::ostringstream message;
        message << "Observation size mismatch: config declares " << configured_obs_dim
                << " values, but the configured terms produced " << clamped_obs.size();
        throw std::runtime_error(message.str());
    }

    const std::string configured_mode = this->params.Get<std::string>("model_forward_mode", "single");
    if (configured_mode != "single" && configured_mode != "flat_history" && configured_mode != "obs_history")
    {
        throw std::runtime_error(
            "Unsupported model_forward_mode '" + configured_mode
            + "'. Expected single, flat_history, or obs_history");
    }

    const auto observations_history = this->params.Get<std::vector<int>>("observations_history");
    const bool history_required = configured_mode == "flat_history" || configured_mode == "obs_history";
    if (history_required && observations_history.empty())
    {
        throw std::runtime_error("model_forward_mode=" + configured_mode + " requires observations_history");
    }

    if (observations_history.empty())
    {
        return {{clamped_obs}, {{1, static_cast<int64_t>(clamped_obs.size())}}};
    }

    const int history_frames = static_cast<int>(observations_history.size());
    if (this->params.Has("history_length"))
    {
        const int configured_history_length = this->params.Get<int>("history_length");
        if (configured_history_length != history_frames)
        {
            std::ostringstream message;
            message << "history_length is " << configured_history_length
                    << ", but observations_history selects " << history_frames << " frames";
            throw std::runtime_error(message.str());
        }
    }

    const std::string priority = this->params.Get<std::string>("observations_history_priority", "time");
    if (configured_mode == "obs_history" && priority != "time")
    {
        throw std::runtime_error("model_forward_mode=obs_history requires observations_history_priority=time");
    }

    if (this->params.Get<bool>("warm_start_history", false) && !this->policy_history_initialized)
    {
        this->history_obs_buf.reset({0}, clamped_obs);
    }
    this->history_obs_buf.insert(clamped_obs);
    this->policy_history_initialized = true;
    this->history_obs = this->history_obs_buf.get_obs_vec(observations_history);

    const size_t expected_history_values = static_cast<size_t>(history_frames) * clamped_obs.size();
    if (this->history_obs.size() != expected_history_values)
    {
        std::ostringstream message;
        message << "History buffer produced " << this->history_obs.size()
                << " values; expected " << expected_history_values;
        throw std::runtime_error(message.str());
    }

    if (configured_mode == "obs_history")
    {
        return {
            {clamped_obs, this->history_obs},
            {
                {1, static_cast<int64_t>(clamped_obs.size())},
                {1, static_cast<int64_t>(history_frames), static_cast<int64_t>(clamped_obs.size())}
            }
        };
    }

    return {{this->history_obs}, {{1, static_cast<int64_t>(this->history_obs.size())}}};
}

void RL::ValidatePolicyModelContract()
{
    if (!this->model || !this->model->is_loaded())
    {
        throw std::runtime_error("Cannot validate an unloaded policy model");
    }

    try
    {
        const auto observation = this->ComputeObservation();
        const auto model_input = this->BuildPolicyModelInput(observation);
        const auto actions = this->model->forward_with_shapes(model_input.values, model_input.shapes);
        const size_t expected_actions = static_cast<size_t>(this->params.Get<int>("num_of_dofs"));
        if (actions.size() != expected_actions)
        {
            std::ostringstream message;
            message << "Policy output size mismatch: model returned " << actions.size()
                    << " actions, but the robot has " << expected_actions << " controlled joints";
            throw std::runtime_error(message.str());
        }
        if (!std::all_of(actions.begin(), actions.end(), [](float action) { return std::isfinite(action); }))
        {
            throw std::runtime_error("Policy returned a non-finite action during validation");
        }

        std::string effective_mode = this->params.Get<std::string>("model_forward_mode", "single");
        if (effective_mode == "single" && !this->params.Get<std::vector<int>>("observations_history").empty())
        {
            effective_mode = "flat_history";
        }
        std::cout << LOGGER::INFO << "Validated policy contract - mode: "
                  << effective_mode
                  << ", observations: " << observation.size()
                  << ", inputs: " << model_input.values.size()
                  << ", actions: " << actions.size() << std::endl;
    }
    catch (const std::exception& error)
    {
        this->ResetPolicyHistory();
        throw std::runtime_error(std::string("Policy contract validation failed: ") + error.what());
    }

    this->ResetPolicyHistory();
}

void RL::InitJointNum(size_t num_joints)
{
    this->robot_state.motor_state.resize(num_joints);
    this->start_state.motor_state.resize(num_joints);
    this->now_state.motor_state.resize(num_joints);
    this->robot_command.motor_command.resize(num_joints);
    {
        std::lock_guard<std::mutex> lock(this->gain_fault_mutex);
        this->gain_alpha.assign(num_joints, 1.0f);
    }
}

void RL::SetGainAlpha(const std::vector<float> &alpha)
{
    std::lock_guard<std::mutex> lock(this->gain_fault_mutex);
    this->gain_alpha = alpha;
}

std::vector<float> RL::GetGainAlpha()
{
    std::lock_guard<std::mutex> lock(this->gain_fault_mutex);
    return this->gain_alpha;
}

void RL::InitRL(std::string robot_config_path)
{
    std::lock_guard<std::mutex> lock(this->model_mutex);

    this->ReadYaml(robot_config_path, "config.yaml");

    // init joint num first
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));

    // init rl
    this->InitObservations();
    this->InitOutputs();
    this->InitControl();

    this->ResetPolicyHistory();

    // init model
    std::string model_path = std::string(POLICY_DIR) + "/" + robot_config_path + "/" + this->params.Get<std::string>("model_name");
    this->model = InferenceRuntime::ModelFactory::load_model(model_path);
    if (!this->model)
    {
        throw std::runtime_error("Failed to load model from: " + model_path);
    }
    if (this->params.Get<bool>("validate_model_on_load", true))
    {
        this->ValidatePolicyModelContract();
    }
}

void RL::ComputeOutput(const std::vector<float> &actions, std::vector<float> &output_dof_pos, std::vector<float> &output_dof_vel, std::vector<float> &output_dof_tau)
{
    std::vector<float> actions_scaled = actions * this->params.Get<std::vector<float>>("action_scale");
    std::vector<float> pos_actions_scaled = actions_scaled;
    std::vector<float> vel_actions_scaled(actions.size(), 0.0f);
    for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
    {
        pos_actions_scaled[i] = 0.0f;
        vel_actions_scaled[i] = actions_scaled[i];
    }
    std::vector<float> all_actions_scaled = pos_actions_scaled + vel_actions_scaled;
    output_dof_pos = pos_actions_scaled + this->params.Get<std::vector<float>>("default_dof_pos");
    if (this->params.Has("joint_pos_limit_lower") && this->params.Has("joint_pos_limit_upper"))
    {
        output_dof_pos = clamp(
            output_dof_pos,
            this->params.Get<std::vector<float>>("joint_pos_limit_lower"),
            this->params.Get<std::vector<float>>("joint_pos_limit_upper"));
        all_actions_scaled = output_dof_pos - this->params.Get<std::vector<float>>("default_dof_pos") + vel_actions_scaled;
    }
    output_dof_vel = vel_actions_scaled;
    const auto gain_alpha = this->GetGainAlpha();
    const auto rl_kp = this->params.Get<std::vector<float>>("rl_kp") * gain_alpha;
    const auto rl_kd = this->params.Get<std::vector<float>>("rl_kd") * gain_alpha;
    output_dof_tau = rl_kp * (all_actions_scaled + this->params.Get<std::vector<float>>("default_dof_pos") - this->obs.dof_pos) - rl_kd * this->obs.dof_vel;
    output_dof_tau = clamp(output_dof_tau, -this->params.Get<std::vector<float>>("torque_limits"), this->params.Get<std::vector<float>>("torque_limits"));
}

int RL::InverseJointMapping(int idx) const
{
    auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    for (size_t i = 0; i < joint_mapping.size(); ++i) {
        if (joint_mapping[i] == idx) return (int)i;
    }
    return -1;
}

void RL::TorqueProtect(const std::vector<float>& origin_output_dof_tau)
{
    std::vector<int> out_of_range_indices;
    std::vector<float> out_of_range_values;
    for (size_t i = 0; i < origin_output_dof_tau.size(); ++i)
    {
        float torque_value = origin_output_dof_tau[i];
        float limit_lower = -this->params.Get<std::vector<float>>("torque_limits")[i];
        float limit_upper = this->params.Get<std::vector<float>>("torque_limits")[i];

        if (torque_value < limit_lower || torque_value > limit_upper)
        {
            out_of_range_indices.push_back(i);
            out_of_range_values.push_back(torque_value);
        }
    }
    if (!out_of_range_indices.empty())
    {
        for (size_t i = 0; i < out_of_range_indices.size(); ++i)
        {
            int index = out_of_range_indices[i];
            float value = out_of_range_values[i];
            float limit_lower = -this->params.Get<std::vector<float>>("torque_limits")[index];
            float limit_upper = this->params.Get<std::vector<float>>("torque_limits")[index];

            std::cout << LOGGER::WARNING << "Torque(" << index + 1 << ")=" << value << " out of range(" << limit_lower << ", " << limit_upper << ")" << std::endl;
        }
        // Just a reminder, no protection
        // this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::INFO << "Switching to STATE_POS_GETDOWN"<< std::endl;
    }
}

void RL::AttitudeProtect(const std::vector<float> &quaternion, float pitch_threshold, float roll_threshold)
{
    // Use QuaternionToEuler from vector_math.hpp
    std::vector<float> euler = QuaternionToEuler(quaternion);
    float roll = euler[0] * 57.2958f;   // Convert to degrees
    float pitch = euler[1] * 57.2958f;

    if (std::fabs(roll) > roll_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Roll exceeds " << roll_threshold << " degrees. Current: " << roll << " degrees." << std::endl;
    }
    if (std::fabs(pitch) > pitch_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Pitch exceeds " << pitch_threshold << " degrees. Current: " << pitch << " degrees." << std::endl;
    }
}

#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

static int kbhit()
{
    static bool initialized = false;
    static termios original_term;

    // Initialize terminal to non-canonical mode on first call
    if (!initialized)
    {
        tcgetattr(STDIN_FILENO, &original_term);

        termios new_term = original_term;
        new_term.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
        new_term.c_cc[VMIN] = 0;   // Non-blocking read
        new_term.c_cc[VTIME] = 0;  // No timeout

        tcsetattr(STDIN_FILENO, TCSANOW, &new_term);

        // Register cleanup function to restore terminal on exit
        static bool cleanup_registered = false;
        if (!cleanup_registered)
        {
            std::atexit([]() {
                tcsetattr(STDIN_FILENO, TCSANOW, &original_term);
            });
            cleanup_registered = true;
        }

        initialized = true;
    }

    // Non-blocking read of a single character
    char c;
    int result = read(STDIN_FILENO, &c, 1);

    return (result == 1) ? (unsigned char)c : -1;
}

void RL::KeyboardInterface()
{
    int c = kbhit();
    if (c > 0)
    {
        switch (c)
        {
        case '0': this->control.SetKeyboard(Input::Keyboard::Num0); break;
        case '1': this->control.SetKeyboard(Input::Keyboard::Num1); break;
        case '2': this->control.SetKeyboard(Input::Keyboard::Num2); break;
        case '3': this->control.SetKeyboard(Input::Keyboard::Num3); break;
        case '4': this->control.SetKeyboard(Input::Keyboard::Num4); break;
        case '5': this->control.SetKeyboard(Input::Keyboard::Num5); break;
        case '6': this->control.SetKeyboard(Input::Keyboard::Num6); break;
        case '7': this->control.SetKeyboard(Input::Keyboard::Num7); break;
        case '8': this->control.SetKeyboard(Input::Keyboard::Num8); break;
        case '9': this->control.SetKeyboard(Input::Keyboard::Num9); break;
        case 'a': case 'A': this->control.SetKeyboard(Input::Keyboard::A); break;
        case 'b': case 'B': this->control.SetKeyboard(Input::Keyboard::B); break;
        case 'c': case 'C': this->control.SetKeyboard(Input::Keyboard::C); break;
        case 'd': case 'D': this->control.SetKeyboard(Input::Keyboard::D); break;
        case 'e': case 'E': this->control.SetKeyboard(Input::Keyboard::E); break;
        case 'f': case 'F': this->control.SetKeyboard(Input::Keyboard::F); break;
        case 'g': case 'G': this->control.SetKeyboard(Input::Keyboard::G); break;
        case 'h': case 'H': this->control.SetKeyboard(Input::Keyboard::H); break;
        case 'i': case 'I': this->control.SetKeyboard(Input::Keyboard::I); break;
        case 'j': case 'J': this->control.SetKeyboard(Input::Keyboard::J); break;
        case 'k': case 'K': this->control.SetKeyboard(Input::Keyboard::K); break;
        case 'l': case 'L': this->control.SetKeyboard(Input::Keyboard::L); break;
        case 'm': case 'M': this->control.SetKeyboard(Input::Keyboard::M); break;
        case 'n': case 'N': this->control.SetKeyboard(Input::Keyboard::N); break;
        case 'o': case 'O': this->control.SetKeyboard(Input::Keyboard::O); break;
        case 'p': case 'P': this->control.SetKeyboard(Input::Keyboard::P); break;
        case 'q': case 'Q': this->control.SetKeyboard(Input::Keyboard::Q); break;
        case 'r': case 'R': this->control.SetKeyboard(Input::Keyboard::R); break;
        case 's': case 'S': this->control.SetKeyboard(Input::Keyboard::S); break;
        case 't': case 'T': this->control.SetKeyboard(Input::Keyboard::T); break;
        case 'u': case 'U': this->control.SetKeyboard(Input::Keyboard::U); break;
        case 'v': case 'V': this->control.SetKeyboard(Input::Keyboard::V); break;
        case 'w': case 'W': this->control.SetKeyboard(Input::Keyboard::W); break;
        case 'x': case 'X': this->control.SetKeyboard(Input::Keyboard::X); break;
        case 'y': case 'Y': this->control.SetKeyboard(Input::Keyboard::Y); break;
        case 'z': case 'Z': this->control.SetKeyboard(Input::Keyboard::Z); break;
        case ' ': this->control.SetKeyboard(Input::Keyboard::Space); break;
        case '\n': case '\r': this->control.SetKeyboard(Input::Keyboard::Enter); break;
        case 27:  // Escape sequence (for arrow keys on Unix/Linux/macOS)
        {
            char seq[2];
            // Try to read escape sequence non-blockingly
            if (read(STDIN_FILENO, &seq[0], 1) == 1)
            {
                if (seq[0] == '[')
                {
                    if (read(STDIN_FILENO, &seq[1], 1) == 1)
                    {
                        switch (seq[1])
                        {
                        case 'A': this->control.SetKeyboard(Input::Keyboard::Up); break;
                        case 'B': this->control.SetKeyboard(Input::Keyboard::Down); break;
                        case 'C': this->control.SetKeyboard(Input::Keyboard::Right); break;
                        case 'D': this->control.SetKeyboard(Input::Keyboard::Left); break;
                        default: break;
                        }
                    }
                }
                else
                {
                    // Plain escape key
                    this->control.SetKeyboard(Input::Keyboard::Escape);
                }
            }
            else
            {
                // Plain escape key
                this->control.SetKeyboard(Input::Keyboard::Escape);
            }
        } break;
        default:  break;
        }
    }
}

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node &node)
{
    std::vector<T> values;
    for (const auto &val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

void RL::ReadYaml(const std::string& file_path, const std::string& file_name)
{
    std::string config_path = std::string(POLICY_DIR) + "/" + file_path + "/" + file_name;
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)[file_path];
    }
    catch (YAML::BadFile &e)
    {
        throw std::runtime_error("The file '" + config_path + "' does not exist");
    }

    if (!config || !config.IsMap())
    {
        throw std::runtime_error("Missing YAML mapping '" + file_path + "' in " + config_path);
    }

    const bool is_base_config = file_name == "base.yaml";
    if (!is_base_config && this->base_config_node)
    {
        this->params.config_node = YAML::Clone(this->base_config_node);
    }

    if (!is_base_config && config["profile"])
    {
        const std::string profile_name = config["profile"].as<std::string>();
        if (!IsSafeProfileName(profile_name))
        {
            throw std::runtime_error("Invalid policy profile name: " + profile_name);
        }

        const size_t separator = file_path.find('/');
        const std::string robot_directory = file_path.substr(0, separator);
        const std::string profile_path = std::string(POLICY_DIR) + "/" + robot_directory
            + "/profiles/" + profile_name + ".yaml";

        YAML::Node profile_config;
        try
        {
            profile_config = YAML::LoadFile(profile_path);
        }
        catch (YAML::BadFile& error)
        {
            throw std::runtime_error("Policy profile file does not exist: " + profile_path);
        }
        MergeYamlMap(this->params.config_node, profile_config);
    }

    MergeYamlMap(this->params.config_node, config);
    if (is_base_config)
    {
        this->base_config_node = YAML::Clone(this->params.config_node);
    }
}

void RL::CSVInit(std::string robot_path)
{
    csv_filename = std::string(POLICY_DIR) + "/" + robot_path + "/motor";

    // Uncomment these lines if need timestamp for file name
    // auto now = std::chrono::system_clock::now();
    // std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // std::stringstream ss;
    // ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
    // std::string timestamp = ss.str();
    // csv_filename += "_" + timestamp;

    csv_filename += ".csv";
    std::ofstream file(csv_filename.c_str());

    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "tau_cal_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "tau_est_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_pos_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_pos_target_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_vel_" << i << ","; }

    file << std::endl;

    file.close();
}

void RL::CSVLogger(const std::vector<float>& torque, const std::vector<float>& tau_est, const std::vector<float>& joint_pos, const std::vector<float>& joint_pos_target, const std::vector<float>& joint_vel)
{
    std::ofstream file(csv_filename.c_str(), std::ios_base::app);

    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << torque[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << tau_est[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_pos[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_pos_target[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_vel[i] << ","; }

    file << std::endl;

    file.close();
}

bool RLFSMState::Interpolate(
    float& percent,
    const std::vector<float>& start_pos,
    const std::vector<float>& target_pos,
    float duration_seconds,
    const std::string& description,
    bool use_fixed_gains)
{
    if (percent >= 1.0f)
    {
        return false;
    }

    if (percent == 0.0f)
    {
        float max_diff = 0.0f;
        for (size_t i = 0; i < start_pos.size() && i < target_pos.size(); ++i)
        {
            max_diff = std::max(max_diff, std::abs(start_pos[i] - target_pos[i]));
        }

        if (max_diff < 0.1f)
        {
            percent = 1.0f;
        }
    }

    int required_frames = std::max(1, static_cast<int>(std::ceil(duration_seconds / rl.params.Get<float>("dt"))));
    float step = 1.0f / required_frames;

    percent += step;
    percent = std::min(percent, 1.0f);

    auto kp = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kp") : rl.params.Get<std::vector<float>>("rl_kp");
    auto kd = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kd") : rl.params.Get<std::vector<float>>("rl_kd");

    for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
    {
        fsm_command->motor_command.q[i] = (1 - percent) * start_pos[i] + percent * target_pos[i];
        fsm_command->motor_command.dq[i] = 0;
        fsm_command->motor_command.kp[i] = kp[i];
        fsm_command->motor_command.kd[i] = kd[i];
        fsm_command->motor_command.tau[i] = 0;
    }

    if (!description.empty())
    {
        LOGGER::PrintProgress(percent, description);
    }

    if (percent >= 1.0f)
    {
        return false;
    }

    return true;
}

void RLFSMState::RLControl()
{
    std::vector<float> _output_dof_pos, _output_dof_vel;
    if (rl.output_dof_pos_queue.try_pop(_output_dof_pos) && rl.output_dof_vel_queue.try_pop(_output_dof_vel))
    {
        const auto rl_kp = rl.params.Get<std::vector<float>>("rl_kp");
        const auto rl_kd = rl.params.Get<std::vector<float>>("rl_kd");
        const auto gain_alpha = rl.GetGainAlpha();
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            if (!_output_dof_pos.empty())
            {
                fsm_command->motor_command.q[i] = _output_dof_pos[i];
            }
            if (!_output_dof_vel.empty())
            {
                fsm_command->motor_command.dq[i] = _output_dof_vel[i];
            }
            fsm_command->motor_command.kp[i] = rl_kp[i] * gain_alpha[i];
            fsm_command->motor_command.kd[i] = rl_kd[i] * gain_alpha[i];
            fsm_command->motor_command.tau[i] = 0;
        }
    }
}
