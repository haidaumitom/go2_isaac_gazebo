#include "rl_sim.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

RL_Sim::RL_Sim(int argc, char **argv)
{
    ros2_node = std::make_shared<rclcpp::Node>("rl_sim_node");
    this->ang_vel_axis = "body";
    this->ros_namespace = ros2_node->get_namespace();
    // get params from param_node
    param_client = ros2_node->create_client<rcl_interfaces::srv::GetParameters>("/param_node/get_parameters");
    while (!param_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()) {
            std::cout << LOGGER::ERROR << "Interrupted while waiting for param_node service. Exiting." << std::endl;
            return;
        }
        std::cout << LOGGER::WARNING << "Waiting for param_node service to be available..." << std::endl;
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_name", "gazebo_model_name"};
    // Use a timeout for the future
    auto future = param_client->async_send_request(request);
    auto status = rclcpp::spin_until_future_complete(ros2_node->get_node_base_interface(), future, std::chrono::seconds(5));
    if (status == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();
        if (result->values.size() < 2)
        {
            std::cout << LOGGER::ERROR << "Failed to get all parameters from param_node" << std::endl;
        }
        else
        {
            this->robot_name = result->values[0].string_value;
            this->gazebo_model_name = result->values[1].string_value;
            std::cout << LOGGER::INFO << "Get param robot_name: " << this->robot_name << std::endl;
            std::cout << LOGGER::INFO << "Get param gazebo_model_name: " << this->gazebo_model_name << std::endl;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "Failed to call param_node service" << std::endl;
    }

    // read params from yaml
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

    this->LoadRecoveryPolicy();

    // init robot
    this->robot_command_publisher_msg.motor_command.resize(this->params.Get<int>("num_of_dofs"));
    this->robot_state_subscriber_msg.motor_state.resize(this->params.Get<int>("num_of_dofs"));
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

    this->StartJointController(this->ros_namespace, this->params.Get<std::vector<std::string>>("joint_names"));
    // publisher
    this->robot_command_publisher = ros2_node->create_publisher<robot_msgs::msg::RobotCommand>(
        this->ros_namespace + "robot_joint_controller/command", rclcpp::SystemDefaultsQoS());

    // subscriber
    this->cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) {this->CmdvelCallback(msg);}
    );
    this->joy_subscriber = ros2_node->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::SystemDefaultsQoS(),
        [this] (const sensor_msgs::msg::Joy::SharedPtr msg) {this->JoyCallback(msg);}
    );
    this->gazebo_imu_subscriber = ros2_node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SystemDefaultsQoS(),
        [this] (const sensor_msgs::msg::Imu::SharedPtr msg) {this->GazeboImuCallback(msg);}
    );
    this->model_state_subscriber = ros2_node->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo/model_states", rclcpp::SystemDefaultsQoS(),
        [this] (const gazebo_msgs::msg::ModelStates::SharedPtr msg) {this->ModelStatesCallback(msg);}
    );

    this->model_state_subscriber_root = ros2_node->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/model_states", rclcpp::SystemDefaultsQoS(),
        [this] (const gazebo_msgs::msg::ModelStates::SharedPtr msg) {this->ModelStatesCallback(msg);}
    );
    this->link_state_subscriber = ros2_node->create_subscription<gazebo_msgs::msg::LinkStates>(
        "/gazebo/link_states", rclcpp::SystemDefaultsQoS(),
        [this] (const gazebo_msgs::msg::LinkStates::SharedPtr msg) {this->LinkStatesCallback(msg);}
    );
    this->link_state_subscriber_root = ros2_node->create_subscription<gazebo_msgs::msg::LinkStates>(
        "/link_states", rclcpp::SystemDefaultsQoS(),
        [this] (const gazebo_msgs::msg::LinkStates::SharedPtr msg) {this->LinkStatesCallback(msg);}
    );
    this->robot_state_subscriber = ros2_node->create_subscription<robot_msgs::msg::RobotState>(
        this->ros_namespace + "robot_joint_controller/state", rclcpp::SystemDefaultsQoS(),
        [this] (const robot_msgs::msg::RobotState::SharedPtr msg) {this->RobotStateCallback(msg);}
    );
    if (this->fault_switch_enabled)
    {
        const std::string fault_alpha_topic = this->params.Get<std::string>(
            "fault_alpha_topic",
            this->ros_namespace + "robot_joint_controller/fault_alpha"
        );
        this->fault_alpha_subscriber = ros2_node->create_subscription<std_msgs::msg::Float32MultiArray>(
            fault_alpha_topic, rclcpp::SystemDefaultsQoS(),
            [this] (const std_msgs::msg::Float32MultiArray::SharedPtr msg) {this->FaultAlphaCallback(msg);}
        );
        std::cout << LOGGER::INFO << "Fault policy switch listening on " << fault_alpha_topic << std::endl;
    }
    const std::string gain_alpha_topic = this->params.Get<std::string>(
        "gain_alpha_topic",
        "/rl_sar/gain_alpha"
    );
    this->gain_alpha_subscriber = ros2_node->create_subscription<std_msgs::msg::Float32MultiArray>(
        gain_alpha_topic, rclcpp::SystemDefaultsQoS(),
        [this] (const std_msgs::msg::Float32MultiArray::SharedPtr msg) {this->GainAlphaCallback(msg);}
    );
    std::cout << LOGGER::INFO << "RL gain fault listening on " << gain_alpha_topic << std::endl;
    const std::string foot_contacts_topic = this->params.Get<std::string>(
        "foot_contacts_topic",
        "/rl_sar/foot_contacts"
    );
    this->foot_contacts_subscriber = ros2_node->create_subscription<std_msgs::msg::Float32MultiArray>(
        foot_contacts_topic, rclcpp::SystemDefaultsQoS(),
        [this] (const std_msgs::msg::Float32MultiArray::SharedPtr msg) {this->FootContactsCallback(msg);}
    );
    std::cout << LOGGER::INFO << "Foot contact observation listening on " << foot_contacts_topic << std::endl;
    const std::vector<std::string> default_gazebo_foot_contact_topics = {
        "/FL_foot_contact",
        "/FR_foot_contact",
        "/RL_foot_contact",
        "/RR_foot_contact"
    };
    const auto gazebo_foot_contact_topics = this->params.Get<std::vector<std::string>>(
        "gazebo_foot_contact_topics",
        default_gazebo_foot_contact_topics
    );
    if (gazebo_foot_contact_topics.size() == 4)
    {
        this->gazebo_foot_contact_subscribers.reserve(gazebo_foot_contact_topics.size());
        for (size_t foot_id = 0; foot_id < gazebo_foot_contact_topics.size(); ++foot_id)
        {
            this->gazebo_foot_contact_subscribers.push_back(
                ros2_node->create_subscription<gazebo_msgs::msg::ContactsState>(
                    gazebo_foot_contact_topics[foot_id],
                    rclcpp::SystemDefaultsQoS(),
                    [this, foot_id] (const gazebo_msgs::msg::ContactsState::SharedPtr msg)
                    {
                        this->GazeboFootContactCallback(foot_id, msg);
                    }
                )
            );
        }
        std::cout << LOGGER::INFO << "Direct Gazebo foot contact listening in [FL, FR, RL, RR] order." << std::endl;
    }
    else
    {
        std::cout << LOGGER::WARNING << "gazebo_foot_contact_topics must contain 4 topics; direct contact input disabled." << std::endl;
    }

    // service
    this->gazebo_pause_physics_client = ros2_node->create_client<std_srvs::srv::Empty>("/pause_physics");
    this->gazebo_unpause_physics_client = ros2_node->create_client<std_srvs::srv::Empty>("/unpause_physics");
    this->gazebo_reset_world_client = ros2_node->create_client<std_srvs::srv::Empty>("/reset_world");

    auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = this->gazebo_reset_world_client->async_send_request(empty_request);

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;
}

void RL_Sim::LoadRecoveryPolicy()
{
    this->fault_switch_enabled = this->params.Get<bool>("fault_policy_switch_enabled", false);
    if (!this->fault_switch_enabled)
    {
        return;
    }

    this->recovery_policy_name = this->params.Get<std::string>("recovery_policy_config_name", "");
    if (this->recovery_policy_name.empty())
    {
        std::cout << LOGGER::WARNING << "Fault policy switch enabled, but recovery_policy_config_name is empty" << std::endl;
        this->fault_switch_enabled = false;
        return;
    }

    std::string recovery_model_name = this->params.Get<std::string>("recovery_model_name", "policy.pt");
    const std::string recovery_config_path =
        std::string(POLICY_DIR) + "/" + this->robot_name + "/" + this->recovery_policy_name + "/config.yaml";
    try
    {
        YAML::Node recovery_config = YAML::LoadFile(recovery_config_path);
        const std::string recovery_config_key = this->robot_name + "/" + this->recovery_policy_name;
        if (recovery_config[recovery_config_key] && recovery_config[recovery_config_key]["model_name"])
        {
            recovery_model_name = recovery_config[recovery_config_key]["model_name"].as<std::string>();
        }
    }
    catch (const YAML::Exception&)
    {
        std::cout << LOGGER::WARNING << "Could not read recovery config; using model name " << recovery_model_name << std::endl;
    }

    const std::string recovery_model_path =
        std::string(POLICY_DIR) + "/" + this->robot_name + "/" + this->recovery_policy_name + "/" + recovery_model_name;
    this->recovery_model = InferenceRuntime::ModelFactory::load_model(recovery_model_path);
    if (!this->recovery_model)
    {
        std::cout << LOGGER::ERROR << "Failed to load recovery policy from: " << recovery_model_path << std::endl;
        this->fault_switch_enabled = false;
        return;
    }

    std::cout << LOGGER::INFO << "Recovery policy loaded: " << this->recovery_policy_name << std::endl;
}

RL_Sim::~RL_Sim()
{
    this->ClosePolicyDebugCsv();
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::StartJointController(const std::string& ros_namespace, const std::vector<std::string>& names)
{
    const char* ros_distro = std::getenv("ROS_DISTRO");
    std::string spawner = (ros_distro && std::string(ros_distro) == "foxy") ? "spawner.py" : "spawner";

    std::filesystem::path tmp_path = std::filesystem::temp_directory_path() / "robot_joint_controller_params.yaml";
    {
        std::ofstream tmp_file(tmp_path);
        if (!tmp_file)
        {
            throw std::runtime_error("Failed to create temporary parameter file");
        }

        tmp_file << "/robot_joint_controller:\n";
        tmp_file << "    ros__parameters:\n";
        tmp_file << "        joints:\n";
        for (const auto& name : names)
        {
            tmp_file << "            - " << name << "\n";
        }
    }

    pid_t pid = fork();
    if (pid == 0)
    {
        std::string cmd = "ros2 run controller_manager " + spawner + " robot_joint_controller ";
        cmd += "-p " + tmp_path.string() + " ";
        // cmd += " > /dev/null 2>&1";  // Comment this line to see the output
        execlp("sh", "sh", "-c", cmd.c_str(), nullptr);
        exit(1);
    }
    else if (pid > 0)
    {
        int status;
        waitpid(pid, &status, 0);

        if (WIFEXITED(status) && WEXITSTATUS(status) != 0)
        {
            throw std::runtime_error("Failed to start joint controller");
        }

        std::filesystem::remove(tmp_path);
    }
    else
    {
        throw std::runtime_error("fork() failed");
    }
}

void RL_Sim::GetState(RobotState<float> *state)
{
    const auto &orientation = this->gazebo_imu.orientation;
    const auto &angular_velocity = this->gazebo_imu.angular_velocity;

    state->imu.quaternion[0] = orientation.w;
    state->imu.quaternion[1] = orientation.x;
    state->imu.quaternion[2] = orientation.y;
    state->imu.quaternion[3] = orientation.z;

    state->imu.gyroscope[0] = angular_velocity.x;
    state->imu.gyroscope[1] = angular_velocity.y;
    state->imu.gyroscope[2] = angular_velocity.z;

    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        state->motor_state.q[i] = this->robot_state_subscriber_msg.motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]].q;
        state->motor_state.dq[i] = this->robot_state_subscriber_msg.motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq;
        state->motor_state.tau_est[i] = this->robot_state_subscriber_msg.motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau_est;
    }
}

void RL_Sim::SetCommand(const RobotCommand<float> *command)
{
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].q = command->motor_command.q[i];
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq = command->motor_command.dq[i];
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].kp = command->motor_command.kp[i];
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].kd = command->motor_command.kd[i];
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau = command->motor_command.tau[i];
    }

    this->robot_command_publisher->publish(this->robot_command_publisher_msg);
}

void RL_Sim::RobotControl()
{
    this->GetState(&this->robot_state);

    this->StateController(&this->robot_state, &this->robot_command);

    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
        auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = this->gazebo_reset_world_client->async_send_request(empty_request);
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_pause_physics_client->async_send_request(empty_request);
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_unpause_physics_client->async_send_request(empty_request);
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
        this->control.current_keyboard = this->control.last_keyboard;
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

void RL_Sim::GazeboImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    this->gazebo_imu = *msg;
}

void RL_Sim::ModelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == this->gazebo_model_name && i < msg->twist.size())
        {
            this->vel = msg->twist[i];
            return;
        }
    }
}

void RL_Sim::LinkStatesCallback(const gazebo_msgs::msg::LinkStates::SharedPtr msg)
{
    const auto now = this->ros2_node->now();
    if (this->last_foot_contacts_topic_time.nanoseconds() > 0 &&
        (now - this->last_foot_contacts_topic_time).seconds() < 0.25)
    {
        return;
    }

    const float height_threshold = this->params.Get<float>("foot_contact_height_threshold", 0.08f);
    const std::vector<std::string> default_foot_links = {
        this->gazebo_model_name + "::FL_foot",
        this->gazebo_model_name + "::FR_foot",
        this->gazebo_model_name + "::RL_foot",
        this->gazebo_model_name + "::RR_foot"
    };
    const auto foot_links = this->params.Get<std::vector<std::string>>("foot_contact_link_names", default_foot_links);
    if (foot_links.size() != 4)
    {
        return;
    }

    std::vector<float> contacts(4, 0.0f);
    for (size_t foot_id = 0; foot_id < foot_links.size(); ++foot_id)
    {
        const std::string short_name = foot_links[foot_id].substr(foot_links[foot_id].find_last_of(':') + 1);
        for (size_t i = 0; i < msg->name.size() && i < msg->pose.size(); ++i)
        {
            if (msg->name[i] == foot_links[foot_id] || msg->name[i] == short_name ||
                (msg->name[i].size() > short_name.size() &&
                 msg->name[i].compare(msg->name[i].size() - short_name.size(), short_name.size(), short_name) == 0))
            {
                contacts[foot_id] = msg->pose[i].position.z <= height_threshold ? 1.0f : 0.0f;
                break;
            }
        }
    }
    this->obs.foot_contacts = contacts;
}

void RL_Sim::CmdvelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg
)
{
    this->cmd_vel = *msg;
}

void RL_Sim::FaultAlphaCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    const float threshold = this->params.Get<float>("fault_alpha_threshold", 0.999f);
    bool detected = false;
    std::ostringstream broken_joints;
    const auto joint_names = this->params.Get<std::vector<std::string>>("joint_names");

    for (size_t i = 0; i < msg->data.size(); ++i)
    {
        if (msg->data[i] < threshold)
        {
            if (detected)
            {
                broken_joints << ", ";
            }
            if (i < joint_names.size())
            {
                broken_joints << joint_names[i];
            }
            else
            {
                broken_joints << "joint_" << i;
            }
            broken_joints << "=" << msg->data[i];
            detected = true;
        }
    }

    bool previous_detected = false;
    {
        std::lock_guard<std::mutex> lock(this->fault_mutex);
        previous_detected = this->fault_detected;
        this->fault_detected = detected;
        this->latest_fault_alpha.assign(msg->data.begin(), msg->data.end());
    }

    if (detected && !previous_detected)
    {
        RCLCPP_WARN(
            this->ros2_node->get_logger(),
            "Fault detected: %s. Zeroing command and blending to recovery policy '%s'.",
            broken_joints.str().c_str(),
            this->recovery_policy_name.c_str()
        );
    }
    else if (!detected && previous_detected)
    {
        RCLCPP_INFO(this->ros2_node->get_logger(), "Fault cleared. Blending back to normal policy.");
    }
}

void RL_Sim::GainAlphaCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    const int num_dofs = this->params.Get<int>("num_of_dofs");
    if (static_cast<int>(msg->data.size()) != num_dofs)
    {
        RCLCPP_ERROR_THROTTLE(
            this->ros2_node->get_logger(),
            *this->ros2_node->get_clock(),
            2000,
            "Ignoring gain alpha message with %zu entries; expected %d.",
            msg->data.size(),
            num_dofs
        );
        return;
    }

    std::vector<float> controller_alpha(num_dofs, 1.0f);
    std::ostringstream damaged_joints;
    bool has_damage = false;
    const auto joint_names = this->params.Get<std::vector<std::string>>("joint_names");

    for (int i = 0; i < num_dofs; ++i)
    {
        controller_alpha[i] = std::clamp(msg->data[i], 0.0f, 1.0f);
        if (controller_alpha[i] < 0.999f)
        {
            if (has_damage)
            {
                damaged_joints << ", ";
            }
            damaged_joints << ((i < static_cast<int>(joint_names.size())) ? joint_names[i] : ("joint_" + std::to_string(i)));
            damaged_joints << "=" << controller_alpha[i];
            has_damage = true;
        }
    }

    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    std::vector<float> policy_alpha(num_dofs, 1.0f);
    for (int policy_index = 0; policy_index < num_dofs; ++policy_index)
    {
        if (policy_index < static_cast<int>(joint_mapping.size()) &&
            joint_mapping[policy_index] >= 0 &&
            joint_mapping[policy_index] < num_dofs)
        {
            policy_alpha[policy_index] = controller_alpha[joint_mapping[policy_index]];
        }
    }

    this->SetGainAlpha(policy_alpha);

    if (has_damage)
    {
        RCLCPP_WARN_THROTTLE(
            this->ros2_node->get_logger(),
            *this->ros2_node->get_clock(),
            2000,
            "Applying RL gain damage: %s. Effective gains are rl_kp*alpha and rl_kd*alpha.",
            damaged_joints.str().c_str()
        );
    }
    else
    {
        RCLCPP_INFO_THROTTLE(
            this->ros2_node->get_logger(),
            *this->ros2_node->get_clock(),
            2000,
            "RL gains restored to healthy alpha=1.0."
        );
    }
}

void RL_Sim::FootContactsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 4)
    {
        RCLCPP_ERROR_THROTTLE(
            this->ros2_node->get_logger(),
            *this->ros2_node->get_clock(),
            2000,
            "Ignoring foot contact message with %zu entries; expected 4.",
            msg->data.size()
        );
        return;
    }

    for (size_t i = 0; i < 4; ++i)
    {
        this->obs.foot_contacts[i] = msg->data[i] > 0.5f ? 1.0f : 0.0f;
    }
    this->last_foot_contacts_topic_time = this->ros2_node->now();
}

void RL_Sim::GazeboFootContactCallback(size_t foot_id, const gazebo_msgs::msg::ContactsState::SharedPtr msg)
{
    if (foot_id >= this->obs.foot_contacts.size())
    {
        return;
    }

    float total_fx = 0.0f;
    float total_fy = 0.0f;
    float total_fz = 0.0f;
    for (const auto& state : msg->states)
    {
        const auto& force = state.total_wrench.force;
        if (std::abs(force.x) > 1.0e-6 || std::abs(force.y) > 1.0e-6 || std::abs(force.z) > 1.0e-6)
        {
            total_fx += static_cast<float>(force.x);
            total_fy += static_cast<float>(force.y);
            total_fz += static_cast<float>(force.z);
        }
        else
        {
            for (const auto& wrench : state.wrenches)
            {
                total_fx += static_cast<float>(wrench.force.x);
                total_fy += static_cast<float>(wrench.force.y);
                total_fz += static_cast<float>(wrench.force.z);
            }
        }
    }

    const float force_norm = std::sqrt(total_fx * total_fx + total_fy * total_fy + total_fz * total_fz);
    const float threshold = this->params.Get<float>("foot_contact_force_threshold", 1.0f);
    this->obs.foot_contacts[foot_id] = force_norm > threshold ? 1.0f : 0.0f;
    this->last_foot_contacts_topic_time = this->ros2_node->now();
}

void RL_Sim::JoyCallback(
    const sensor_msgs::msg::Joy::SharedPtr msg
)
{
    this->joy_msg = *msg;

    // joystick control
    // Description of buttons and axes(F710):
    // |__ buttons[]: A=0, B=1, X=2, Y=3, LB=4, RB=5, back=6, start=7, power=8, stickL=9, stickR=10
    // |__ axes[]: Lx=0, Ly=1, Rx=3, Ry=4, LT=2, RT=5, DPadX=6, DPadY=7

    if (this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::A);
    if (this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::B);
    if (this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::X);
    if (this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->joy_msg.buttons[4]) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->joy_msg.buttons[5]) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[5]) this->control.SetGamepad(Input::Gamepad::LB_RB);

    this->control.x = this->joy_msg.axes[1]; // LY
    this->control.y = this->joy_msg.axes[0]; // LX
    this->control.yaw = this->joy_msg.axes[3]; // RX
}

void RL_Sim::RobotStateCallback(const robot_msgs::msg::RobotState::SharedPtr msg)
{
    this->robot_state_subscriber_msg = *msg;
}

void RL_Sim::RunModel()
{
    if (this->rl_init_done && simulation_running)
    {
        this->episode_length_buf += 1;
        this->UpdateFaultBlend();

        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        std::vector<float> lin_vel_world = {
            static_cast<float>(this->vel.linear.x),
            static_cast<float>(this->vel.linear.y),
            static_cast<float>(this->vel.linear.z)
        };
        this->obs.lin_vel = QuatRotateInverse(this->robot_state.imu.quaternion, lin_vel_world);
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
        if (this->control.navigation_mode)
        {
            this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
        }
        bool active_fault = false;
        {
            std::lock_guard<std::mutex> lock(this->fault_mutex);
            active_fault = this->fault_detected;
        }
        if (this->fault_switch_enabled && this->params.Get<bool>("zero_command_after_fault", true) && (active_fault || this->fault_blend > 0.0f))
        {
            this->obs.commands = {0.0f, 0.0f, 0.0f};
        }
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);
        this->WritePolicyDebugCsv();

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
        std::vector<float> tau_est(this->params.Get<int>("num_of_dofs"), 0.0f);
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            tau_est[i] = this->joint_efforts[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]];
        }
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

void RL_Sim::InitPolicyDebugCsv()
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

        const std::filesystem::path debug_dir = std::filesystem::path(POLICY_DIR).parent_path() / "debug_logs";
        std::filesystem::create_directories(debug_dir);
        csv_path = (debug_dir / ("policy_debug_" + this->config_name + "_" + timestamp.str() + ".csv")).string();
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
        std::cout << LOGGER::ERROR << "Could not open policy debug CSV: " << csv_path << std::endl;
        this->policy_debug_csv_enabled = false;
        return;
    }

    this->policy_debug_csv_file
        << "episode_step,ros_time_s,cmd_x,cmd_y,cmd_yaw,"
        << "lin_vel_x,lin_vel_y,lin_vel_z,ang_vel_x,ang_vel_y,ang_vel_z,"
        << "foot_FL,foot_FR,foot_RL,foot_RR";

    const auto controller_joint_names = this->params.Get<std::vector<std::string>>("joint_names");
    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        std::string joint_name = "joint_" + std::to_string(i);
        if (i < static_cast<int>(joint_mapping.size()) &&
            joint_mapping[i] >= 0 &&
            joint_mapping[i] < static_cast<int>(controller_joint_names.size()))
        {
            joint_name = controller_joint_names[joint_mapping[i]];
        }
        this->policy_debug_csv_file
            << ",action_" << joint_name
            << ",target_q_" << joint_name
            << ",actual_q_" << joint_name
            << ",error_q_" << joint_name
            << ",actual_dq_" << joint_name
            << ",target_tau_" << joint_name;
    }
    this->policy_debug_csv_file << "\n";
    this->policy_debug_csv_file.flush();

    std::cout << LOGGER::INFO << "Policy debug CSV logging to: " << csv_path << std::endl;
}

void RL_Sim::WritePolicyDebugCsv()
{
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

    this->policy_debug_csv_file
        << this->episode_length_buf << ","
        << this->ros2_node->now().seconds() << ","
        << this->obs.commands[0] << ","
        << this->obs.commands[1] << ","
        << this->obs.commands[2] << ","
        << this->obs.lin_vel[0] << ","
        << this->obs.lin_vel[1] << ","
        << this->obs.lin_vel[2] << ","
        << this->obs.ang_vel[0] << ","
        << this->obs.ang_vel[1] << ","
        << this->obs.ang_vel[2] << ","
        << this->obs.foot_contacts[0] << ","
        << this->obs.foot_contacts[1] << ","
        << this->obs.foot_contacts[2] << ","
        << this->obs.foot_contacts[3];

    const int num_dofs = this->params.Get<int>("num_of_dofs");
    for (int i = 0; i < num_dofs; ++i)
    {
        const float action = i < static_cast<int>(this->obs.actions.size()) ? this->obs.actions[i] : 0.0f;
        const float target_q = i < static_cast<int>(this->output_dof_pos.size()) ? this->output_dof_pos[i] : 0.0f;
        const float actual_q = i < static_cast<int>(this->obs.dof_pos.size()) ? this->obs.dof_pos[i] : 0.0f;
        const float actual_dq = i < static_cast<int>(this->obs.dof_vel.size()) ? this->obs.dof_vel[i] : 0.0f;
        const float target_tau = i < static_cast<int>(this->output_dof_tau.size()) ? this->output_dof_tau[i] : 0.0f;
        this->policy_debug_csv_file
            << "," << action
            << "," << target_q
            << "," << actual_q
            << "," << (target_q - actual_q)
            << "," << actual_dq
            << "," << target_tau;
    }
    this->policy_debug_csv_file << "\n";

    if ((this->policy_debug_csv_counter % 25) == 0)
    {
        this->policy_debug_csv_file.flush();
    }
}

void RL_Sim::ClosePolicyDebugCsv()
{
    if (this->policy_debug_csv_file.is_open())
    {
        this->policy_debug_csv_file.flush();
        this->policy_debug_csv_file.close();
    }
}

void RL_Sim::UpdateFaultBlend()
{
    if (!this->fault_switch_enabled || !this->recovery_model)
    {
        this->fault_blend = 0.0f;
        return;
    }

    bool active_fault = false;
    {
        std::lock_guard<std::mutex> lock(this->fault_mutex);
        active_fault = this->fault_detected;
    }

    const float dt = this->params.Get<float>("dt", 0.005f) * static_cast<float>(this->params.Get<int>("decimation", 4));
    const float blend_time = std::max(0.001f, this->params.Get<float>("fault_recovery_blend_time", 1.0f));
    const float blend_step = dt / blend_time;

    if (active_fault)
    {
        this->fault_blend = std::min(1.0f, this->fault_blend + blend_step);
    }
    else
    {
        this->fault_blend = std::max(0.0f, this->fault_blend - blend_step);
    }
}

std::vector<float> RL_Sim::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    const std::string model_forward_mode = this->params.Get<std::string>("model_forward_mode", "single");
    std::vector<std::vector<float>> model_input;
    std::vector<std::vector<int64_t>> model_input_shapes;
    if (model_forward_mode == "obs_history")
    {
        const auto observations_history = this->params.Get<std::vector<int>>("observations_history");
        if (observations_history.empty())
        {
            throw std::runtime_error("model_forward_mode=obs_history requires observations_history");
        }
        if (this->params.Get<bool>("warm_start_history", false) && this->episode_length_buf <= 1)
        {
            this->history_obs_buf.reset({0}, clamped_obs);
        }
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(observations_history);

        const int64_t obs_dim = static_cast<int64_t>(this->params.Get<int>("num_observations", static_cast<int>(clamped_obs.size())));
        const int64_t history_length = static_cast<int64_t>(this->params.Get<int>("history_length", static_cast<int>(observations_history.size())));
        model_input = {clamped_obs, this->history_obs};
        model_input_shapes = {{1, obs_dim}, {1, history_length, obs_dim}};
    }
    else if (this->params.Get<std::vector<int>>("observations_history").size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        model_input = {this->history_obs};
        model_input_shapes = {{1, static_cast<int64_t>(this->history_obs.size())}};
    }
    else
    {
        model_input = {clamped_obs};
        model_input_shapes = {{1, static_cast<int64_t>(clamped_obs.size())}};
    }

    std::vector<float> actions = this->model->forward_with_shapes(model_input, model_input_shapes);
    if (model_forward_mode != "obs_history" && this->fault_switch_enabled && this->recovery_model && this->fault_blend > 0.0f)
    {
        std::vector<float> recovery_actions = this->recovery_model->forward_with_shapes(model_input, model_input_shapes);
        if (recovery_actions.size() == actions.size())
        {
            for (size_t i = 0; i < actions.size(); ++i)
            {
                actions[i] = (1.0f - this->fault_blend) * actions[i] + this->fault_blend * recovery_actions[i];
            }
        }
        else
        {
            std::cout << LOGGER::WARNING << "Recovery policy action size mismatch; using normal policy action" << std::endl;
        }
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

void RL_Sim::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->robot_state_subscriber_msg.motor_state[i].q);
        this->plot_target_joint_pos[i].push_back(this->robot_command_publisher_msg.motor_command[i].q);
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.01);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Sim>(argc, argv);
    rclcpp::spin(rl_sar->ros2_node);
    rclcpp::shutdown();
    return 0;
}
