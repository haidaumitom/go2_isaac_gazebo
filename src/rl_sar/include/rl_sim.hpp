#ifndef RL_SIM_HPP
#define RL_SIM_HPP

// #define PLOT
// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_all.hpp"

#include <csignal>
#include <vector>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class RL_Sim : public RL
{
public:
    RL_Sim(int argc, char **argv);
    ~RL_Sim();

    std::shared_ptr<rclcpp::Node> ros2_node;

private:
    // rl functions
    std::vector<float> Forward() override;
    void LoadRecoveryPolicy();
    void UpdateFaultBlend();
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();
    void InitPolicyDebugCsv();
    void WritePolicyDebugCsv();
    void ClosePolicyDebugCsv();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // ros interface
    std::string ros_namespace;
    sensor_msgs::msg::Imu gazebo_imu;
    geometry_msgs::msg::Twist cmd_vel;
    sensor_msgs::msg::Joy joy_msg;
    robot_msgs::msg::RobotCommand robot_command_publisher_msg;
    robot_msgs::msg::RobotState robot_state_subscriber_msg;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr fault_alpha_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gain_alpha_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr foot_contacts_subscriber;
    std::vector<rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr> gazebo_foot_contact_subscribers;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gazebo_imu_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gazebo_pause_physics_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gazebo_unpause_physics_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gazebo_reset_world_client;
    rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr robot_command_publisher;
    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr robot_state_subscriber;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_client;
    geometry_msgs::msg::Twist vel;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_state_subscriber;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_state_subscriber_root;
    rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr link_state_subscriber;
    rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr link_state_subscriber_root;
    void ModelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);
    void LinkStatesCallback(const gazebo_msgs::msg::LinkStates::SharedPtr msg);
    void GazeboImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void FaultAlphaCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void GainAlphaCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void FootContactsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void GazeboFootContactCallback(size_t foot_id, const gazebo_msgs::msg::ContactsState::SharedPtr msg);
    void RobotStateCallback(const robot_msgs::msg::RobotState::SharedPtr msg);
    void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // others
    std::unique_ptr<InferenceRuntime::Model> recovery_model;
    std::string recovery_policy_name;
    bool fault_switch_enabled = false;
    bool fault_detected = false;
    float fault_blend = 0.0f;
    std::vector<float> latest_fault_alpha;
    std::mutex fault_mutex;
    std::string gazebo_model_name;
    std::map<std::string, float> joint_positions;
    std::map<std::string, float> joint_velocities;
    std::map<std::string, float> joint_efforts;
    rclcpp::Time last_foot_contacts_topic_time;
    bool policy_debug_csv_initialized = false;
    bool policy_debug_csv_enabled = false;
    int policy_debug_csv_stride = 1;
    int policy_debug_csv_counter = 0;
    std::ofstream policy_debug_csv_file;
    void StartJointController(const std::string& ros_namespace, const std::vector<std::string>& names);
};

#endif // RL_SIM_HPP
