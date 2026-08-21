/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_GO2_HPP
#define RL_REAL_GO2_HPP

// #define PLOT
// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_go2.hpp"
#include "go2_wireless_remote.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <chrono>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>

#if defined(USE_ROS)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#endif

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_JOYSTICK "rt/wirelesscontroller"
constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

// union for joystick keys
typedef union
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

class RL_Real : public RL
{
public:
    RL_Real(int argc, char **argv);
    ~RL_Real();
    void Shutdown();

#if defined(USE_ROS)
    std::shared_ptr<rclcpp::Node> ros2_node;
#endif

private:
    enum class RealTuckState : int
    {
        Disabled = 0,
        Idle = 1,
        Armed = 2,
        Tucking = 3,
        Locked = 4,
        Aborted = 5,
    };

    enum class RealGainFaultState : int
    {
        Disabled = 0,
        Idle = 1,
        Armed = 2,
        Ramping = 3,
        Active = 4,
        Completed = 5,
        Aborted = 6,
    };

    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();
    void InitPolicyDebugCsv();
    void WritePolicyDebugCsv();
    void ClosePolicyDebugCsv();
    void HandleRealTuckControlInput();
    void ApplyRealTuckCommandGuard();
    void ApplyRealTuckedLegFault();
    bool CheckRealTuckArmConditions(std::string& reason) const;
    bool ResolveRealTuckJoints(std::string& reason);
    double LowStateAgeSeconds() const;
    void RequestRealTuckAbort(const std::string& reason);
    void HandleRealGainFaultControlInput();
    void ApplyRealGainFaultCommandGuard();
    void UpdateRealGainFault();
    bool CheckRealGainFaultArmConditions(std::string& reason) const;
    bool SelectRealGainFaultJoint(std::string& reason);
    void RestoreRealGainFault(const std::string& reason, bool request_getdown);

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

    // unitree interface
    void InitLowCmd();
    int QueryMotionStatus();
    std::string QueryServiceName(std::string form, std::string name);
    uint32_t Crc32Core(uint32_t *ptr, uint32_t len);
    void LowStateMessageHandler(const void *messages);
    bool MoveToProneForShutdown();
    void PublishSafeMotorRelease();
    MotionSwitcherClient msc;
    unitree_go::msg::dds_::LowCmd_ unitree_low_command{};
    unitree_go::msg::dds_::LowState_ unitree_low_state{};
    unitree_go::msg::dds_::WirelessController_ joystick{};
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    xKeySwitchUnion unitree_joy;
    bool policy_debug_csv_initialized = false;
    bool policy_debug_csv_enabled = false;
    int policy_debug_csv_stride = 1;
    int policy_debug_csv_counter = 0;
    std::ofstream policy_debug_csv_file;

    // Explicitly gated real-robot fault experiment. This state is never active
    // during normal real deployment.
    bool real_tuck_launch_enabled = false;
    RealTuckState real_tuck_state = RealTuckState::Disabled;
    std::mutex real_tuck_mutex;
    std::atomic<bool> real_tuck_abort_requested{false};
    std::atomic<int64_t> last_lowstate_time_ns{0};
    std::atomic<bool> shutdown_started{false};
    std::atomic<bool> shutdown_prone_requested{false};
    std::atomic<bool> shutdown_prone_complete{false};
    std::atomic<bool> shutdown_getdown_started{false};
    std::chrono::steady_clock::time_point real_tuck_arm_deadline{};
    std::vector<int> real_tuck_joint_indices;
    std::vector<float> real_tuck_start_q;
    std::vector<float> real_tuck_target_q;
    float real_tuck_elapsed_s = 0.0f;
    float real_tuck_locked_elapsed_s = 0.0f;
    float real_tuck_blend = 0.0f;
    float real_tuck_violation_elapsed_s = 0.0f;
    std::string real_tuck_abort_reason;

    // Explicitly gated real-robot residual-authority experiment. It scales
    // the PD gains for one selected joint and is inert during normal runs.
    bool real_gain_fault_launch_enabled = false;
    bool real_gain_fault_arm_attempted = false;
    RealGainFaultState real_gain_fault_state = RealGainFaultState::Disabled;
    std::mutex real_gain_fault_mutex;
    std::atomic<bool> real_gain_fault_getdown_requested{false};
    std::chrono::steady_clock::time_point real_gain_fault_arm_deadline{};
    int real_gain_fault_policy_index = -1;
    int real_gain_fault_hardware_index = -1;
    std::string real_gain_fault_joint_name;
    float real_gain_fault_target_alpha = 1.0f;
    float real_gain_fault_current_alpha = 1.0f;
    float real_gain_fault_ramp_elapsed_s = 0.0f;
    float real_gain_fault_active_elapsed_s = 0.0f;
    float real_gain_fault_violation_elapsed_s = 0.0f;
    std::string real_gain_fault_abort_reason;

    // others
    std::vector<float> mapped_joint_positions;
    std::vector<float> mapped_joint_velocities;

#if defined(USE_ROS)
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
#endif
};

#endif // RL_REAL_GO2_HPP
