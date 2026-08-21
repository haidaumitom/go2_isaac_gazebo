/*
 * Copyright (c) 2024-2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "go2_wireless_remote.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

namespace
{
constexpr char kLowStateTopic[] = "rt/lowstate";

std::atomic<bool> running{true};

void HandleSignal(int)
{
    running = false;
}

class StateProbe
{
public:
    void Start()
    {
        lowstate_subscriber_.reset(
            new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(kLowStateTopic));
        lowstate_subscriber_->InitChannel(
            std::bind(&StateProbe::HandleLowState, this, std::placeholders::_1), 1);

    }

    void Stop()
    {
        if (lowstate_subscriber_)
        {
            lowstate_subscriber_->CloseChannel();
            lowstate_subscriber_.reset();
        }
    }

    void Print() const
    {
        unitree_go::msg::dds_::LowState_ lowstate;
        unitree_go::msg::dds_::WirelessController_ joystick;
        uint64_t lowstate_count = 0;
        uint64_t joystick_count = 0;
        uint64_t joystick_changes = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            lowstate = lowstate_;
            joystick = joystick_;
            lowstate_count = lowstate_count_;
            joystick_count = joystick_count_;
            joystick_changes = joystick_change_count_;
        }

        const auto& quaternion = lowstate.imu_state().quaternion();
        const float quaternion_norm = std::sqrt(
            quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1]
            + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);

        std::cout << std::fixed << std::setprecision(3)
                  << "lowstate_samples=" << lowstate_count
                  << " tick=" << lowstate.tick()
                  << " quat_norm=" << quaternion_norm
                  << " q0=" << lowstate.motor_state()[0].q()
                  << " q1=" << lowstate.motor_state()[1].q()
                  << " q2=" << lowstate.motor_state()[2].q()
                  << " motor_mode=[" << static_cast<int>(lowstate.motor_state()[0].mode()) << ","
                  << static_cast<int>(lowstate.motor_state()[1].mode()) << ","
                  << static_cast<int>(lowstate.motor_state()[2].mode()) << "]"
                  << " foot_force=[" << lowstate.foot_force()[0] << ","
                  << lowstate.foot_force()[1] << ","
                  << lowstate.foot_force()[2] << ","
                  << lowstate.foot_force()[3] << "]"
                  << " joystick_samples=" << joystick_count
                  << " joystick_changes=" << joystick_changes
                  << " axes=[" << joystick.lx() << "," << joystick.ly() << ","
                  << joystick.rx() << "," << joystick.ry() << "]"
                  << " keys=0x" << std::hex << joystick.keys() << std::dec
                  << std::endl;
    }

    uint64_t LowStateCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lowstate_count_;
    }

    uint64_t JoystickCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return joystick_change_count_;
    }

private:
    void HandleLowState(const void* message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto& incoming = *static_cast<const unitree_go::msg::dds_::LowState_*>(message);
        if (lowstate_count_ > 0 && incoming.wireless_remote() != lowstate_.wireless_remote())
        {
            ++joystick_change_count_;
        }
        lowstate_ = incoming;
        joystick_ = go2_wireless_remote::Decode(lowstate_.wireless_remote());
        ++lowstate_count_;
        ++joystick_count_;
    }

    mutable std::mutex mutex_;
    unitree_go::msg::dds_::LowState_ lowstate_{};
    unitree_go::msg::dds_::WirelessController_ joystick_{};
    uint64_t lowstate_count_ = 0;
    uint64_t joystick_count_ = 0;
    uint64_t joystick_change_count_ = 0;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber_;
};
}

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 3)
    {
        std::cerr << "Usage: " << argv[0] << " network_interface [duration_seconds]" << std::endl;
        return 1;
    }

    const int duration_seconds = argc == 3 ? std::max(1, std::atoi(argv[2])) : 10;
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    StateProbe probe;
    probe.Start();

    std::cout << "Read-only probe started on " << argv[1]
              << ". Move the hand-controller sticks during this " << duration_seconds
              << " second test." << std::endl;

    for (int elapsed = 0; running && elapsed < duration_seconds; ++elapsed)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        probe.Print();
    }

    int result = 0;
    if (probe.LowStateCount() == 0)
    {
        result = 2;
        std::cerr << "FAIL: no rt/lowstate samples received." << std::endl;
    }
    else if (probe.JoystickCount() == 0)
    {
        result = 3;
        std::cerr << "FAIL: no hand-controller changes found in LowState." << std::endl;
    }
    else
    {
        std::cout << "PASS: LowState and embedded hand-controller data are live." << std::endl;
    }

    // The bundled aarch64 CycloneDDS library aborts in its instrumented
    // teardown worker. The process boundary safely releases this read-only
    // probe's resources without entering that incompatible destructor path.
    std::cout.flush();
    std::cerr.flush();
    std::_Exit(result);
}
