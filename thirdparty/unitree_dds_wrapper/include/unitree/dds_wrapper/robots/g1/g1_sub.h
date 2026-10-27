// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <algorithm>
#include <cstring>
#include "unitree/dds_wrapper/common/Subscription.h"
#include "unitree/dds_wrapper/common/unitree_joystick.hpp"

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

namespace unitree
{
namespace robot
{
namespace g1
{
namespace subscription
{

class LowCmd : public SubscriptionBase<unitree_hg::msg::dds_::LowCmd_>
{
public:
    using SharedPtr = std::shared_ptr<LowCmd>;

    LowCmd(std::string topic = "rt/lowcmd") : SubscriptionBase<MsgType>(topic) {}

};

class LowState : public SubscriptionBase<unitree_hg::msg::dds_::LowState_>
{
public:
    using SharedPtr = std::shared_ptr<LowState>;

    LowState(std::string topic = "rt/lowstate") : SubscriptionBase<MsgType>(topic) {}

    void update()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // ********** Joystick ********** //
        // Check if all joystick values are zero to determine if the joystick is inactive
        if(std::all_of(msg_.wireless_remote().begin(), msg_.wireless_remote().end(), [](uint8_t i){return i == 0;}))
        {
            auto now = std::chrono::system_clock::now();
            auto elasped_time = now - last_joystick_time_;
            if(elasped_time > std::chrono::milliseconds(joystick_timeout_ms_))
            {
                isJoystickTimeout_ = true;
            }
        } else {
            last_joystick_time_ = std::chrono::system_clock::now();
            isJoystickTimeout_ = false;
        }

        // update joystick state
        unitree::common::REMOTE_DATA_RX key;
        memcpy(&key, &msg_.wireless_remote()[0], 40);
        joystick.extract(key);
    }

    bool isJoystickTimeout() const  { return isJoystickTimeout_; }
    unitree::common::UnitreeJoystick joystick;

private:
    uint32_t joystick_timeout_ms_ = 3000;
    bool isJoystickTimeout_ = false;
    std::chrono::time_point<std::chrono::system_clock> last_joystick_time_;
};

} // namespace subscription
} // namespace g1
} // namespace robot
} // namespace unitree
