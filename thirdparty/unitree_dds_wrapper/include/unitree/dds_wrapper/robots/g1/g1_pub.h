// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <unitree/dds_wrapper/common/Publisher.h>
#include "unitree/dds_wrapper/common/crc.h"

#include <unitree/idl/hg/LowCmd_.hpp>

#include "g1_sub.h"


namespace unitree
{
namespace robot
{
namespace g1
{
namespace publisher
{

class LowCmd : public RealTimePublisher<unitree_hg::msg::dds_::LowCmd_>
{
public:
    LowCmd(std::string topic = "rt/lowcmd") : RealTimePublisher<MsgType>(topic)
    {
    }

    bool check_mode_machine(subscription::LowState::SharedPtr lowstate = nullptr) const
    {
        auto sub = lowstate == nullptr ? std::make_shared<subscription::LowState>() : lowstate;
        sub->wait_for_connection();
        auto m_sub = sub->msg_.mode_machine();
        auto m_pub = msg_.mode_machine();

        // 0: simulation environment
        return !(m_sub != 0 && m_sub != m_pub);
    }
private:
    /**
     * @brief Something before sending the message.
     */
    void pre_communication() override {
        msg_.crc() = crc32_core((uint32_t*)&msg_, (sizeof(MsgType)>>2)-1);
    }
};

}; // namespace publisher
}; // namespace g1
}; // namespace robot
}; // namespace unitree
