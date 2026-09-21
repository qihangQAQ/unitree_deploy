// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"

class State_Passive : public FSMState
{
public:
    State_Passive(int state, std::string state_string = "Passive") 
    : FSMState(state, state_string) 
    {
        auto motor_mode = param::config["FSM"]["Passive"]["mode"];
        if(motor_mode.IsDefined())
        {
            motor_mode_ = motor_mode.as<std::vector<int>>();
        }
    } 

    void enter()
    {
        for(int i(0); i < motor_mode_.size(); ++i) {
            lowcmd->msg_.motor_cmd()[i].mode() = motor_mode_[i];
        }
        // set gain
        static auto kd = param::config["FSM"]["Passive"]["kd"].as<std::vector<float>>();
        for(int i(0); i < kd.size(); ++i)
        {
            auto & motor = lowcmd->msg_.motor_cmd()[i];
            motor.kp() = 0;
            motor.kd() = kd[i];
            motor.dq() = 0;
            motor.tau() = 0;
        }
    }

    void run()
    {
        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        for(int i(0); i < lowcmd->msg_.motor_cmd().size(); ++i)
        {
            lowcmd->msg_.motor_cmd()[i].q() = lowstate->msg_.motor_state()[i].q();
        }
    }

private:
    std::vector<int> motor_mode_;
};

REGISTER_FSM(State_Passive)
