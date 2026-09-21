// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"
#include <atomic>
#include <mutex>

class State_RLBase : public FSMState
{
public:
    State_RLBase(int state_mode, std::string state_string);
    
    void enter();

    void run();
    
    void exit();

    bool has_policy_fault() const { return policy_fault_.load(); }
    std::string policy_fault_reason() const;

private:
    void set_policy_fault(const std::string& reason);

    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;

    std::thread policy_thread;
    std::atomic_bool policy_thread_running{false};
    std::atomic_bool policy_fault_{false};
    mutable std::mutex policy_fault_mutex_;
    std::string policy_fault_reason_;
    int max_policy_step_ms_ = 40;
    float max_target_delta_per_cycle_ = 0.02f;
    std::vector<float> last_joint_targets_;
};

REGISTER_FSM(State_RLBase)
