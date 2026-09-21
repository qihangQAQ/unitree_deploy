// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <unitree/common/thread/recurrent_thread.hpp>
#include "BaseState.h"
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

class CtrlFSM
{
public:
    CtrlFSM(std::shared_ptr<BaseState> initstate)
    {
        // Initialize FSM states
        states.push_back(std::move(initstate));

    }

    CtrlFSM(YAML::Node cfg)
    {
        auto fsms = cfg["_"]; // enabled FSMs

        // register FSM string map; used for state transition
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            FSMStringMap.insert({id, fsm_name});
        }

        // Initialize FSM states
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            std::string fsm_type = it->second["type"] ? it->second["type"].as<std::string>() : fsm_name;
            auto fsm_class = getFsmMap().find("State_" + fsm_type);
            if (fsm_class == getFsmMap().end()) {
                throw std::runtime_error("FSM: Unknown FSM type " + fsm_type);
            }
            auto state_instance = fsm_class->second(id, fsm_name);
            add(state_instance);
        }
    }

    void start() 
    {
        // Start From State_Passive
        currentState = states[0];
        while (!currentState->pre_run()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        currentState->enter();
        currentState->post_run();
        current_state_id_.store(currentState->getState());

        fsm_thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "FSM", 0, this->dt * 1e6, &CtrlFSM::run_, this);
        spdlog::info("FSM: Start {}", currentState->getStateString());
    }

    void request_passive()
    {
        emergency_stop_.store(true);
    }

    void stop(std::chrono::milliseconds damping_duration = std::chrono::milliseconds(500))
    {
        if (!fsm_thread_) {
            return;
        }
        request_passive();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        const int passive_id = FSMStringMap.right.at("Passive");
        while (current_state_id_.load() != passive_id &&
               std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        std::this_thread::sleep_for(damping_duration);
        fsm_thread_.reset();
        if (currentState) {
            currentState->exit();
        }
        spdlog::info("FSM: Stopped in damping mode");
    }

    void add(std::shared_ptr<BaseState> state)
    {
        for(auto & s : states)
        {
            if(s->isState(state->getState()))
            {
                spdlog::error("FSM: State_{} already exists", state->getStateString());
                std::exit(0);
            }
        }

        states.push_back(std::move(state));
    }
    
    ~CtrlFSM()
    {
        stop();
        states.clear();
    }

    std::vector<std::shared_ptr<BaseState>> states;
private:
    const double dt = 0.001;

    void run_()
    {
        if (!currentState->pre_run()) {
            return;
        }
        currentState->run();
        
        // Check if need to change state
        int nextStateMode = 0;
        if (emergency_stop_.load()) {
            nextStateMode = FSMStringMap.right.at("Passive");
        } else {
            for(std::size_t i = 0; i < currentState->registered_checks.size(); ++i)
            {
                if(currentState->registered_checks[i].first())
                {
                    nextStateMode = currentState->registered_checks[i].second;
                    break;
                }
            }
        }

        currentState->post_run();

        if(nextStateMode != 0 && !currentState->isState(nextStateMode))
        {
            for(auto & state : states)
            {
                if(state->isState(nextStateMode))
                {
                    std::string rejection_reason;
                    if (!state->can_enter(rejection_reason)) {
                        const auto now = std::chrono::steady_clock::now();
                        if (nextStateMode != last_rejected_state_ ||
                            now - last_rejection_log_ > std::chrono::seconds(1)) {
                            spdlog::warn("FSM: Refusing transition from {} to {}: {}",
                                currentState->getStateString(), state->getStateString(), rejection_reason);
                            last_rejected_state_ = nextStateMode;
                            last_rejection_log_ = now;
                        }
                        break;
                    }
                    spdlog::info("FSM: Change state from {} to {}", currentState->getStateString(), state->getStateString());
                    currentState->exit();
                    currentState = state;
                    while (!currentState->pre_run()) {
                        std::this_thread::yield();
                    }
                    currentState->enter();
                    currentState->post_run();
                    current_state_id_.store(currentState->getState());
                    break;
                }
            }
        }
    }

    std::shared_ptr<BaseState> currentState;
    unitree::common::RecurrentThreadPtr fsm_thread_;
    std::atomic_bool emergency_stop_{false};
    std::atomic<int> current_state_id_{0};
    int last_rejected_state_ = 0;
    std::chrono::steady_clock::time_point last_rejection_log_{};
};
