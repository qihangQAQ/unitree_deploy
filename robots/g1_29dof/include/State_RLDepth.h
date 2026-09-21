#pragma once

#include "FSM/FSMState.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/external_observation_store.h"
#include "sensors/depth_preprocessor.h"
#include "sensors/depth_source.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class State_RLDepth : public FSMState
{
public:
    State_RLDepth(int state_mode, std::string state_string);
    ~State_RLDepth() override;

    bool can_enter(std::string& reason) override;
    void enter() override;
    void run() override;
    void exit() override;

private:
    void create_depth_source(const YAML::Node& cfg);
    void load_policy_if_available(const YAML::Node& cfg);
    sensors::ProcessedDepthFrame update_depth_observation();
    void set_fault(const std::string& reason);
    std::string fault_reason() const;

    std::shared_ptr<isaaclab::ExternalObservationStore> external_observations_;
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env_;
    std::unique_ptr<sensors::DepthSource> depth_source_;
    sensors::DepthPreprocessor preprocessor_;

    std::thread policy_thread_;
    std::atomic_bool policy_thread_running_{false};
    std::atomic_bool faulted_{false};
    mutable std::mutex fault_mutex_;
    std::string fault_reason_;
    std::string unavailable_reason_;
    std::vector<float> last_joint_targets_;

    bool policy_available_ = false;
    bool require_camera_info_ = true;
    bool strict_intrinsics_ = true;
    std::chrono::milliseconds stale_timeout_{100};
    float min_valid_ratio_ = 0.01f;
    int max_consecutive_invalid_frames_ = 5;
    int max_policy_step_ms_ = 40;
    float max_target_delta_per_cycle_ = 0.02f;
};

REGISTER_FSM(State_RLDepth)
