#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <cmath>
#include <unordered_map>

namespace isaaclab
{
// keyboard velocity commands example
// change "velocity_commands" observation name in policy deploy.yaml to "keyboard_velocity_commands"
REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    std::string key = FSMState::keyboard->key();
    static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    static std::unordered_map<std::string, std::vector<float>> key_commands = {
        {"w", {1.0f, 0.0f, 0.0f}},
        {"s", {-1.0f, 0.0f, 0.0f}},
        {"a", {0.0f, 1.0f, 0.0f}},
        {"d", {0.0f, -1.0f, 0.0f}},
        {"q", {0.0f, 0.0f, 1.0f}},
        {"e", {0.0f, 0.0f, -1.0f}}
    };
    std::vector<float> cmd = {0.0f, 0.0f, 0.0f};
    if (key_commands.find(key) != key_commands.end())
    {
        // TODO: smooth and limit the velocity commands
        cmd = key_commands[key];
    }
    return cmd;
}

}

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string) 
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    max_policy_step_ms_ = cfg["max_policy_step_ms"].as<int>(40);
    max_target_delta_per_cycle_ = cfg["max_target_delta_per_cycle"].as<float>(0.02f);
    if (max_policy_step_ms_ <= 0 || !std::isfinite(max_target_delta_per_cycle_) ||
        max_target_delta_per_cycle_ <= 0.0f) {
        throw std::runtime_error("Invalid BlindWalk timing or target-rate safety threshold");
    }

    register_safety_check(
        [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
        FSMStringMap.right.at("Passive"));
    register_safety_check(
        [&]()->bool{ return has_policy_fault(); },
        FSMStringMap.right.at("Passive"));
}

void State_RLBase::enter()
{
    for (int i = 0; i < env->robot->data.joint_stiffness.size(); ++i)
    {
        lowcmd->msg_.motor_cmd()[i].kp() = env->robot->data.joint_stiffness[i];
        lowcmd->msg_.motor_cmd()[i].kd() = env->robot->data.joint_damping[i];
        lowcmd->msg_.motor_cmd()[i].dq() = 0;
        lowcmd->msg_.motor_cmd()[i].tau() = 0;
    }

    {
        std::lock_guard<std::mutex> lock(policy_fault_mutex_);
        policy_fault_reason_.clear();
    }
    policy_fault_.store(false);
    env->reset();

    last_joint_targets_.resize(env->robot->data.joint_ids_map.size());
    {
        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        for (std::size_t i = 0; i < last_joint_targets_.size(); ++i) {
            const auto sdk_id = static_cast<std::size_t>(env->robot->data.joint_ids_map[i]);
            last_joint_targets_[i] = lowstate->msg_.motor_state()[sdk_id].q();
        }
    }
    policy_thread_running.store(true);
    policy_thread = std::thread([this]{
        using clock = std::chrono::steady_clock;
        const auto dt = std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(env->step_dt));
        auto sleep_until = clock::now() + dt;

        try {
            while (policy_thread_running.load())
            {
                const auto started_at = clock::now();
                env->step();
                const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    clock::now() - started_at);
                if (elapsed.count() > max_policy_step_ms_) {
                    throw std::runtime_error(
                        "Policy step exceeded deadline: " + std::to_string(elapsed.count()) + " ms");
                }

                std::this_thread::sleep_until(sleep_until);
                sleep_until += dt;
                if (sleep_until < clock::now()) {
                    sleep_until = clock::now() + dt;
                }
            }
        } catch (const std::exception& error) {
            set_policy_fault(error.what());
        }
        policy_thread_running.store(false);
    });
}

void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();
    if (action.size() != env->robot->data.joint_ids_map.size() ||
        action.size() != last_joint_targets_.size()) {
        set_policy_fault("Processed action size does not match joint map");
        return;
    }
    for(std::size_t i = 0; i < env->robot->data.joint_ids_map.size(); ++i) {
        const float delta = std::clamp(
            action[i] - last_joint_targets_[i],
            -max_target_delta_per_cycle_,
            max_target_delta_per_cycle_);
        last_joint_targets_[i] += delta;
        const auto sdk_id = static_cast<std::size_t>(env->robot->data.joint_ids_map[i]);
        lowcmd->msg_.motor_cmd()[sdk_id].q() = last_joint_targets_[i];
    }
}

void State_RLBase::exit()
{
    policy_thread_running.store(false);
    if (policy_thread.joinable()) {
        policy_thread.join();
    }
}

void State_RLBase::set_policy_fault(const std::string& reason)
{
    {
        std::lock_guard<std::mutex> lock(policy_fault_mutex_);
        policy_fault_reason_ = reason;
    }
    policy_fault_.store(true);
    spdlog::error("{} policy fault: {}", getStateString(), reason);
}

std::string State_RLBase::policy_fault_reason() const
{
    std::lock_guard<std::mutex> lock(policy_fault_mutex_);
    return policy_fault_reason_;
}
