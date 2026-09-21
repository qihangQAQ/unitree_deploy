#include "State_RLDepth.h"

#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/observations/depth_observations.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/terminations.h"
#include "sensors/depth_config.h"
#include "sensors/replay_depth_source.h"
#include "unitree_articulation.h"

#ifdef UNITREE_DEPLOY_WITH_ROS2
#include "Ros2DepthSource.h"
#endif

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>

namespace
{

template <typename T>
T yaml_or(const YAML::Node& node, const char* key, const T& default_value)
{
    return node && node[key] ? node[key].as<T>() : default_value;
}

sensors::DepthPreprocessorConfig make_preprocessor_config()
{
    return sensors::load_depth_preprocessor_config(param::config["depth"]);
}

bool shape_equals(const std::vector<std::int64_t>& actual,
                  std::initializer_list<std::int64_t> expected)
{
    return actual == std::vector<std::int64_t>(expected);
}

} // namespace

State_RLDepth::State_RLDepth(int state_mode, std::string state_string)
    : FSMState(state_mode, std::move(state_string)),
      external_observations_(std::make_shared<isaaclab::ExternalObservationStore>()),
      preprocessor_(make_preprocessor_config())
{
    const auto depth_cfg = param::config["depth"];
    stale_timeout_ = std::chrono::milliseconds(
        yaml_or<int>(depth_cfg, "stale_timeout_ms", 100));
    min_valid_ratio_ = yaml_or<float>(depth_cfg, "min_valid_ratio", 0.01f);
    max_consecutive_invalid_frames_ = yaml_or<int>(
        depth_cfg, "max_consecutive_invalid_frames", 5);
    require_camera_info_ = yaml_or<bool>(depth_cfg, "require_camera_info", true);
    strict_intrinsics_ = yaml_or<bool>(depth_cfg, "strict_intrinsics", true);
    max_target_delta_per_cycle_ = yaml_or<float>(
        depth_cfg, "max_target_delta_per_cycle", 0.02f);
    if (stale_timeout_.count() <= 0 || !std::isfinite(min_valid_ratio_) ||
        min_valid_ratio_ < 0.0f || min_valid_ratio_ > 1.0f ||
        max_consecutive_invalid_frames_ <= 0 || !std::isfinite(max_target_delta_per_cycle_) ||
        max_target_delta_per_cycle_ <= 0.0f) {
        throw std::runtime_error("Invalid depth safety thresholds in config.yaml");
    }

    external_observations_->set(
        "depth", std::vector<float>(preprocessor_.output_elements(), -1.0f));
    create_depth_source(depth_cfg);
    load_policy_if_available(param::config["FSM"][getStateString()]);

    register_safety_check(
        [this] { return faulted_.load(); },
        FSMStringMap.right.at("Passive"));
    register_safety_check(
        [this] {
            return depth_source_ && policy_thread_running_.load() &&
                   depth_source_->stale(stale_timeout_);
        },
        FSMStringMap.right.at("Passive"));
    register_safety_check(
        [this] {
            return env_ && isaaclab::mdp::bad_orientation(env_.get(), 1.0);
        },
        FSMStringMap.right.at("Passive"));
}

State_RLDepth::~State_RLDepth()
{
    exit();
    if (depth_source_) {
        depth_source_->stop();
    }
}

void State_RLDepth::create_depth_source(const YAML::Node& cfg)
{
    const std::string source_type = yaml_or<std::string>(cfg, "source", "ros2");
    if (source_type == "replay") {
        sensors::ReplayDepthSourceConfig replay;
        const auto replay_cfg = cfg["replay"];
        replay.file = yaml_or<std::string>(replay_cfg, "file", "");
        if (replay.file.is_relative()) {
            replay.file = param::proj_dir / replay.file;
        }
        replay.width = yaml_or<int>(replay_cfg, "width", 0);
        replay.height = yaml_or<int>(replay_cfg, "height", 0);
        replay.fps = yaml_or<float>(replay_cfg, "fps", 60.0f);
        replay.loop = yaml_or<bool>(replay_cfg, "loop", true);
        depth_source_ = std::make_unique<sensors::ReplayDepthSource>(std::move(replay));
        require_camera_info_ = false;
    } else if (source_type == "ros2") {
#ifdef UNITREE_DEPLOY_WITH_ROS2
        sensors::Ros2DepthSourceConfig ros;
        ros.image_topic = yaml_or<std::string>(
            cfg, "topic", "/camera/depth/image_rect_raw");
        ros.camera_info_topic = yaml_or<std::string>(
            cfg, "camera_info_topic", "/camera/depth/camera_info");
        if (const char* topic = std::getenv("UNITREE_DEPTH_TOPIC"); topic && *topic) {
            ros.image_topic = topic;
        }
        if (const char* topic = std::getenv("UNITREE_DEPTH_CAMERA_INFO_TOPIC"); topic && *topic) {
            ros.camera_info_topic = topic;
        }
        depth_source_ = std::make_unique<sensors::Ros2DepthSource>(std::move(ros));
#else
        unavailable_reason_ =
            "binary was built without ROS2 support (configure with UNITREE_DEPLOY_WITH_ROS2=ON)";
#endif
    } else {
        unavailable_reason_ = "unknown depth source type '" + source_type + "'";
    }

    if (depth_source_ && !depth_source_->start()) {
        unavailable_reason_ = "depth source failed to start: " + depth_source_->status();
    }
}

void State_RLDepth::load_policy_if_available(const YAML::Node& cfg)
{
    if (!cfg || !cfg["policy_dir"]) {
        unavailable_reason_ = "DepthWalk policy_dir is not configured";
        return;
    }

    std::filesystem::path policy_dir = cfg["policy_dir"].as<std::string>();
    if (policy_dir.is_relative()) {
        policy_dir = param::proj_dir / policy_dir;
    }
    const auto deploy_path = policy_dir / "params" / "deploy.yaml";
    const auto model_path = policy_dir / "exported" / "policy.onnx";
    if (!std::filesystem::is_regular_file(deploy_path) ||
        !std::filesystem::is_regular_file(model_path)) {
        unavailable_reason_ =
            "missing depth policy artifacts under " + policy_dir.string();
        spdlog::warn("DepthWalk disabled: {}", unavailable_reason_);
        return;
    }

    try {
        auto env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
            YAML::LoadFile(deploy_path.string()),
            std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate),
            external_observations_);
        auto runner = std::make_unique<isaaclab::OrtRunner>(model_path);

        const auto* policy_input = runner->find_input("policy");
        const auto* depth_input = runner->find_input("depth");
        if (runner->inputs().size() != 2 || !policy_input || !depth_input ||
            !shape_equals(policy_input->shape, {1, 480}) ||
            !shape_equals(depth_input->shape, {1, 16, 24, 1}) ||
            runner->output().name != "actions" ||
            !shape_equals(runner->output().shape, {1, 29})) {
            throw std::runtime_error(
                "expected ONNX contract policy[1,480], depth[1,16,24,1], actions[1,29]");
        }
        if (env->action_manager->total_action_dim() != 29 ||
            env->robot->data.joint_ids_map.size() != 29) {
            throw std::runtime_error("depth deploy.yaml must define 29 actions and 29 joint ids");
        }

        max_policy_step_ms_ = yaml_or<int>(cfg, "max_policy_step_ms", 40);
        if (max_policy_step_ms_ <= 0) {
            throw std::runtime_error("DepthWalk max_policy_step_ms must be positive");
        }
        env->alg = std::move(runner);
        env_ = std::move(env);
        policy_available_ = true;
        unavailable_reason_.clear();
        spdlog::info("DepthWalk policy contract validated: {}", model_path.string());
    } catch (const std::exception& error) {
        unavailable_reason_ = std::string("invalid depth policy: ") + error.what();
        spdlog::error("DepthWalk disabled: {}", unavailable_reason_);
    }
}

bool State_RLDepth::can_enter(std::string& reason)
{
    if (!policy_available_ || !env_) {
        reason = unavailable_reason_.empty() ? "depth policy is unavailable" : unavailable_reason_;
        return false;
    }
    if (!depth_source_) {
        reason = unavailable_reason_.empty() ? "depth source is unavailable" : unavailable_reason_;
        return false;
    }
    if (!depth_source_->ready()) {
        reason = "waiting for first depth frame: " + depth_source_->status();
        return false;
    }
    if (depth_source_->stale(stale_timeout_)) {
        reason = "latest depth frame is stale";
        return false;
    }

    try {
        const auto frame = depth_source_->latest();
        if (!frame) {
            reason = "no depth frame is available";
            return false;
        }
        const auto processed = preprocessor_.process(*frame);
        const float valid_ratio = static_cast<float>(processed.valid_count) / processed.data.size();
        if (valid_ratio < min_valid_ratio_) {
            reason = "depth valid-pixel ratio is too low";
            return false;
        }
        if (require_camera_info_) {
            const auto info = depth_source_->camera_info();
            if (!info) {
                reason = "waiting for depth CameraInfo";
                return false;
            }
            if (info->width != frame->width || info->height != frame->height) {
                reason = "depth image and CameraInfo dimensions do not match";
                return false;
            }
            std::string mismatch;
            if (!preprocessor_.intrinsics_match(*info, &mismatch)) {
                if (strict_intrinsics_) {
                    reason = mismatch;
                    return false;
                }
                spdlog::warn("Depth camera intrinsics warning: {}", mismatch);
            }
        }
    } catch (const std::exception& error) {
        reason = error.what();
        return false;
    }
    return true;
}

void State_RLDepth::enter()
{
    if (!env_) {
        set_fault("DepthWalk entered without a loaded policy");
        return;
    }
    try {
        {
            std::lock_guard<std::mutex> lock(fault_mutex_);
            fault_reason_.clear();
        }
        faulted_.store(false);

        const auto first_frame = update_depth_observation();
        (void)first_frame;
        env_->reset();

        for (int i = 0; i < env_->robot->data.joint_stiffness.size(); ++i) {
            lowcmd->msg_.motor_cmd()[i].kp() = env_->robot->data.joint_stiffness[i];
            lowcmd->msg_.motor_cmd()[i].kd() = env_->robot->data.joint_damping[i];
            lowcmd->msg_.motor_cmd()[i].dq() = 0.0f;
            lowcmd->msg_.motor_cmd()[i].tau() = 0.0f;
        }

        last_joint_targets_.resize(env_->robot->data.joint_ids_map.size());
        {
            std::lock_guard<std::mutex> lock(lowstate->mutex_);
            for (std::size_t i = 0; i < last_joint_targets_.size(); ++i) {
                const auto sdk_id = static_cast<std::size_t>(env_->robot->data.joint_ids_map[i]);
                last_joint_targets_[i] = lowstate->msg_.motor_state()[sdk_id].q();
            }
        }

        policy_thread_running_.store(true);
        policy_thread_ = std::thread([this] {
            using clock = std::chrono::steady_clock;
            const auto period = std::chrono::duration_cast<clock::duration>(
                std::chrono::duration<double>(env_->step_dt));
            auto next = clock::now() + period;
            int consecutive_invalid_frames = 0;

            try {
                while (policy_thread_running_.load()) {
                    const auto started_at = clock::now();
                    if (depth_source_->stale(stale_timeout_)) {
                        throw std::runtime_error("depth frame timeout");
                    }
                    const auto processed = update_depth_observation();
                    const float valid_ratio =
                        static_cast<float>(processed.valid_count) / processed.data.size();
                    consecutive_invalid_frames = valid_ratio < min_valid_ratio_
                        ? consecutive_invalid_frames + 1
                        : 0;
                    if (consecutive_invalid_frames >= max_consecutive_invalid_frames_) {
                        throw std::runtime_error("too many consecutive invalid depth frames");
                    }

                    env_->step();
                    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                        clock::now() - started_at);
                    if (elapsed.count() > max_policy_step_ms_) {
                        throw std::runtime_error(
                            "depth policy step exceeded deadline: " +
                            std::to_string(elapsed.count()) + " ms");
                    }

                    std::this_thread::sleep_until(next);
                    next += period;
                    if (next < clock::now()) {
                        next = clock::now() + period;
                    }
                }
            } catch (const std::exception& error) {
                set_fault(error.what());
            }
            policy_thread_running_.store(false);
        });
    } catch (const std::exception& error) {
        policy_thread_running_.store(false);
        if (policy_thread_.joinable()) {
            policy_thread_.join();
        }
        set_fault(std::string("failed to enter DepthWalk: ") + error.what());
    }
}

sensors::ProcessedDepthFrame State_RLDepth::update_depth_observation()
{
    const auto frame = depth_source_ ? depth_source_->latest() : nullptr;
    if (!frame) {
        throw std::runtime_error("no depth frame is available");
    }
    auto processed = preprocessor_.process(*frame);
    external_observations_->set("depth", processed.data);
    return processed;
}

void State_RLDepth::run()
{
    if (!env_ || faulted_.load()) {
        return;
    }
    const auto targets = env_->action_manager->processed_actions();
    if (targets.size() != env_->robot->data.joint_ids_map.size() ||
        targets.size() != last_joint_targets_.size()) {
        set_fault("processed depth action size does not match joint map");
        return;
    }

    for (std::size_t i = 0; i < targets.size(); ++i) {
        if (!std::isfinite(targets[i])) {
            set_fault("processed depth action contains NaN or Inf");
            return;
        }
        const float delta = std::clamp(
            targets[i] - last_joint_targets_[i],
            -max_target_delta_per_cycle_,
            max_target_delta_per_cycle_);
        last_joint_targets_[i] += delta;
        const auto sdk_id = static_cast<std::size_t>(env_->robot->data.joint_ids_map[i]);
        lowcmd->msg_.motor_cmd()[sdk_id].q() = last_joint_targets_[i];
    }
}

void State_RLDepth::exit()
{
    policy_thread_running_.store(false);
    if (policy_thread_.joinable()) {
        policy_thread_.join();
    }
}

void State_RLDepth::set_fault(const std::string& reason)
{
    {
        std::lock_guard<std::mutex> lock(fault_mutex_);
        fault_reason_ = reason;
    }
    if (!faulted_.exchange(true)) {
        spdlog::error("DepthWalk fault: {}", reason);
    }
}

std::string State_RLDepth::fault_reason() const
{
    std::lock_guard<std::mutex> lock(fault_mutex_);
    return fault_reason_;
}
