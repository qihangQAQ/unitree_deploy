#include "isaaclab/algorithms/algorithms.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/observations/depth_observations.h"
#include "isaaclab/envs/mdp/observations/observations.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::filesystem::path configured_path(
    const YAML::Node& config, const char* state, const char* key)
{
    const auto value = config["FSM"][state][key];
    if (!value.IsScalar()) {
        throw std::runtime_error(std::string(state) + " requires " + key);
    }
    std::filesystem::path path = value.as<std::string>();
    if (path.is_relative()) path = std::filesystem::path(TEST_PROJECT_DIR) / path;
    if (!std::filesystem::is_regular_file(path)) {
        throw std::runtime_error(std::string(state) + " file is missing: " + path.string());
    }
    return path;
}

class MockArticulation final : public isaaclab::Articulation
{
public:
    MockArticulation()
    {
        data.joystick = &joystick_;
        data.root_ang_vel_b.setZero();
        data.projected_gravity_b = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
        data.root_quat_w = Eigen::Quaternionf::Identity();
    }

    void update() override
    {
        data.joint_pos.setZero();
        data.joint_vel.setZero();
    }

private:
    unitree::common::UnitreeJoystick joystick_;
};
}

int main()
{
    const auto config = YAML::LoadFile(TEST_CONFIG_PATH);
    const auto blind_model = configured_path(config, "BlindWalk", "model_path");
    const auto blind_deploy = configured_path(config, "BlindWalk", "deploy_path");
    const auto depth_model = configured_path(config, "DepthWalk", "model_path");
    const auto depth_deploy = configured_path(config, "DepthWalk", "deploy_path");

    auto runner = std::make_unique<isaaclab::OrtRunner>(blind_model);
    require(runner->inputs().size() == 1, "unexpected ONNX input count");
    const auto& input = runner->inputs().front();
    require(input.name == "obs", "unexpected ONNX input name");
    require(input.shape == std::vector<std::int64_t>{1, 480} &&
            input.element_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
            "unexpected blind ONNX input contract");
    require(runner->output().name == "actions" &&
            runner->output().shape == std::vector<std::int64_t>{1, 29},
            "unexpected blind ONNX output contract");

    isaaclab::ObservationMap observations;
    observations[input.name] = std::vector<float>(input.element_count, 0.0f);
    const auto action = runner->act(observations);
    require(action.size() == 29, "wrong inference output length");
    require(std::all_of(action.begin(), action.end(),
        [](float value) { return std::isfinite(value); }), "inference returned NaN or Inf");

    bool rejected_bad_size = false;
    observations[input.name].resize(input.element_count - 1);
    try {
        runner->act(observations);
    } catch (const std::runtime_error&) {
        rejected_bad_size = true;
    }
    require(rejected_bad_size, "bad ONNX input size was accepted");

    auto robot = std::make_shared<MockArticulation>();
    isaaclab::ManagerBasedRLEnv env(YAML::LoadFile(blind_deploy.string()), robot);
    env.alg = std::move(runner);
    env.reset();
    const auto configured_observations = env.observation_manager->compute();
    const auto configured_input = configured_observations.find("obs");
    require(configured_observations.size() == 1 &&
            configured_input != configured_observations.end(),
            "deploy.yaml did not produce the ONNX 'obs' input");
    require(configured_input->second.size() == 480,
            "deploy.yaml observation history did not produce 480 values");
    env.step();
    const auto processed_action = env.action_manager->processed_actions();
    require(processed_action.size() == 29,
            "deploy.yaml action processing did not produce 29 joint targets");
    require(std::all_of(processed_action.begin(), processed_action.end(),
        [](float value) { return std::isfinite(value); }),
        "deploy.yaml action processing returned NaN or Inf");

    auto depth_runner = std::make_unique<isaaclab::OrtRunner>(depth_model);
    const auto* policy_input = depth_runner->find_input("policy");
    const auto* depth_input = depth_runner->find_input("depth");
    require(depth_runner->inputs().size() == 2 && policy_input && depth_input,
            "depth ONNX must have policy and depth inputs");
    require(policy_input->shape == std::vector<std::int64_t>{1, 480},
            "unexpected depth-policy proprioceptive input shape");
    require(depth_input->shape == std::vector<std::int64_t>{1, 16, 24, 1},
            "unexpected depth image input shape");
    require(policy_input->element_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT &&
            depth_input->element_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
            "depth ONNX inputs must be float32");
    require(depth_runner->output().name == "actions" &&
            depth_runner->output().shape == std::vector<std::int64_t>{1, 29},
            "unexpected depth ONNX output contract");

    auto external_observations = std::make_shared<isaaclab::ExternalObservationStore>();
    external_observations->set("depth", std::vector<float>(384, -1.0f));
    auto depth_robot = std::make_shared<MockArticulation>();
    isaaclab::ManagerBasedRLEnv depth_env(
        YAML::LoadFile(depth_deploy.string()), depth_robot, external_observations);
    depth_env.reset();
    const auto depth_observations = depth_env.observation_manager->compute();
    require(depth_observations.size() == 2 &&
            depth_observations.at("policy").size() == 480 &&
            depth_observations.at("depth").size() == 384,
            "depth deploy.yaml observation dimensions do not match the ONNX model");
    require(depth_env.action_manager->total_action_dim() == 29,
            "depth deploy.yaml action dimension does not match the ONNX model");
    depth_env.alg = std::move(depth_runner);
    depth_env.step();
    const auto depth_action = depth_env.action_manager->processed_actions();
    require(depth_action.size() == 29 &&
            std::all_of(depth_action.begin(), depth_action.end(),
                [](float value) { return std::isfinite(value); }),
            "depth policy inference did not produce 29 finite joint targets");

    std::cout << "ort_runner_test passed\n";
    return 0;
}
