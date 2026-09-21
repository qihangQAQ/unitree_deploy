#include "isaaclab/algorithms/algorithms.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/observations/observations.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace
{
void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
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
    auto runner = std::make_unique<isaaclab::OrtRunner>(TEST_POLICY_PATH);
    require(runner->inputs().size() == 1, "unexpected ONNX input count");
    const auto& input = runner->inputs().front();
    require(input.name == "obs", "unexpected ONNX input name");
    require(input.element_count == 480, "unexpected ONNX input size");
    require(runner->output().element_count == 29, "unexpected ONNX output size");

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
    isaaclab::ManagerBasedRLEnv env(YAML::LoadFile(TEST_DEPLOY_PATH), robot);
    env.alg = std::move(runner);
    env.reset();
    const auto configured_observations = env.observation_manager->compute();
    const auto configured_input = configured_observations.find("obs");
    require(configured_input != configured_observations.end(),
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

    std::cout << "ort_runner_test passed\n";
    return 0;
}
