// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "isaaclab/manager/observation_manager.h"
#include "isaaclab/manager/action_manager.h"
#include "isaaclab/assets/articulation/articulation.h"
#include "isaaclab/algorithms/algorithms.h"
#include "isaaclab/external_observation_store.h"
#include <iostream>
#include "isaaclab/utils/utils.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <unordered_set>

namespace isaaclab
{

class ObservationManager;
class ActionManager;

class ManagerBasedRLEnv
{
public:
    // Constructor
    ManagerBasedRLEnv(
        YAML::Node cfg,
        std::shared_ptr<Articulation> robot_,
        std::shared_ptr<ExternalObservationStore> external_observations_ =
            std::make_shared<ExternalObservationStore>())
    :cfg(cfg), robot(std::move(robot_)), external_observations(std::move(external_observations_))
    {
        if (!robot) {
            throw std::runtime_error("ManagerBasedRLEnv requires a robot articulation");
        }
        if (!external_observations) {
            throw std::runtime_error("ManagerBasedRLEnv requires an external observation store");
        }

        // Parse configuration
        this->step_dt = cfg["step_dt"].as<float>();
        if (!std::isfinite(step_dt) || step_dt <= 0.0f) {
            throw std::runtime_error("deploy.yaml step_dt must be finite and positive");
        }
        robot->data.joint_ids_map = cfg["joint_ids_map"].as<std::vector<float>>();
        if (robot->data.joint_ids_map.empty()) {
            throw std::runtime_error("deploy.yaml joint_ids_map must not be empty");
        }
        std::unordered_set<int> unique_joint_ids;
        for (float id : robot->data.joint_ids_map) {
            if (!std::isfinite(id) || id < 0.0f || std::floor(id) != id) {
                throw std::runtime_error("deploy.yaml joint_ids_map must contain non-negative integers");
            }
            if (!unique_joint_ids.insert(static_cast<int>(id)).second) {
                throw std::runtime_error("deploy.yaml joint_ids_map contains duplicate joint ids");
            }
        }
        robot->data.joint_pos.resize(robot->data.joint_ids_map.size());
        robot->data.joint_vel.resize(robot->data.joint_ids_map.size());

        { // default joint positions
            auto default_joint_pos = cfg["default_joint_pos"].as<std::vector<float>>();
            validate_joint_vector(default_joint_pos, "default_joint_pos");
            robot->data.default_joint_pos = Eigen::VectorXf::Map(default_joint_pos.data(), default_joint_pos.size());
        }
        { // joint stiffness and damping
            robot->data.joint_stiffness = cfg["stiffness"].as<std::vector<float>>();
            robot->data.joint_damping = cfg["damping"].as<std::vector<float>>();
            validate_joint_vector(robot->data.joint_stiffness, "stiffness");
            validate_joint_vector(robot->data.joint_damping, "damping");
        }

        robot->update();

        // load managers
        action_manager = std::make_unique<ActionManager>(cfg["actions"], this);
        observation_manager = std::make_unique<ObservationManager>(cfg["observations"], this);
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(step_mutex);
        global_phase = 0;
        episode_length.store(0);
        robot->update();
        action_manager->reset();
        observation_manager->reset();
    }

    void step()
    {
        if (!alg) {
            throw std::runtime_error("Policy algorithm is not configured");
        }
        ObservationMap obs;
        {
            // Keep robot/observation state consistent, but never hold this lock
            // during ONNX inference: the 1 kHz safety checks also read robot state.
            std::lock_guard<std::mutex> lock(step_mutex);
            episode_length.fetch_add(1);
            robot->update();
            obs = observation_manager->compute();
        }
        auto action = alg->act(obs);
        action_manager->process_action(action);
    }

    float step_dt;
    
    YAML::Node cfg;

    std::unique_ptr<ObservationManager> observation_manager;
    std::unique_ptr<ActionManager> action_manager;
    std::shared_ptr<Articulation> robot;
    std::unique_ptr<Algorithms> alg;
    std::shared_ptr<ExternalObservationStore> external_observations;
    std::atomic<long> episode_length{0};
    float global_phase = 0.0f;
    mutable std::mutex step_mutex;

private:
    void validate_joint_vector(const std::vector<float>& values, const std::string& name) const
    {
        if (values.size() != robot->data.joint_ids_map.size()) {
            throw std::runtime_error(
                "deploy.yaml " + name + " length does not match joint_ids_map");
        }
        if (!std::all_of(values.begin(), values.end(),
                         [](float value) { return std::isfinite(value); })) {
            throw std::runtime_error("deploy.yaml " + name + " contains NaN or Inf");
        }
    }
};

};
