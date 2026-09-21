// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <unordered_set>
#include "isaaclab/manager/manager_term_cfg.h"
#include <iostream>

namespace isaaclab
{

using ObsMap = std::map<std::string, ObsFunc>;

inline ObsMap& observations_map() {
    static ObsMap instance;
    return instance;
}

#define REGISTER_OBSERVATION(name) \
    inline std::vector<float> name(ManagerBasedRLEnv* env, YAML::Node params); \
    inline struct name##_registrar { \
        name##_registrar() { observations_map()[#name] = name; } \
    } name##_registrar_instance; \
    inline std::vector<float> name(ManagerBasedRLEnv* env, YAML::Node params)


class ObservationManager
{
public:
    ObservationManager(YAML::Node cfg, ManagerBasedRLEnv* env)
    :cfg(cfg), env(env)
    {
        _prapare_terms();
    }

    void reset()
    {
        for(auto & group : group_obs_term_cfgs_)
        {
            for(auto & term : group.second)
            {
                term.reset(term.func(this->env, term.params));
            }
        }
    }

    std::unordered_map<std::string, std::vector<float>> compute()
    {
        std::unordered_map<std::string, std::vector<float>> obs_map;
        for(const auto & group : group_obs_term_cfgs_)
        {
            const auto group_obs = compute_group(group.first);
            obs_map[group.first] = group_obs;
        }
        return obs_map;
    }



    const std::vector<float> compute_group(const std::string& group_name)
    {
        std::vector<float> obs;
        auto& group_terms = group_obs_term_cfgs_.at(group_name);

        for(auto & term : group_terms) {
            term.add(term.func(this->env, term.params));
        }

        if(group_use_gym_history_.at(group_name))
        {
            for(int h = 0; h < group_terms[0].history_length; ++h)
            {
                for(auto & term : group_terms)
                {
                    auto term_obs_scaled = term.get(h);
                    obs.insert(obs.end(), term_obs_scaled.begin(), term_obs_scaled.end());
                }
            }            
        }
        else
        {
            for(const auto & term : group_terms)
            {
                auto obs_ = term.get();
                obs.insert(obs.end(), obs_.begin(), obs_.end());
            }
        }
        return obs;
    }

protected:
    void _prapare_terms()
    {
        if (!cfg || !cfg.IsMap() || cfg.size() == 0) {
            throw std::runtime_error("Observations configuration is empty");
        }

        // Legacy single-input deploy.yaml stores observation terms directly.
        const bool only_one_input = this->cfg.begin()->second["params"].IsDefined();
        if(only_one_input) {
            bool use_gym_history = false;
            group_obs_term_cfgs_["obs"] = _prepare_group_terms(this->cfg, use_gym_history);
            group_use_gym_history_["obs"] = use_gym_history;
        } else {
            for(auto group = this->cfg.begin(); group != this->cfg.end(); ++group)
            {
                auto group_name = group->first.as<std::string>();
                bool use_gym_history = false;
                group_obs_term_cfgs_[group_name] = _prepare_group_terms(group->second, use_gym_history);
                group_use_gym_history_[group_name] = use_gym_history;
            }
        }
    }

    std::vector<ObservationTermCfg> _prepare_group_terms(
        const YAML::Node & group_cfg, bool& use_gym_history)
    {
        std::vector<ObservationTermCfg> terms;
        bool scale_first = false; // isaaclab default: clip first
        for(auto it = group_cfg.begin(); it != group_cfg.end(); ++it)
        {
            std::string key = it->first.as<std::string>();
            if(it->first.as<std::string>() == "scale_first") {
                scale_first = it->second.as<bool>();
                continue;
            }
            if(it->first.as<std::string>() == "use_gym_history") { // set only once
                use_gym_history = it->second.as<bool>();
                continue;
            }

            /*** observation terms ***/
            const auto term_yaml_cfg = it->second;
            ObservationTermCfg term_cfg;
            term_cfg.params = term_yaml_cfg["params"];
            term_cfg.scale_first = scale_first;
            term_cfg.history_length = term_yaml_cfg["history_length"].as<int>(1);
            if (term_cfg.history_length <= 0) {
                throw std::runtime_error("Observation history_length must be positive");
            }

            auto term_name = it->first.as<std::string>();
            if(observations_map()[term_name] == nullptr) {
                throw std::runtime_error("Observation term '" + term_name + "' is not registered.");
            }

            if(!term_yaml_cfg["scale"].IsNull()) {
                term_cfg.scale = term_yaml_cfg["scale"].as<std::vector<float>>();
            }
            if(!term_yaml_cfg["clip"].IsNull()) {
                term_cfg.clip = term_yaml_cfg["clip"].as<std::vector<float>>();
            }
            term_cfg.func = observations_map()[term_name];   


            auto obs = term_cfg.func(this->env, term_cfg.params);
            if (obs.empty()) {
                throw std::runtime_error("Observation term '" + term_name + "' returned no values");
            }
            term_cfg.reset(obs);

            terms.push_back(term_cfg);
        }
        if (terms.empty()) {
            throw std::runtime_error("Observation group has no terms");
        }
        if (use_gym_history) {
            const int history_length = terms.front().history_length;
            for (const auto& term : terms) {
                if (term.history_length != history_length) {
                    throw std::runtime_error(
                        "All terms in a gym-history observation group must have equal history_length");
                }
            }
        }
        return terms;
    }

    const YAML::Node cfg;
    ManagerBasedRLEnv* env;

private:
    std::unordered_map<std::string, std::vector<ObservationTermCfg>> group_obs_term_cfgs_;
    std::unordered_map<std::string, bool> group_use_gym_history_;
};

};
