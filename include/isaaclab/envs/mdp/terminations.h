#pragma once

#include "isaaclab/envs/manager_based_rl_env.h"

namespace isaaclab
{
namespace mdp
{

inline bool bad_orientation(ManagerBasedRLEnv* env, float limit_angle = 1.0)
{
    std::lock_guard<std::mutex> lock(env->step_mutex);
    auto & asset = env->robot;
    auto & data = asset->data.projected_gravity_b;
    const float gravity_z = std::clamp(-data[2], -1.0f, 1.0f);
    return std::fabs(std::acos(gravity_z)) > limit_angle;
}

}
} 
