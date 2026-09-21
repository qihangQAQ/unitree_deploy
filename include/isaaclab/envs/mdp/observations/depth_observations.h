#pragma once

#include "isaaclab/envs/manager_based_rl_env.h"

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(depths)
{
    (void)params;
    return env->external_observations->get("depth");
}

} // namespace mdp
} // namespace isaaclab
