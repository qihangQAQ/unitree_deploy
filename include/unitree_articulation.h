// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "isaaclab/assets/articulation/articulation.h"

#include <cmath>
#include <stdexcept>

namespace unitree
{

template <typename LowStatePtr>
class BaseArticulation : public isaaclab::Articulation
{
public:
    BaseArticulation(LowStatePtr lowstate_)
    : lowstate(lowstate_)
    {
        data.joystick = &lowstate->joystick;
    }

    void update() override
    {
        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        // base_angular_velocity
        for(int i(0); i<3; i++) {
            data.root_ang_vel_b[i] = lowstate->msg_.imu_state().gyroscope()[i];
        }
        // project_gravity_body
        data.root_quat_w = Eigen::Quaternionf(
            lowstate->msg_.imu_state().quaternion()[0],
            lowstate->msg_.imu_state().quaternion()[1],
            lowstate->msg_.imu_state().quaternion()[2],
            lowstate->msg_.imu_state().quaternion()[3]
        );
        data.projected_gravity_b = data.root_quat_w.conjugate() * data.GRAVITY_VEC_W;
        // joint positions and velocities
        for(int i(0); i< data.joint_ids_map.size(); i++) {
            const float configured_id = data.joint_ids_map[i];
            if (!std::isfinite(configured_id) || configured_id < 0.0f ||
                std::floor(configured_id) != configured_id ||
                static_cast<std::size_t>(configured_id) >= lowstate->msg_.motor_state().size()) {
                throw std::runtime_error("Joint id is outside the Unitree LowState motor array");
            }
            const auto sdk_id = static_cast<std::size_t>(configured_id);
            data.joint_pos[i] = lowstate->msg_.motor_state()[sdk_id].q();
            data.joint_vel[i] = lowstate->msg_.motor_state()[sdk_id].dq();
        }
    }

    LowStatePtr lowstate;
};

}
