
#pragma once

#include "kalman_ik/math/types.hpp"
#include "kalman_ik/model/robot_model.hpp"

namespace kalman::kinematics {

// [v_tool; w_tool] = J q_dot, world frame
KALMAN_IK_API kalman::math::MatrixX toolJacobian(
    const kalman::model::RobotModel& model,
    const kalman::math::VectorX& joint_angles);

// [v_tool; w_tool] = J q_dot, tool local frame
KALMAN_IK_API kalman::math::MatrixX toolJacobianLocal(
    const kalman::model::RobotModel& model,
    const kalman::math::VectorX& joint_angles);
}  // namespace kalman::kinematics
