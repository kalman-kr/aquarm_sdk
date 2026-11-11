#pragma once

#include "kalman_ik/math/se3.hpp"
#include "kalman_ik/math/types.hpp"
#include "kalman_ik/model/robot_model.hpp"

namespace kalman::kinematics {

KALMAN_IK_API kalman::math::SE3 forwardKinematics(
    const kalman::model::RobotModel& model,
    const kalman::math::VectorX& joint_angles);

}  // namespace kalman::kinematics
