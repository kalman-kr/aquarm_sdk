#pragma once

#include <cstddef>

#include "kalman_ik/math/se3.hpp"
#include "kalman_ik/math/types.hpp"
#include "kalman_ik/model/robot_model.hpp"

namespace kalman::kinematics {

struct DlsIkOptions {
  double tolerance = 1e-8;
  std::size_t max_iterations = 400;

  double time_step = 1.0;
  double damping_base = 1e-8;
  double lm_weight = 0.1;

  double orientation_weight = 0.25;
  double step_limit = 0.2;
  bool position_only = false;
};

struct DlsIkResult {
  kalman::math::VectorX joint_angles;
  bool converged = false;
  std::size_t iterations = 0;
};

KALMAN_IK_API DlsIkResult solveDlsIk(
    const kalman::model::RobotModel& model,
    const kalman::math::SE3& target_pose,
    const kalman::math::VectorX& initial_joint_angles,
    const DlsIkOptions& options = {});

}  // namespace kalman::kinematics
