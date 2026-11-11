#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include "kalman_ik/math/se3.hpp"

namespace kalman::model {

struct RevoluteJoint {
  kalman::math::SE3 parent_to_joint;
  kalman::math::Vector3 axis;

  double lower_limit = -std::numeric_limits<double>::infinity();
  double upper_limit = std::numeric_limits<double>::infinity();
};

struct RobotModel {
  std::vector<RevoluteJoint> joints;
  kalman::math::SE3 joint_to_tool;

  std::size_t jointCount() const {
    return joints.size();
  }
};

}  // namespace kalman::model
