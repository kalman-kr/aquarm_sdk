#pragma once

#include <string>

#include "kalman_ik/model/robot_model.hpp"

namespace kalman::model {

// URDF (revolute / continuous / fixed) -> RobotModel, chain root -> tip_link
KALMAN_IK_API RobotModel loadUrdf(const std::string& path, const std::string& tip_link = "tool");
KALMAN_IK_API RobotModel loadUrdfFromString(const std::string& xml, const std::string& tip_link = "tool");

}  // namespace kalman::model
