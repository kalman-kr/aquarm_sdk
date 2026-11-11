
#pragma once

#include "kalman_ik/math/types.hpp"

namespace kalman::math {

struct KALMAN_IK_API SE3 {
  Matrix3 rotation = Matrix3::Identity();
  Vector3 translation = Vector3::Zero();

  SE3 inverse() const;

  SE3 operator*(const SE3& rhs) const;

  Vector3 act(const Vector3& point) const;
};

// twist = [linear; angular]
KALMAN_IK_API SE3 se3Exp(const Vector6& twist);

KALMAN_IK_API Vector6 se3Log(const SE3& transform);

KALMAN_IK_API Matrix6 se3Jlog(const SE3& transform);

// xyz [m], rpy [rad], R = Rz(yaw) * Ry(pitch) * Rx(roll) (URDF convention)
KALMAN_IK_API SE3 se3FromXyzRpy(const Vector3& xyz, const Vector3& rpy);
}  // namespace kalman::math
