#pragma once

#include "kalman_ik/math/types.hpp"

namespace kalman::math {

KALMAN_IK_API Matrix3 skew(const Vector3& vector);

KALMAN_IK_API Matrix3 so3Exp(const Vector3& rotation_vector);

KALMAN_IK_API Vector3 so3Log(const Matrix3& rotation);

}  // namespace kalman::math
