#pragma once

#include <Eigen/Core>

#include "kalman_ik/config.hpp"

// libkalman_ik.so is built with 16-byte Eigen alignment (default x86-64 / aarch64 flags).
// -mavx / -march=native changes Eigen's heap and stack alignment, and Eigen objects
// crossing the .so then crash. EIGEN_MAX_ALIGN_BYTES=16 does not fix heap alignment in Eigen 3.4.0.
#if EIGEN_DEFAULT_ALIGN_BYTES != 16 || EIGEN_MAX_STATIC_ALIGN_BYTES != 16
#error "kalman_ik: build without -mavx / -march=native (Eigen alignment must match the prebuilt lib)"
#endif

namespace kalman::math {

using Scalar = double;
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
// 6D (Pinocchio): twist = [linear; angular], wrench = [force; moment]
using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;
using VectorX = Eigen::VectorXd;
using MatrixX = Eigen::MatrixXd;

}  // namespace kalman::math
