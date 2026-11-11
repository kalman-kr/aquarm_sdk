#pragma once

#include <vector>

#include "kalman_ik/math/types.hpp"

namespace kalman::trajectory {

struct QuinticBlend {
  double s;
  double ds_dt;
  double d2s_dt2;
};

struct TrajectoryPoint {
  double time;
  kalman::math::VectorX position;
  kalman::math::VectorX velocity;
  kalman::math::VectorX acceleration;
};

KALMAN_IK_API QuinticBlend quinticBlend(double time, double duration);

KALMAN_IK_API TrajectoryPoint quinticInterpolate(
    const kalman::math::VectorX& start,
    const kalman::math::VectorX& goal,
    double time,
    double duration);

KALMAN_IK_API std::vector<TrajectoryPoint> sampleQuintic(
    const kalman::math::VectorX& start,
    const kalman::math::VectorX& goal,
    double duration,
    double dt);

}  // namespace kalman::trajectory
