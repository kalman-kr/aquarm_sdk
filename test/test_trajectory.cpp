#undef NDEBUG  // CMake forces Release; keep assert() active
#include "kalman_ik.hpp"

#include <cassert>
#include <cmath>

int main() {
  using kalman::trajectory::quinticBlend;
  using kalman::trajectory::quinticInterpolate;
  using kalman::trajectory::sampleQuintic;

  constexpr double kEpsilon = 1e-12;

  const auto start_blend = quinticBlend(0.0, 2.0);
  assert(std::abs(start_blend.s) < kEpsilon);
  assert(std::abs(start_blend.ds_dt) < kEpsilon);
  assert(std::abs(start_blend.d2s_dt2) < kEpsilon);

  const auto end_blend = quinticBlend(2.0, 2.0);
  assert(std::abs(end_blend.s - 1.0) < kEpsilon);
  assert(std::abs(end_blend.ds_dt) < kEpsilon);
  assert(std::abs(end_blend.d2s_dt2) < kEpsilon);

  kalman::math::VectorX start(6);
  start << 0.0, 0.2, -0.2, 0.0, 0.2, 0.0;

  kalman::math::VectorX goal(6);
  goal << 0.3, 0.1, -0.1, 0.2, 0.3, -0.2;

  const auto point = quinticInterpolate(start, goal, 2.0, 2.0);
  assert((point.position - goal).norm() < kEpsilon);
  assert(point.velocity.norm() < kEpsilon);
  assert(point.acceleration.norm() < kEpsilon);

  const auto trajectory = sampleQuintic(start, goal, 1.0, 0.3);
  assert(trajectory.size() == 5);
  assert(std::abs(trajectory.back().time - 1.0) < kEpsilon);

  return 0;
}
