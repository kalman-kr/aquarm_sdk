#include "kalman_ik.hpp"

#include <kcan_sdk_user.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <thread>
#include <vector>

constexpr double kMaxJointVelocity = 0.5;
constexpr double kMinDuration = 1.0;

static volatile std::sig_atomic_t running = 1;

int main() {
  std::signal(SIGINT, [](int) { running = 0; });

  //Target pose
  const auto robot = kalman::model::loadUrdf(AQUARM_URDF);
  const size_t joint_count = robot.jointCount();
  const auto target = kalman::math::se3FromXyzRpy(
      kalman::math::Vector3(0.10, -0.30, 0.10),
      kalman::math::Vector3(0, 1.57, 0.0));

  //Connect KCAN
  kcan_user::KcanUserSDK sdk(static_cast<int>(joint_count));
  if (!sdk.connect("192.168.1.9", 20000, 20001)) return 1;
  const auto fail = [&] {
    sdk.send_stop();
    sdk.disconnect();
    return 1;
  };
  if (!sdk.send_operational(0x00, 0.005f)) return fail();

  //Get Joint feedback
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const auto states = sdk.get_joint_states();
  if (states.size() != joint_count) return fail();
  kalman::math::VectorX start(joint_count);
  for (size_t i = 0; i < joint_count; ++i) start[i] = states[i].position;

  //Solve IK from the current pose (nearest solution, no multi-turn unwinding)
  const auto ik = kalman::kinematics::solveDlsIk(robot, target, start);
  if (!ik.converged) return fail();
  const kalman::math::VectorX& goal = ik.joint_angles;

  //Calculate Trajectory
  const double duration = std::max(
      kMinDuration, 1.875 * (goal - start).cwiseAbs().maxCoeff() / kMaxJointVelocity);

  std::vector<float> command(joint_count);
  const auto t0 = std::chrono::steady_clock::now();
  while (running) {
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const auto q = kalman::trajectory::quinticInterpolate(start, goal, t, duration).position;
    for (size_t i = 0; i < joint_count; ++i) command[i] = static_cast<float>(q[i]);
    //Send KCAN command
    sdk.send_joint_command(command);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  sdk.send_stop();
  sdk.disconnect();
  return 0;
}
