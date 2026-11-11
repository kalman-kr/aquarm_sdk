#include "kalman_ik.hpp"

#include <cmath>
#include <iostream>

using namespace kalman;
/*
 * ./build/test_ik [urdf_path]    (run from the SDK root)
 */
int main(int argc, char** argv) {
  const char* urdf = argc > 1 ? argv[1] : "resources/urdf/aquarm_origin_with_gripper.urdf";
  const model::RobotModel robot = model::loadUrdf(urdf);

  // Target tool pose: xyz [m], rpy [rad]
  const math::Vector3 xyz(0.10, -0.30, 0.10);
  const math::Vector3 rpy(0, M_PI, 0.0);
  const math::SE3 target = math::se3FromXyzRpy(xyz, rpy);

  const math::VectorX q0 = math::VectorX::Zero(robot.jointCount());
  const kinematics::DlsIkResult ik = kinematics::solveDlsIk(robot, target, q0);

  const math::Vector6 err =
      math::se3Log(kinematics::forwardKinematics(robot, ik.joint_angles).inverse() * target);

  std::cout << "kalman_ik " << KALMAN_IK_VERSION << "\n"
            << "converged: " << ik.converged << " (" << ik.iterations << " iters)\n"
            << "q:         " << ik.joint_angles.transpose() << "\n"
            << "pos err:   " << err.head<3>().norm() << " m\n"
            << "rot err:   " << err.tail<3>().norm() << " rad\n";
  return ik.converged ? 0 : 1;
}
