import kalman_ik_py as ik


robot = ik.model.load_urdf("resources/urdf/aquarm_origin_with_gripper.urdf")
target = ik.math.se3_from_xyz_rpy([0.10, -0.30, 0.10], [0.0, 3.141592653589793, 0.0])
result = ik.kinematics.solve_dls_ik(robot, target, [0.0] * robot.joint_count())
assert result.converged

pose = ik.kinematics.forward_kinematics(robot, result.joint_angles)
error = ik.math.se3_log(pose.inverse() * target)
assert sum(value * value for value in error[:3]) ** 0.5 < 1e-6
assert sum(value * value for value in error[3:]) ** 0.5 < 1e-6
