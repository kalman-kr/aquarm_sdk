import sys
import time
from pathlib import Path

SDK_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SDK_ROOT / "python"))
import kalman_ik_py as ik
from kcan_sdk_user_py import KcanUserSDK

MAX_VEL = 0.5  # rad/s

#Target pose
XYZ = [0.10, -0.30, 0.10]  # m
RPY = [0.0, 1.57, 0.0]     # rad

robot = ik.model.load_urdf(str(SDK_ROOT / "resources/urdf/aquarm_origin_with_gripper.urdf"))
JOINT_COUNT = robot.joint_count()

#Connect KCAN
sdk = KcanUserSDK(JOINT_COUNT)
if not sdk.connect("192.168.1.9", 20000, 20001):
    raise SystemExit("connect failed")

try:
    #Set position mode and start sync
    if not sdk.send_operational(0, 0.005):
        raise SystemExit("send_operational failed")

    #Get Joint feedback
    time.sleep(0.1)
    start = [s.position for s in sdk.get_joint_states()]
    if len(start) != JOINT_COUNT:
        raise SystemExit(f"expected {JOINT_COUNT} joint states, got {len(start)}")

    #Solve IK from the current pose (nearest solution, no multi-turn unwinding)
    result = ik.kinematics.solve_dls_ik(robot, ik.math.se3_from_xyz_rpy(XYZ, RPY), start)
    if not result.converged:
        raise SystemExit("IK failed")
    goal = result.joint_angles

    #Calculate Trajectory
    duration = max(1.0, 1.875 * max(abs(g - s) for g, s in zip(goal, start)) / MAX_VEL)

    #Send KCAN command
    t0 = time.monotonic()
    while True:
        q = ik.trajectory.quintic_interpolate(start, goal, time.monotonic() - t0, duration).position
        sdk.send_joint_command(q)
        time.sleep(0.005)
except KeyboardInterrupt:
    pass
finally:
    sdk.send_stop()
    sdk.disconnect()
