import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))
from kcan_sdk_user_py import KcanUserSDK

#Target position (rad)
REF = 0.0
NODE_ID = 6
JOINT_COUNT = 6

#Connect KCAN
sdk = KcanUserSDK(JOINT_COUNT)
if not sdk.connect("192.168.1.9", 20000, 20001):
    raise SystemExit("connect failed")

try:
    #User API sets position mode and starts sync
    if not sdk.send_operational(NODE_ID, 0.005):
        raise SystemExit("send_operational failed")

    #Hold the other joints at their feedback positions
    time.sleep(0.1)
    command = [s.position for s in sdk.get_joint_states()]
    command[NODE_ID - 1] = REF

    while True:
        sdk.send_joint_command(command)
        s = sdk.get_joint_states()[NODE_ID - 1]
        print(f"\rpos {s.position:+.3f}  vel {s.velocity:+.3f}  torque {s.torque:+.3f}", end="", flush=True)
        time.sleep(0.005)
except KeyboardInterrupt:
    pass
finally:
    sdk.send_stop()
    sdk.disconnect()
