#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer


HERE = Path(__file__).resolve().parent
XML = HERE /  "aquarm_origin_with_gripper.xml"
DEFAULT_GRIPPER_COMMAND = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Open an Aquarm MuJoCo XML in the viewer.")
    parser.add_argument(
        "xml",
        nargs="?",
        default=str(XML),
        help="MJCF XML path. Relative paths are resolved from this mujoco directory.",
    )
    return parser.parse_args()


def resolve_xml_path(xml: str) -> Path:
    path = Path(xml).expanduser()
    if not path.is_absolute():
        path = HERE / path
    return path.resolve()


def set_joint_position(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str, value: float) -> None:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        return
    qpos_addr = int(model.jnt_qposadr[joint_id])
    data.qpos[qpos_addr] = value


def apply_default_gripper_ctrl(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_gripper_joint1")
    if actuator_id < 0:
        return

    q = DEFAULT_GRIPPER_COMMAND / 22.0
    set_joint_position(model, data, "gripper_joint1", q)
    for joint_name, multiplier in {
        "gripper_joint2": 1.0,
        "gripper_joint5": 1.0,
        "gripper_joint6": -1.0,
        "gripper_joint7": 1.0,
        "gripper_joint8": -1.0,
        "gripper_joint11": -1.0,
        "gripper_joint12": 1.0,
    }.items():
        set_joint_position(model, data, joint_name, multiplier * q)

    data.ctrl[actuator_id] = DEFAULT_GRIPPER_COMMAND
    mujoco.mj_forward(model, data)


def main() -> None:
    args = parse_args()
    xml_path = resolve_xml_path(args.xml)
    if not xml_path.exists():
        raise FileNotFoundError(f"{xml_path} does not exist.")

    print(f"[XML] {xml_path}")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    apply_default_gripper_ctrl(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.geomgroup[3] = 0

        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
