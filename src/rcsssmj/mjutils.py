from math import cos, sin
from typing import Any

import mujoco
import numpy as np


def quat_from_axis_angle(axis: tuple[float, float, float], angle: float) -> tuple[float, float, float, float]:
    """Construct a quaternion from the given axis and angle.

    Parameter
    ---------
    axis: tuple[float, float, float]
        The normalized rotation axis (unit vector).

    angle: float
        The rotation angle (in radian).

    Returns
    -------
    tuple[float, float, float, float]
        A quaternion representing the given axis angle [qw, qx, qy, qz].
    """

    s = sin(angle / 2)
    return cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s


def place_robot_3d(
    model_prefix: str,
    mj_model: Any,
    mj_data: Any,
    pos: tuple[float, float, float],
    quat: tuple[float, float, float, float],
) -> None:
    """Place the robot identified by the given model prefix at the specified 3D location."""

    root_joint = mj_data.joint(model_prefix + 'root')
    root_joint.qpos[0:3] = pos
    root_joint.qpos[3:7] = quat
    root_joint.qvel = np.zeros(6)
    root_joint.qacc = np.zeros(6)

    zero_all_joints(model_prefix, mj_model, mj_data)


def zero_all_joints(
    model_prefix: str,
    mj_model: Any,
    mj_data: Any,
) -> None:
    """Set all joint positions, velocities and accelerations of the robot identified by the given model prefix to zero."""

    all_joint_names = [mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_JOINT, jnt_id) for jnt_id in range(mj_model.njnt)]
    for joint_name in all_joint_names:
        if joint_name.startswith(model_prefix) and joint_name != model_prefix + 'root':
            joint = mj_model.joint(joint_name)
            qpos_adr = joint.qposadr[0]
            qvel_adr = joint.dofadr[0]
            mj_data.qpos[qpos_adr] = 0.0
            mj_data.qvel[qvel_adr] = 0.0
            mj_data.qacc[qvel_adr] = 0.0
