from typing import Any

import numpy as np


def place_robot_2d(
    model_prefix: str,
    mj_data: Any,
    pose: tuple[float, float, float],
) -> None:
    """
    Place the robot identified by the given model prefix at the specified 2D location.
    """

    root_joint = mj_data.joint(model_prefix + 'root')
    root_joint.qpos[0] = pose[0]
    root_joint.qpos[1] = pose[1]
    root_joint.qpos[3:7] = [1, 0, 0, 0]
    root_joint.qvel = np.zeros(6)
    root_joint.qacc = np.zeros(6)

    root_body = mj_data.body(model_prefix + 'torso')
    root_body.xpos[0] = pose[0]
    root_body.xpos[1] = pose[1]
    root_body.xquat = [1, 0, 0, 0]


def place_robot_3d(
    model_prefix: str,
    mj_data: Any,
    pos: tuple[float, float, float],
    quat: tuple[float, float, float, float],
) -> None:
    """
    Place the robot identified by the given model prefix at the specified 3D location.
    """

    root_joint = mj_data.joint(model_prefix + 'root')
    root_joint.qpos[0:3] = pos
    root_joint.qpos[3:7] = quat
    root_joint.qvel = np.zeros(6)
    root_joint.qacc = np.zeros(6)

    root_body = mj_data.body(model_prefix + 'torso')
    root_body.xpos = pos
    root_body.xquat = quat
