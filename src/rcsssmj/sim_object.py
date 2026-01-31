from typing import Any, Final

import numpy as np
import numpy.typing as npt


class SimObject:
    """Abstraction of an object in a simulation."""

    def __init__(self, name: str) -> None:
        """Construct a new simulation object.

        Parameter
        ---------
        name: str
            The unique name of the object.
        """

        self.name: Final[str] = name
        """The unique name of the object."""

        self._xpos: npt.NDArray[np.float64] = np.zeros(3)
        """The root body position array view."""

        self._xquat: npt.NDArray[np.float64] = np.array([1, 0, 0, 0])
        """The root body orientation quaternion array view."""

        self._xmat: npt.NDArray[np.float64] = np.array([1, 0, 0, 0, 1, 0, 0, 0, 1])
        """The root body orientation matrix array view."""

        self._qpos: npt.NDArray[np.float64] = np.zeros(7)
        """The root body joint position and orientation quaternion array view."""

        self._qvel: npt.NDArray[np.float64] = np.zeros(6)
        """The root body joint (linear and angular) velocity array view."""

        self._qacc: npt.NDArray[np.float64] = np.zeros(6)
        """The root body joint (linear and angular) acceleration array view."""

        self._prev_xpos: npt.NDArray[np.float64] = np.zeros(3)
        """The previous position of the object."""

    @property
    def xpos(self) -> npt.NDArray[np.float64]:
        """The root body position array view."""

        return self._xpos

    @property
    def xquat(self) -> npt.NDArray[np.float64]:
        """The root body orientation quaternion array view."""

        return self._xquat

    @property
    def xmat(self) -> npt.NDArray[np.float64]:
        """The root body orientation matrix array view."""

        return self._xmat

    @property
    def prev_xpos(self) -> npt.NDArray[np.float64]:
        """The previous position of the object."""

        return self._prev_xpos

    @property
    def root_body_name(self) -> str:
        """The name of the root body of the object."""

        return self.name

    @property
    def root_joint_name(self) -> str:
        """The name of the root joint of the object."""

        return self.name + '-root'

    def bind(self, mj_model: Any, mj_data: Any) -> None:
        """Bind the object to the given model and data."""

        root_body = mj_data.body(self.root_body_name)
        self._xpos = root_body.xpos
        self._xquat = root_body.xquat
        self._xmat = root_body.xmat

        root_joint = mj_data.joint(self.root_joint_name)
        self._qpos = root_joint.qpos
        self._qvel = root_joint.qvel
        self._qacc = root_joint.qacc

    def pre_step(self, mj_model: Any, mj_data: Any) -> None:
        """Method triggered before a simulation step."""

        self._prev_xpos = self._xpos.astype(np.float64)

    def post_step(self, mj_model: Any, mj_data: Any) -> None:
        """Method triggered after a simulation step."""

    def place_at(
        self,
        pos: tuple[float, float, float],
        quat: tuple[float, float, float, float] | None = None,
    ) -> None:
        """Place the object at the specified location.

        Note: This method will also reset the object velocities and accelerations to zero.

        Parameter
        ---------
        pos: tuple[float, float, float]
            The target position.

        quat: tuple[float, float, float, float] | None, default=None
            The target orientation. If ``None``, the "identity" quaternion is used.
        """

        # set object state
        self._qpos[0:3] = pos
        self._qpos[3:7] = (1, 0, 0, 0) if quat is None else quat

        self._qvel[0:6] = np.zeros(6)
        self._qacc[0:6] = np.zeros(6)

        # set derived state information
        self._xpos[0:3] = self._qpos[0:3].astype(np.float64)
        self._xquat[0:4] = self._qpos[3:7].astype(np.float64)
        self._prev_xpos = self._xpos.astype(np.float64)
