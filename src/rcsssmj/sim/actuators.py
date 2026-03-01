from typing import TYPE_CHECKING, Final

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray


class CustomActuator:
    """Base class for custom actuator implementations."""

    def __init__(self, name: str, site: str) -> None:
        """Construct a new custom actuator.

        Parameter
        ---------
        name: str
            The name of the actuator.

        site: str
            The mujoco site name associated with this actuator.
        """

        self.name: Final[str] = name
        """The name of the actuator."""

        self.site: Final[str] = site
        """The mujoco site name associated with this custom actuator."""


class Speaker(CustomActuator):
    """Speaker actuator implementation."""

    def __init__(self, name: str, site: str) -> None:
        """Construct a new speaker actuator.

        Parameter
        ---------
        name: str
            The name of the actuator.

        site: str
            The mujoco site name associated with this actuator.
        """

        super().__init__(name, site)

        self.ctrl: bytes | bytearray | None = None
        """The message to broadcast."""

        self.gainprm: NDArray[np.float64] = np.array([1], dtype=np.float64)
        """The speaker gain parameter."""
