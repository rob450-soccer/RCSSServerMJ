from collections.abc import Mapping
from typing import TYPE_CHECKING

import numpy as np

from rcsssmj.sim.actuators import Speaker
from rcsssmj.sim.sensors import Microphone

if TYPE_CHECKING:
    from collections.abc import Sequence

    from numpy.typing import NDArray


class AudioData:
    """Audio engine state data."""

    def __init__(
        self,
        sensors: Mapping[str, Microphone],
        actuators: Mapping[str, Speaker],
    ) -> None:
        """Construct a new audio state data.

        Parameter
        ---------
        sensors: Mapping[str, Microphone]
            The list of known microphone sensors.

        actuators: Mapping[str, Speaker]
            The list of known speaker actuators.
        """

        self.sensors: Mapping[str, Microphone] = sensors
        """The list of known microphone sensors."""

        self.actuators: Mapping[str, Speaker] = actuators
        """The list of known speaker actuators."""

        self.messages: Sequence[bytes | bytearray] = []
        """The list of broadcasted messages."""

        self.sources: Sequence[str] = []
        """The audio sources (actuator name)."""

        self.volumes: NDArray[np.float64] = np.zeros(0, dtype=np.float64)
        """The volumes at which audio messages are transmitted."""

        self.origins: NDArray[np.float64] = np.zeros((0, 3), dtype=np.float64)
        """The audio signal origins."""
