from abc import ABC, abstractmethod
from typing import Any, Final


class SimStateInformation(ABC):
    """Base implementation for simulation state information."""

    def __init__(self, name: str):
        """Construct a new simulation state information.

        Parameter
        ---------
        name: str
            The state information name / id.
        """

        self.name: str = name
        """The state information name / id."""

    @abstractmethod
    def to_sexp(self) -> str:
        """Return an symbolic expression representing this state information."""


class SceneGraph(SimStateInformation):
    """Scene graph state information."""

    def __init__(self, mj_model: Any, mj_data: Any):
        """Construct a new scene graph state information."""

        super().__init__('scene-graph')

        self.mj_model: Final[Any] = mj_model
        """The current mujoco simulation model."""

        self.mj_data: Final[Any] = mj_data
        """The current mujoco simulation data array."""

    def to_sexp(self) -> str:
        # TODO: Add s-expression encoding for scene graph information
        return ''
