from abc import ABC, abstractmethod
from typing import Any, Final

from rcsssmj.game.field import SoccerField
from rcsssmj.game.rules import SoccerRules


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


class SoccerEnvironmentInformation(SimStateInformation):
    """Soccer environment state information."""

    def __init__(self, field: SoccerField, rules: SoccerRules):
        """Construct a new soccer environment state information.

        Parameter
        ---------
        field: SoccerField
            The soccer field specification.

        rules: SoccerRules
            The active soccer rule book.
        """

        super().__init__('soccer-env')

        self.field: Final[SoccerField] = field
        """The soccer field specification."""

        self.rules: Final[SoccerRules] = rules
        """The active soccer rule book."""

    def to_sexp(self) -> str:
        # TODO: Add s-expression encoding for environment information
        return '()'


class SoccerGameInformation(SimStateInformation):
    """Soccer game state information."""

    def __init__(
        self,
        left_team: str,
        right_team: str,
        left_score: int,
        right_score: int,
        play_time: float,
        play_mode: str,
    ):
        """Construct a new soccer game state information.

        Parameter
        ---------
        left_team: str
            The name of the left team.

        right_team: str
            The name of the right team.

        left_score: int
            The score of the left team.

        right_score: int
            The score of the right team.

        play_time: float
            The current play time.

        play_mode: str
            The current play mode.
        """

        super().__init__('soccer-game')

        self.left_team: Final[str] = left_team
        """The name of the left team."""

        self.right_team: Final[str] = right_team
        """The name of the right team."""

        self.left_score: Final[int] = left_score
        """The score of the left team."""

        self.right_score: Final[int] = right_score
        """The score of the right team."""

        self.play_time: Final[float] = play_time
        """The current play time."""

        self.play_mode: Final[str] = play_mode
        """The current play mode."""

    def to_sexp(self) -> str:
        # TODO: Add s-expression encoding for game state information
        return '()'
