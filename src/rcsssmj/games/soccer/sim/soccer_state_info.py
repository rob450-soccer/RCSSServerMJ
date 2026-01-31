from typing import Final

from rcsssmj.games.soccer.soccer_fields import SoccerField
from rcsssmj.games.soccer.soccer_rules import SoccerRules
from rcsssmj.sim.state_info import SimStateInformation


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
