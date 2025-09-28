from typing import Final

from rcsssmj.agent.perception import Perception


class GameStatePerception(Perception):
    """Game state perception."""

    def __init__(
        self,
        play_time: float,
        play_mode: str,
        team_left: str | None,
        team_right: str | None,
        score_left: int,
        score_right: int,
    ) -> None:
        """Construct a new game state perception.

        Parameter
        ---------
        play_time: float
            The current play time.

        play_mode: str
            the current play mode.

        team_left: str
            The name of the left team.

        team_right: str
            The name of the right team.

        score_left: int
            The score of the left team.

        score_right: int
            The score of the right team.
        """

        super().__init__('GS')

        self.play_time: Final[float] = play_time
        """The current play time."""

        self.play_mode: Final[str] = play_mode
        """The current play mode."""

        self.team_left: Final[str | None] = team_left
        """The name of the left team or None, if no left team has connected yet."""

        self.team_right: Final[str | None] = team_right
        """The name of the right team or None, if no right team has connected yet."""

        self.score_left: Final[int] = score_left
        """The score of the left team."""

        self.score_right: Final[int] = score_right
        """The score of the right team."""

    def to_sexp(self) -> str:
        """Return an symbolic expression representing this perception.

        Expression format: (GS (t <play_time>) (pm <play_mode>) (tl <team_left>) (rl <team_right>) (sl <sl>) (sr <sr>))
        """

        left_team_expr = '' if self.team_left is None else '(tl ' + self.team_left + ')'
        right_team_expr = '' if self.team_right is None else '(tr ' + self.team_right + ')'

        return f'(GS (t {self.play_time})(pm {self.play_mode}){left_team_expr}{right_team_expr}(sl {self.score_left})(sr {self.score_right}))'
