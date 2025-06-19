import logging
from abc import ABC, abstractmethod
from typing import Any, Final

from rcsssmj.game.referee import SoccerReferee

logger = logging.getLogger(__name__)


class MonitorCommand(ABC):
    """Base class for monitor commands."""

    @abstractmethod
    def perform(self, referee: SoccerReferee, mj_data: Any) -> None:
        """Perform this command.

        Parameter
        ---------
        referee: SoccerReferee
            The soccer referee instance.

        mj_data: MjData
            The mujoco simulation data array.
        """


class KickOffCommand(MonitorCommand):
    """The kick-off command."""

    def __init__(self, team_id: int) -> None:
        """Construct a new kick-off command.

        Parameter
        ---------
        team_id: int
            The id of the team for which to grant a kick-off.
        """

        super().__init__()

        self.team_id: Final[int] = team_id
        """The id of the team for which to grant a kick-off."""

    def perform(self, referee: SoccerReferee, mj_data: Any) -> None:
        del mj_data  # signal unused parameter

        referee.kick_off(self.team_id)
        logger.info('[COMMAND] "kick-off" for team: %d', self.team_id)


class DropBallCommand(MonitorCommand):
    """The drop-ball command."""

    def __init__(self, pos: tuple[float, float] | None = None) -> None:
        """Construct a new drop-ball command.

        Parameter
        ---------
        pos: tuple[float, float] | None, default=None
            The position at which to drop the ball. If ``None``, the ball is dropped at it's current position.
        """

        super().__init__()

        self.pos: Final[tuple[float, float] | None] = pos
        """The position at which to drop the ball."""

    def perform(self, referee: SoccerReferee, mj_data: Any) -> None:
        del mj_data  # signal unused parameter

        referee.drop_ball(self.pos)
        logger.info('[COMMAND] "drop-ball @ %s"', self.pos)


class SetPlayModeCommand(MonitorCommand):
    """The set-play-mode command."""

    def __init__(self, play_mode: str) -> None:
        """Construct a new set-play-mode command.

        Parameter
        ---------
        play_mode: str
            The intended play mode.
        """

        super().__init__()

        self.play_mode: Final[str] = play_mode
        """The intended play mode."""

    def perform(self, referee: SoccerReferee, mj_data: Any) -> None:
        del mj_data  # signal unused parameter

        # referee.request_play_mode(self.play_mode)
        logger.info('[COMMAND] "play-mode": %s', self.play_mode)
