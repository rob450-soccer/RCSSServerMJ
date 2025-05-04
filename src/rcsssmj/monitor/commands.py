import logging
from abc import ABC, abstractmethod
from typing import Any, Final

from rcsssmj.game.referee import SoccerReferee

logger = logging.getLogger(__name__)


class MonitorCommand(ABC):
    """
    Base class for monitor commands.
    """

    @abstractmethod
    def perform(self, referee: SoccerReferee, mj_data: Any) -> None:
        """
        Perform this command.
        """


class KickOffCommand(MonitorCommand):
    """
    The kick-off command.
    """

    def __init__(self, team_id: int) -> None:
        """
        Construct a new kick-off command.
        """

        super().__init__()

        self.team_id: Final[int] = team_id

    def perform(self, referee: SoccerReferee, mj_data: Any) -> None:
        """
        Perform this command.
        """

        del mj_data  # signal unused parameter

        referee.request_kickoff(self.team_id)
        logger.info('[COMMAND] "kick-off" for team: %d', self.team_id)


class DropBallCommand(MonitorCommand):
    """
    The drop-ball command.
    """

    def perform(self, referee: SoccerReferee, mj_data: Any) -> None:
        """
        Perform this command.
        """

        # referee.request_drop_ball()
        logger.info('[COMMAND] "drop-ball"')


class SetPlayModeCommand(MonitorCommand):
    """
    The set-play-mode command.
    """

    def __init__(self, play_mode: str) -> None:
        """
        Construct a new set-play-mode command.
        """

        super().__init__()

        self.play_mode: Final[str] = play_mode

    def perform(self, referee: SoccerReferee, mj_data: Any) -> None:
        """
        Perform this command.
        """

        # referee.request_play_mode(self.play_mode)
        logger.info('[COMMAND] "play-mode": %s', self.play_mode)
