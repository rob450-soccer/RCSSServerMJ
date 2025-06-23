import logging
from abc import ABC, abstractmethod
from typing import Final

from rcsssmj.game.soccer_sim_interfaces import PSoccerSimCommandInterface
from rcsssmj.simulation_interfaces import PSimCommandInterface

logger = logging.getLogger(__name__)


class MonitorCommand(ABC):
    """Base class for monitor commands."""

    @abstractmethod
    def perform(self, ci: PSimCommandInterface) -> None:
        """Perform this command.

        Parameter
        ---------
        ci: PSimCommandInterface
            The simulation command interface.
        """


class SoccerMonitorCommand(MonitorCommand):
    """Base class for soccer monitor commands."""

    def perform(self, ci: PSimCommandInterface) -> None:
        if isinstance(ci, PSoccerSimCommandInterface):
            self._perform(ci)
        else:
            logger.warning('Expected soccer simulation instance in soccer monitor command!')

    @abstractmethod
    def _perform(self, sci: PSoccerSimCommandInterface) -> None:
        """Perform this command.

        Parameter
        ---------
        sci: PSoccerSimCommandInterface
            The soccer simulation command interface.
        """


class KickOffCommand(SoccerMonitorCommand):
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

    def _perform(self, sci: PSoccerSimCommandInterface) -> None:
        sci.request_kick_off(self.team_id)
        logger.info('[COMMAND] "kick-off" for team: %d', self.team_id)


class DropBallCommand(SoccerMonitorCommand):
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

    def _perform(self, sci: PSoccerSimCommandInterface) -> None:
        sci.request_drop_ball(self.pos)
        logger.info('[COMMAND] "drop-ball @ %s"', self.pos)


class SetPlayModeCommand(SoccerMonitorCommand):
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

    def _perform(self, sci: PSoccerSimCommandInterface) -> None:
        # sci.request_play_mode(self.play_mode)
        logger.info('[COMMAND] "play-mode": %s', self.play_mode)
