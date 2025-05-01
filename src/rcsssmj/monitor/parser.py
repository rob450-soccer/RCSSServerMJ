import logging
from abc import ABC, abstractmethod
from random import randint

from rcsssmj.monitor.commands import DropBallCommand, KickOffCommand, MonitorCommand, SetPlayModeCommand
from rcsssmj.utils.sexpression import SExpression

logger = logging.getLogger(__name__)


class CommandParser(ABC):
    """
    Base class for monitor command message parsers.
    """

    @abstractmethod
    def parse(self, data: bytes | bytearray) -> list[MonitorCommand]:
        """
        Parse a monitor command message.
        """


class SExprCommandParser(CommandParser):
    """
    Default symbolic expression monitor command message parser implementation.
    """

    def parse(self, data: bytes | bytearray) -> list[MonitorCommand]:
        """
        Try parsing a monitor command message containing arbitrary commands.
        """

        commands: list[MonitorCommand] = []

        # parse individual commands from message
        try:
            node: SExpression = SExpression.from_array(data)

            for child in node:
                if not isinstance(child, SExpression):
                    continue

                n_elements = len(child)

                if child[0] == b'dropBall':
                    # drop-ball command: (dropBall)
                    commands.append(DropBallCommand())

                if child[0] == b'kickOff':
                    # kick-off command: (kickOff [Left|Right])
                    # random kick-off: (kickOff)
                    team_id = randint(0, 1)

                    if n_elements > 1 and child[1] == b'Left':
                        team_id = 0
                    elif n_elements > 1 and child[1] == b'Right':
                        team_id = 1

                    commands.append(KickOffCommand(team_id))

                elif child[0] == b'ball':
                    # place ball command: (ball (pos <x> <y> <z>) (vel <vx> <vy> <vz>))
                    pass

                elif child[0] == b'agent':
                    # place agent command: (agent (unum <player_number>) (team <team_name>) (pos <x> <y> <z>))
                    # alternative command: (agent (unum <player_number>) (team <team_name>) (move <x> <y> <z> <theta>))
                    pass

                elif child[0] == b'playMode':
                    # set play mode command: (playMode <play_mode>)
                    commands.append(SetPlayModeCommand(child.get_str(1)))

                elif child[0] == b'killsim':
                    # shutdown simulation command: (killsim)
                    # Note: Relates to simulation server instead of referee... but the referee might trigger a shutdown of the simulation
                    pass

        except Exception:  # noqa: BLE001
            # error while parsing
            logger.debug('Error parsing command message for monitor.', exc_info=True)

        return commands
