import logging
from abc import ABC, abstractmethod
from random import randint

from rcsssmj.monitor.commands import DropBallCommand, KickOffCommand, MonitorCommand, SetPlayModeCommand
from rcsssmj.utils.sexpression import SExpression

logger = logging.getLogger(__name__)


class CommandParser(ABC):
    """Base class for monitor command message parsers."""

    @abstractmethod
    def parse(self, data: bytes | bytearray) -> list[MonitorCommand]:
        """Parse a monitor command message."""


class DefaultCommandParser(CommandParser):
    """Default monitor command message parser implementation based on symbolic expressions."""

    def parse(self, data: bytes | bytearray) -> list[MonitorCommand]:
        """Try parsing a monitor command message containing arbitrary commands."""

        commands: list[MonitorCommand] = []

        # parse individual commands from message
        try:
            node: SExpression = SExpression.from_array(data)

            for child in node.expressions():
                command = self.parse_node(child)
                if command is not None:
                    commands.append(command)

        except Exception:  # noqa: BLE001
            # error while parsing
            logger.debug('Error parsing command message for monitor.', exc_info=True)

        return commands

    def parse_node(self, node: SExpression) -> MonitorCommand | None:
        """Try parsing a monitor command node into a monitor command."""

        if node[0] == b'killsim':
            # shutdown simulation command: (killsim)
            # Note: Relates to simulation server instead of referee... but the referee might trigger a shutdown of the simulation
            return None

        logger.debug('Unknown command node: %s', node)

        return None


class SoccerCommandParser(DefaultCommandParser):
    """Soccer monitor command message parser implementation."""

    def parse_node(self, node: SExpression) -> MonitorCommand | None:
        """Try parsing a soccer monitor command node into a monitor command."""

        n_elements = len(node)

        if node[0] == b'dropBall':
            # drop-ball command: (dropBall)
            return DropBallCommand()

        if node[0] == b'kickOff':
            # kick-off command: (kickOff [Left|Right])
            # random kick-off: (kickOff)
            team_id = randint(0, 1)

            if n_elements > 1 and node[1] == b'Left':
                team_id = 0
            elif n_elements > 1 and node[1] == b'Right':
                team_id = 1

            return KickOffCommand(team_id)

        if node[0] == b'ball':
            # place ball command: (ball (pos <x> <y> <z>) (vel <vx> <vy> <vz>))
            pos: tuple[float, float] | None = None
            for sub_node in node:
                if isinstance(sub_node, SExpression) and sub_node[0] == b'pos':
                    pos = (sub_node.get_float(1), sub_node.get_float(2))
                    break

            return DropBallCommand(pos)

        if node[0] == b'agent':
            # place agent command: (agent (unum <player_number>) (team <team_name>) (pos <x> <y> <z>))
            # alternative command: (agent (unum <player_number>) (team <team_name>) (move <x> <y> <z> <theta>))
            return None

        if node[0] == b'playMode':
            # set play mode command: (playMode <play_mode>)
            return SetPlayModeCommand(node.get_str(1))

        return super().parse_node(node)
