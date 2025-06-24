import logging
from abc import ABC, abstractmethod

from rcsssmj.monitor.commands import KillSimCommand, MonitorCommand
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
            return KillSimCommand()

        logger.debug('Unknown command node: %s', node)

        return None
